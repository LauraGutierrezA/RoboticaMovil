#!/usr/bin/python3
"""
volante_web.py — El celular como volante: interfaz web por IP que reemplaza a teleop_twist_keyboard.

    Celular (navegador)                         PC / robot
    ┌─────────────────────────┐   HTTPS (LAN)   ┌──────────────────┐
    │ inclinación (acelerómetro)│ ── POST /cmd ─► │ volante_web.py   │ ─► /cmd_vel_teleop ─► aeb_mux ─► /cmd_vel
    │ botón ADELANTE / REVERSA │   20 veces/s    │ (este nodo)      │
    └─────────────────────────┘                 └──────────────────┘

El celular se sostiene horizontal, como un volante, y se gira en el plano de la pantalla.
El ángulo se calcula con la gravedad (acelerómetro) en el plano de la pantalla:
    θ = atan2(g_x, g_y)          →  giro = −(θ − θ_0), con θ_0 tomado al presionar "Calibrar"
(girar el celular a la derecha hace girar la gravedad al revés en el marco del celular, de ahí
el signo). Da igual cómo esté orientado el celular o si es iOS/Android (iOS invierte los dos
ejes, lo que solo suma 180°): la calibración define el "recto". Si en algún celular el giro
sale al revés, el botón "Invertir" de la página lo corrige.

Comando:
    v = ±v_max · acelerador        (solo mientras se mantiene presionado ADELANTE o REVERSA)
    ω = −w_max · giro_normalizado  (giro a la izquierda → ω > 0, convención de ROS)
    En reversa ω se invierte para que se maneje como un carro (opción `reversa_como_carro`).

Seguridad:
  - aeb_mux.py sigue teniendo prioridad (el celular publica en /cmd_vel_teleop, no en /cmd_vel).
  - aeb_mux usa timeout_teleop = 0 (mantiene el último comando), así que el que corta es ESTE
    nodo: si el celular deja de enviar más de `timeout` s (se cae el WiFi, se bloquea la
    pantalla, se cierra la pestaña), publica cero.

Por qué HTTPS: los navegadores solo entregan los datos del acelerómetro/giroscopio en un
"contexto seguro" (HTTPS o localhost). Por eso el nodo genera un certificado autofirmado la
primera vez (requiere `openssl`) y el celular muestra una advertencia que hay que aceptar una vez
("Avanzado → Continuar").

Uso:
  ros2 launch mi_robot_diferencial robot_yahboom.launch.py           # simulación (modo teleop)
  ros2 run mi_robot_diferencial volante_web.py                       # en otra terminal
  → abrir en el celular  https://<IP del PC>:8443   (misma red WiFi; la IP sale en el log)

Solo usa la librería estándar de Python + rclpy (no hay que instalar nada).
"""
import json
import math
import os
import socket
import ssl
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist


class NodoVolanteWeb(Node):
    def __init__(self):
        super().__init__('volante_web')
        dp = self.declare_parameter
        dp('puerto', 8443)
        dp('topico', '/cmd_vel_teleop')
        dp('v_max', 0.6)                 # [m/s] tope del acelerador (el DiffDrive limita a 1.0)
        dp('w_max', 4.0)                 # [rad/s] = w_max de parametros_microros.xacro
        dp('angulo_max', 45.0)           # [°] inclinación del celular que da ω = w_max
        dp('zona_muerta', 4.0)           # [°] alrededor del recto, para que no "tiemble"
        dp('reversa_como_carro', True)   # en reversa invertir ω, como un carro
        dp('timeout', 0.3)               # [s] sin datos del celular → cero
        dp('frecuencia', 20.0)           # [Hz]
        dp('carpeta_cert', '~/.volante_web')
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        self.v_max, self.w_max = gp('v_max'), gp('w_max')
        self.angulo_max, self.zona_muerta = gp('angulo_max'), gp('zona_muerta')
        self.reversa_como_carro = gp('reversa_como_carro')
        self.timeout = gp('timeout')

        self.lock = threading.Lock()
        self.ultimo = None        # dict con lo que mandó el celular
        self.t_ultimo = 0.0       # time.monotonic() del último dato
        self.conectado = False

        self.pub = self.create_publisher(Twist, gp('topico'), 10)
        self.create_timer(1.0 / gp('frecuencia'), self.cb_timer)

        cert, key = self.certificado(os.path.expanduser(gp('carpeta_cert')))
        self.servidor = ThreadingHTTPServer(('0.0.0.0', gp('puerto')), self.handler())
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(cert, key)
        self.servidor.socket = ctx.wrap_socket(self.servidor.socket, server_side=True)
        threading.Thread(target=self.servidor.serve_forever, daemon=True).start()

        self.get_logger().info(
            f"Volante listo → abrir en el celular: https://{ip_local()}:{gp('puerto')}  "
            f"(publica en {gp('topico')}, v_max {self.v_max} m/s, w_max {self.w_max} rad/s)")

    # ---------- comando ----------
    def calcular(self, d):
        """Del estado del celular a (v, ω)."""
        giro = float(d.get('giro', 0.0))                      # [°], positivo = a la derecha
        if abs(giro) < self.zona_muerta:
            giro = 0.0
        else:
            giro = math.copysign(abs(giro) - self.zona_muerta, giro)
        g = max(-1.0, min(1.0, giro / (self.angulo_max - self.zona_muerta)))

        acel = max(0.0, min(1.0, float(d.get('acelerador', 1.0))))
        sentido = 1.0 if d.get('adelante') else (-1.0 if d.get('reversa') else 0.0)
        v = sentido * self.v_max * acel
        w = -self.w_max * g                                    # derecha → ω < 0
        if v < 0.0 and self.reversa_como_carro:
            w = -w
        return v, w

    def cb_timer(self):
        with self.lock:
            d, t = self.ultimo, self.t_ultimo
        vivo = d is not None and time.monotonic() - t <= self.timeout
        if vivo != self.conectado:
            if vivo:
                self.get_logger().info('Celular conectado')
            else:
                self.get_logger().warn('Sin datos del celular: robot detenido')
                self.pub.publish(Twist())                      # una sola vez: cero
            self.conectado = vivo
        if not vivo:
            return                                             # no pelear con otro teleop
        msg = Twist()
        msg.linear.x, msg.angular.z = self.calcular(d)
        self.pub.publish(msg)

    # ---------- servidor web ----------
    def handler(self):
        nodo = self

        class H(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'                      # keep-alive: no un TLS por mensaje

            def responder(self, codigo, cuerpo, tipo):
                self.send_response(codigo)
                self.send_header('Content-Type', tipo)
                self.send_header('Content-Length', str(len(cuerpo)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(cuerpo)

            def do_GET(self):
                if self.path in ('/', '/index.html'):
                    self.responder(200, PAGINA.encode(), 'text/html; charset=utf-8')
                else:
                    self.responder(404, b'', 'text/plain')

            def do_POST(self):
                if self.path != '/cmd':
                    self.responder(404, b'', 'text/plain')
                    return
                try:
                    n = int(self.headers.get('Content-Length', 0))
                    d = json.loads(self.rfile.read(n) or b'{}')
                    with nodo.lock:
                        nodo.ultimo, nodo.t_ultimo = d, time.monotonic()
                    v, w = nodo.calcular(d)
                    self.responder(200, json.dumps({'v': v, 'w': w}).encode(), 'application/json')
                except (ValueError, TypeError):
                    self.responder(400, b'', 'text/plain')

            def log_message(self, *a):                         # sin un log por petición
                pass

        return H

    def certificado(self, carpeta):
        """Certificado autofirmado para HTTPS (se genera una vez)."""
        cert, key = os.path.join(carpeta, 'cert.pem'), os.path.join(carpeta, 'key.pem')
        if not (os.path.exists(cert) and os.path.exists(key)):
            os.makedirs(carpeta, exist_ok=True)
            self.get_logger().info(f'Generando certificado autofirmado en {carpeta}')
            subprocess.run(['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes',
                            '-keyout', key, '-out', cert, '-days', '365',
                            '-subj', '/CN=volante-robot'],
                           check=True, capture_output=True)
        return cert, key

    def cerrar(self):
        self.servidor.shutdown()


def ip_local():
    """IP del PC en la red WiFi (la que se escribe en el celular)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))                       # no envía nada; solo elige la interfaz
        return s.getsockname()[0]
    except OSError:
        return '127.0.0.1'
    finally:
        s.close()


PAGINA = r"""<!DOCTYPE html>
<html lang="es"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>Volante</title>
<style>
  * { box-sizing: border-box; -webkit-user-select: none; user-select: none;
      -webkit-touch-callout: none; touch-action: none; }
  html, body { margin: 0; height: 100%; background: #111; color: #eee;
               font-family: system-ui, sans-serif; overflow: hidden; }
  #app { display: grid; grid-template-columns: 1fr 1.2fr 1fr; height: 100%; gap: 10px; padding: 10px; }
  .pedal { border: none; border-radius: 18px; font-size: 1.6rem; font-weight: 700; color: #fff; }
  #rev { background: #8a2a2a; } #fwd { background: #2a7a3a; }
  .pedal.on { filter: brightness(1.6); transform: scale(.97); }
  #centro { display: flex; flex-direction: column; align-items: center; justify-content: space-around; }
  #volante { width: 46vmin; height: 46vmin; border: 3vmin solid #ddd; border-radius: 50%;
             position: relative; transition: transform 40ms linear; }
  #volante::before { content: ''; position: absolute; left: 50%; top: -3vmin; width: 2vmin;
                     height: 9vmin; margin-left: -1vmin; background: #e33; border-radius: 1vmin; }
  #volante::after { content: ''; position: absolute; left: 0; right: 0; top: 50%; height: 3vmin;
                    margin-top: -1.5vmin; background: #ddd; }
  #info { font: 1rem ui-monospace, monospace; text-align: center; }
  .fila { display: flex; gap: 10px; align-items: center; }
  .btn { background: #333; color: #eee; border: 1px solid #666; border-radius: 10px;
         padding: 10px 16px; font-size: 1rem; }
  input[type=range] { width: 30vw; touch-action: auto; }
  #inicio { position: fixed; inset: 0; background: #111e; display: flex; flex-direction: column;
            align-items: center; justify-content: center; gap: 16px; text-align: center; padding: 20px; }
  #inicio button { font-size: 1.4rem; padding: 16px 28px; }
  #estado.ok { color: #6d6; } #estado.mal { color: #f66; }
</style></head>
<body>
<div id="app">
  <button class="pedal" id="rev">◀ REVERSA</button>
  <div id="centro">
    <div id="volante"></div>
    <div id="info"><span id="estado">—</span><br><span id="val">v 0.00 · ω 0.00</span></div>
    <div class="fila">
      <button class="btn" id="cal">Calibrar recto</button>
      <button class="btn" id="inv">Invertir</button>
      <label>Acel <input type="range" id="acel" min="0.1" max="1" step="0.05" value="0.6"></label>
    </div>
  </div>
  <button class="pedal" id="fwd">ADELANTE ▶</button>
</div>
<div id="inicio">
  <div>Gira el celular horizontal y sostenlo como un volante.</div>
  <button class="btn" id="go">Empezar</button>
  <div id="aviso" style="color:#f99"></div>
</div>
<script>
const $ = id => document.getElementById(id);
let theta = null, theta0 = 0, giro = 0, signo = 1, adelante = false, reversa = false;

// Ángulo en el plano de la pantalla a partir de la gravedad. Los signos cambian entre
// iOS y Android, pero eso solo suma 180° y lo absorbe la calibración.
function onMotion(e) {
  const g = e.accelerationIncludingGravity;
  if (!g || g.x === null) return;
  theta = Math.atan2(g.x, g.y) * 180 / Math.PI;
  let d = theta - theta0;
  d = ((d + 540) % 360) - 180;                 // a [-180, 180)
  giro = -d * signo;                          // girar el celular a la derecha → giro > 0
  $('volante').style.transform = `rotate(${giro}deg)`;
}
function calibrar() { if (theta !== null) theta0 = theta; }

async function empezar() {
  if (!window.isSecureContext) {
    $('aviso').textContent = 'Hay que abrir la página con https:// para leer los sensores.';
    return;
  }
  try {   // iOS pide permiso explícito tras un toque
    if (typeof DeviceMotionEvent !== 'undefined' && DeviceMotionEvent.requestPermission) {
      if (await DeviceMotionEvent.requestPermission() !== 'granted') throw 0;
    }
  } catch (_) { $('aviso').textContent = 'Sin permiso para los sensores de movimiento.'; return; }
  window.addEventListener('devicemotion', onMotion);
  try { await document.documentElement.requestFullscreen(); } catch (_) {}
  try { await screen.orientation.lock('landscape'); } catch (_) {}
  try { await navigator.wakeLock.request('screen'); } catch (_) {}
  $('inicio').style.display = 'none';
  setTimeout(calibrar, 300);                  // la posición al empezar es el "recto"
  setInterval(enviar, 50);                    // 20 Hz
}

function pedal(id, set) {
  const b = $(id);
  const on  = e => { e.preventDefault(); set(true);  b.classList.add('on'); };
  const off = e => { e.preventDefault(); set(false); b.classList.remove('on'); };
  b.addEventListener('pointerdown', on);
  ['pointerup', 'pointercancel', 'pointerleave'].forEach(t => b.addEventListener(t, off));
}
pedal('fwd', x => adelante = x);
pedal('rev', x => reversa = x);
$('cal').onclick = calibrar;
$('inv').onclick = () => { signo = -signo; };
$('go').onclick = empezar;
document.addEventListener('visibilitychange', () => { adelante = reversa = false; });

let enVuelo = false;
async function enviar() {
  if (enVuelo) return;                        // no acumular peticiones si la red está lenta
  enVuelo = true;
  try {
    const r = await fetch('/cmd', { method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({ giro, adelante, reversa, acelerador: +$('acel').value }) });
    const c = await r.json();
    $('val').textContent = `v ${c.v.toFixed(2)} · ω ${c.w.toFixed(2)}`;
    $('estado').textContent = 'conectado'; $('estado').className = 'ok';
  } catch (_) {
    $('estado').textContent = 'sin conexión'; $('estado').className = 'mal';
  }
  enVuelo = false;
}
</script>
</body></html>
"""


def main():
    rclpy.init()
    nodo = NodoVolanteWeb()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            nodo.pub.publish(Twist())
        except Exception:  # noqa: BLE001
            pass
        nodo.cerrar()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
