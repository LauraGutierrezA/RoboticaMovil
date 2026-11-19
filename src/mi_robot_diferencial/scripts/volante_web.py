#!/usr/bin/python3
"""
volante_web.py — Consola web en el celular: volante, odometría en vivo y tuning del wall_follow.

    Celular (navegador, HTTPS)                         PC
    ┌───────────────────────────────┐                 ┌─────────────────────┐
    │ 1 Inicio       (modo, guía)    │  POST /modo     │                     │ ─► `modo` de /aeb_mux, `activo` de /wall_follow
    │ 2 Calibración  (no mueve)      │  POST /cmd 20Hz │                     │ ─► /cmd_vel_teleop ─► aeb_mux
    │ 3 Operación    (cámara, mueve) │ ──────────────► │   volante_web.py    │ ◄─ /image_raw (GET /video, MJPEG)
    │ 4 Odometría    (gráficas)      │  GET /datos     │   (este nodo)       │ ◄─ /ground_truth /odom /odom_llantas
    │ 5 Wall following (tuning)      │ ◄────────────── │                     │ ◄─ /wall/* /aeb/* /vuelta
    └───────────────────────────────┘  POST /wall     └─────────────────────┘ ─► parámetros de /wall_follow

Pestañas:
  1. Inicio: selector de modo (Manual / Autónomo / Detener autónomo, sin relanzar: cambia `modo`
     de /aeb_mux y `activo` de /wall_follow), estado general y guía de uso.
  2. Calibración: el volante reacciona al celular pero NO publica nada. "Calibrar recto",
     sensibilidad (ángulo de giro que da ω máx), zona muerta e invertir. Se guardan en el celular.
  3. Operación: la cámara del robot (/image_raw, MJPEG) de fondo, con los pedales Reversa /
     Adelante translúcidos encima. Solo aquí se publica en /cmd_vel_teleop (y solo mientras la
     pestaña está abierta y llegan datos). Inclinar el celular gira.
  4. Odometría: error de posición y de orientación de la odometría propia (/odom_llantas,
     ω_I y ω_D) y del DiffDrive (/odom) frente a la pose real (/ground_truth), en vivo, más la
     trayectoria. /odom arranca en (0,0,0): se lleva al marco del mundo con la pose real
     tomada la primera vez que el robot está quieto.
  5. Wall following: parámetros de /wall_follow en caliente (cliente de parámetros, igual que
     `ros2 param set`), e(t), D_t, vueltas y frenos del AEB.

Ángulo del volante: θ = atan2(g_x, g_y) con la gravedad en el plano de la pantalla;
giro = −(θ − θ_0) con θ_0 de "Calibrar recto". iOS invierte los ejes (suma 180°): lo absorbe
la calibración. "Invertir" corrige el sentido si hace falta.
    v = ±v_max·acelerador ;  ω = −w_max·clip((|giro| − zona_muerta)/(sensibilidad − zona_muerta))
    (en reversa ω se invierte, como un carro: reversa_como_carro)

Seguridad: aeb_mux sigue teniendo prioridad (este nodo publica en /cmd_vel_teleop). Si el
celular deja de enviar más de `timeout` s, o sale de la pestaña Operación, publica cero UNA vez
y deja de publicar (no pelea con el teclado).

HTTPS: los navegadores solo entregan el acelerómetro en contexto seguro. El nodo genera un
certificado autofirmado la primera vez (requiere `openssl`); en el celular hay que aceptar la
advertencia una vez.

Uso:
  ros2 launch mi_robot_diferencial robot_yahboom.launch.py pista:=competencia volante:=true
  ros2 launch mi_robot_diferencial robot_yahboom.launch.py pista:=competencia modo:=auto volante:=true
  → abrir en el celular  https://<IP del PC>:8443   (misma red WiFi; la IP sale en el log)

Solo librería estándar + rclpy.
"""
import bisect
import json
import math
import os
import socket
import ssl
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, Int32
import numpy as np

try:                                   # compresión JPEG para la cámara
    import cv2
except ImportError:                    # noqa: E722
    cv2 = None
try:
    from PIL import Image as PILImage
except ImportError:
    PILImage = None


# Parámetros de /wall_follow que se pueden tunear desde el celular: (nombre, tipo, paso, ayuda)
WALL_PARAMS = [
    ('Control', [
        ('kp', float, 0.25, 'Ganancia proporcional [rad/s por m]'),
        ('ki', float, 0.1, 'Integral: quita el error en curvas largas (0 = PD)'),
        ('kd', float, 0.05, 'Derivativa: amortigua oscilaciones'),
        ('filtro_d', float, 0.1, 'Filtro de ė: 0 = nada, 0.9 = mucho'),
    ]),
    ('Geometría', [
        ('L', float, 0.05, 'Proyección hacia adelante [m]'),
        ('d_des', float, 0.01, 'Distancia a la pared interior [m]'),
        ('theta_deg', float, 5.0, 'Separación entre haces a y b [°]'),
        ('frac_centro', float, 0.05, 'd_obj ≤ frac·ancho (0.5 = centrarse si es angosto)'),
    ]),
    ('Velocidad', [
        ('v_max', float, 0.05, 'Velocidad máxima [m/s] (el robot llega a 1.0)'),
        ('v_min', float, 0.05, 'Velocidad mínima [m/s]'),
        ('w_max', float, 0.25, 'Saturación de ω [rad/s]; mayor = frena menos al girar'),
        ('v_estrecho', float, 0.05, 'Tope de v en el estrechamiento [m/s]'),
        ('ancho_estrecho', float, 0.05, 'Ancho por debajo del cual es estrechamiento [m]'),
        ('k_frente', float, 0.25, 'v ≤ k·(d_frente − d_seg) [1/s]'),
        ('d_seg', float, 0.05, 'Distancia al frente donde v llega a v_min [m]'),
    ]),
]
TIPOS = {n: t for _, g in WALL_PARAMS for n, t, _, _ in g}
TIPOS['activo'] = bool


def yaw_de_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def envolver(a):
    return math.atan2(math.sin(a), math.cos(a))


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class NodoVolanteWeb(Node):
    def __init__(self):
        super().__init__('volante_web')
        dp = self.declare_parameter
        dp('puerto', 8443)
        dp('topico', '/cmd_vel_teleop')
        dp('v_max', 0.6)                 # [m/s] tope del acelerador (el DiffDrive limita a 1.0)
        dp('w_max', 4.0)                 # [rad/s] = w_max de parametros_microros.xacro
        dp('reversa_como_carro', True)
        dp('timeout', 0.3)               # [s] sin datos del celular → cero
        dp('frecuencia', 20.0)           # [Hz]
        dp('muestreo_odom', 5.0)         # [Hz] puntos de la gráfica de odometría
        dp('b_ruedas', 0.1315)           # [m] vía geométrica (para sugerir b_ef = b/k)
        dp('carpeta_cert', '~/.volante_web')
        dp('cam_topico', '/image_raw')
        dp('cam_fps', 15.0)              # [Hz] tope de cuadros enviados al celular
        dp('cam_calidad', 70)            # calidad JPEG (0-100)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.v_max, self.w_max = gp('v_max'), gp('w_max')
        self.reversa_como_carro = gp('reversa_como_carro')
        self.timeout = gp('timeout')
        self.dt_odom = 1.0 / gp('muestreo_odom')

        self.lock = threading.Lock()
        # --- celular
        self.ultimo, self.t_ultimo = None, 0.0
        self.publicando = False
        self.cmd_out = (0.0, 0.0)
        # --- odometría
        self.gt_hist = deque(maxlen=200)          # (t, x, y, psi) para interpolar
        self.dist = 0.0
        self.pose0_dd = None                      # pose real cuando /odom era (0,0,0)
        self.dd = self.ll = self.sk = None        # última pose de /odom (mundo), /odom_llantas, /odom_skid
        self.err_prev = {}                        # para "desenvolver" el error de orientación
        self.w_gt = 0.0                           # ω real (/ground_truth)
        self.sxy = self.sxx = 0.0                 # mínimos cuadrados ω_real ≈ k·ω_ruedas
        self.n_k = 0
        self.b_ruedas = gp('b_ruedas')
        self.v_odom = 0.0
        self.t_ult_muestra = -math.inf
        self.odom_pts, self.odom_base = [], 0     # puntos de la gráfica (con índice absoluto)
        # --- wall following / AEB / vueltas
        self.wall_pts, self.wall_base = [], 0
        self.wall_ult = {}
        self.aeb_activo, self.aeb_cuenta, self.ttc = False, 0, math.inf
        self.vueltas = []                         # (k, t_sim)
        self.modo = None                          # teleop | auto (parámetro de /aeb_mux)
        self.wall_activo = None
        # --- cámara
        self.cam_cond = threading.Condition()
        self.cam_msg, self.cam_n = None, 0
        self.cam_fps, self.cam_q = gp('cam_fps'), int(gp('cam_calidad'))
        self.cam_clientes = 0

        self.pub = self.create_publisher(Twist, gp('topico'), 10)
        sd = qos_profile_sensor_data
        self.create_subscription(Odometry, '/ground_truth', self.cb_gt, sd)
        self.create_subscription(Odometry, '/odom', self.cb_dd, sd)
        self.create_subscription(Odometry, '/odom_llantas', self.cb_ll, sd)
        self.create_subscription(Odometry, '/odom_skid', self.cb_sk, sd)
        self.create_subscription(Bool, '/aeb/activo', self.cb_aeb, 10)
        self.create_subscription(Float64, '/aeb/ttc_min', self.cb_ttc, 10)
        self.create_subscription(Int32, '/vuelta', self.cb_vuelta, 10)
        self.create_subscription(Image, gp('cam_topico'), self.cb_cam, sd)
        for n in ('error', 'dist', 'd_obj'):
            self.create_subscription(Float64, f'/wall/{n}', lambda m, n=n: self.cb_wall(n, m), 10)
        self.pc_wall = AsyncParameterClient(self, 'wall_follow')
        self.pc_mux = AsyncParameterClient(self, 'aeb_mux')
        self.create_timer(1.0 / gp('frecuencia'), self.cb_timer)
        self.create_timer(1.0, self.consultar_modo)

        cert, key = self.certificado(os.path.expanduser(gp('carpeta_cert')))
        self.servidor = ThreadingHTTPServer(('0.0.0.0', gp('puerto')), self.handler())
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(cert, key)
        self.servidor.socket = ctx.wrap_socket(self.servidor.socket, server_side=True)
        threading.Thread(target=self.servidor.serve_forever, daemon=True).start()
        self.get_logger().info(
            f"Consola lista → abrir en el celular: https://{ip_local()}:{gp('puerto')}  "
            f"(publica en {gp('topico')} solo desde la pestaña Operación)")

    # ================================================================ volante
    def calcular(self, d):
        """Del estado del celular a (v, ω)."""
        sens = max(10.0, min(90.0, float(d.get('sensibilidad', 45.0))))
        zm = max(0.0, min(sens - 5.0, float(d.get('zona_muerta', 4.0))))
        giro = float(d.get('giro', 0.0))                      # [°], positivo = a la derecha
        giro = 0.0 if abs(giro) < zm else math.copysign(abs(giro) - zm, giro)
        g = max(-1.0, min(1.0, giro / (sens - zm)))
        acel = max(0.0, min(1.0, float(d.get('acelerador', 1.0))))
        sentido = 1.0 if d.get('adelante') else (-1.0 if d.get('reversa') else 0.0)
        v = sentido * self.v_max * acel
        w = -self.w_max * g
        if v < 0.0 and self.reversa_como_carro:
            w = -w
        return v, w

    def cb_timer(self):
        with self.lock:
            d, t = self.ultimo, self.t_ultimo
        operar = (d is not None and d.get('operar') is True
                  and time.monotonic() - t <= self.timeout)
        if operar != self.publicando:
            self.get_logger().info('Operación desde el celular' if operar
                                   else 'Celular fuera de Operación / sin datos: robot detenido')
            if not operar:
                self.pub.publish(Twist())                      # una sola vez: cero
                self.cmd_out = (0.0, 0.0)
            self.publicando = operar
        if not operar:
            return
        msg = Twist()
        msg.linear.x, msg.angular.z = self.calcular(d)
        self.cmd_out = (msg.linear.x, msg.angular.z)
        self.pub.publish(msg)

    # ================================================================ odometría
    def gt_en(self, t):
        h = self.gt_hist
        if not h or t < h[0][0] or t > h[-1][0]:
            return None
        ts = [p[0] for p in h]
        i = bisect.bisect_left(ts, t)
        if i == 0 or ts[i] == t:
            return h[i][1:]
        (t0, x0, y0, p0), (t1, x1, y1, p1) = h[i - 1], h[i]
        f = (t - t0) / (t1 - t0)
        return x0 + f * (x1 - x0), y0 + f * (y1 - y0), envolver(p0 + f * envolver(p1 - p0))

    def cb_gt(self, m):
        t, p = stamp_s(m.header), m.pose.pose
        x, y, psi = p.position.x, p.position.y, yaw_de_quat(p.orientation)
        if self.gt_hist:
            self.dist += math.hypot(x - self.gt_hist[-1][1], y - self.gt_hist[-1][2])
        if not self.gt_hist or t > self.gt_hist[-1][0]:
            self.gt_hist.append((t, x, y, psi))
        self.w_gt = m.twist.twist.angular.z

    def cb_ll(self, m):
        p = m.pose.pose
        self.ll = (p.position.x, p.position.y, yaw_de_quat(p.orientation))

    def cb_sk(self, m):
        p = m.pose.pose
        self.sk = (p.position.x, p.position.y, yaw_de_quat(p.orientation))

    def cb_dd(self, m):
        t, p, tw = stamp_s(m.header), m.pose.pose, m.twist.twist
        xo, yo, po = p.position.x, p.position.y, yaw_de_quat(p.orientation)
        self.v_odom = tw.linear.x
        real = self.gt_en(t)
        if real is None:
            return
        if self.pose0_dd is None:
            if abs(tw.linear.x) > 0.01 or abs(tw.angular.z) > 0.01:
                return                       # anclar /odom al mundo solo con el robot quieto
            p0 = envolver(real[2] - po)
            self.pose0_dd = (real[0] - (math.cos(p0) * xo - math.sin(p0) * yo),
                             real[1] - (math.sin(p0) * xo + math.cos(p0) * yo), p0)
        x0, y0, p0 = self.pose0_dd
        dd = (x0 + math.cos(p0) * xo - math.sin(p0) * yo,
              y0 + math.sin(p0) * xo + math.cos(p0) * yo, envolver(p0 + po))
        self.dd = dd
        if t - self.t_ult_muestra < self.dt_odom:
            return
        self.t_ult_muestra = t

        # k en vivo: ω_real ≈ k·ω_ruedas (ω de /odom = a(ωD−ωI)/b, sin deslizamiento)
        if abs(tw.angular.z) > 0.2:
            self.sxy += self.w_gt * tw.angular.z
            self.sxx += tw.angular.z ** 2
            self.n_k += 1

        def err(clave, est):
            if est is None:
                return None, None
            # orientación "desenvuelta": sigue creciendo más allá de ±180° (sin saltos)
            crudo = math.degrees(envolver(est[2] - real[2]))
            prev = self.err_prev.get(clave, crudo)
            psi_e = prev + math.degrees(envolver(math.radians(crudo - prev)))
            self.err_prev[clave] = psi_e
            return (round(math.hypot(est[0] - real[0], est[1] - real[1]) * 1000, 1), round(psi_e, 2))
        e_ll, s_ll = err('l', self.ll)
        e_dd, s_dd = err('d', dd)
        e_sk, s_sk = err('k', self.sk)
        r3 = lambda p, i: round(p[i], 3) if p else None  # noqa: E731
        pt = dict(t=round(t, 2), d=round(self.dist, 3),
                  xr=round(real[0], 3), yr=round(real[1], 3),
                  xl=r3(self.ll, 0), yl=r3(self.ll, 1), xd=round(dd[0], 3), yd=round(dd[1], 3),
                  xk=r3(self.sk, 0), yk=r3(self.sk, 1),
                  el=e_ll, sl=s_ll, ed=e_dd, sd=s_dd, ek=e_sk, sk=s_sk)
        with self.lock:
            self.odom_pts.append(pt)
            if len(self.odom_pts) > 6000:                     # 20 min a 5 Hz
                del self.odom_pts[:1000]
                self.odom_base += 1000

    # ================================================================ wall / AEB / vueltas
    def cb_wall(self, n, m):
        self.wall_ult[n] = m.data
        if n != 'error' or not self.gt_hist:
            return
        def cm(x):
            return round(x * 100, 1) if x is not None and math.isfinite(x) else None
        pt = dict(t=round(self.gt_hist[-1][0], 2), e=cm(m.data),
                  D=cm(self.wall_ult.get('dist')), o=cm(self.wall_ult.get('d_obj')))
        with self.lock:
            self.wall_pts.append(pt)
            if len(self.wall_pts) > 3000:
                del self.wall_pts[:500]
                self.wall_base += 500

    def cb_aeb(self, m):
        if m.data and not self.aeb_activo:
            self.aeb_cuenta += 1
        self.aeb_activo = m.data

    def cb_ttc(self, m):
        self.ttc = m.data

    def cb_vuelta(self, m):
        t = self.gt_hist[-1][0] if self.gt_hist else math.nan
        with self.lock:
            if m.data == 0:
                self.vueltas = []
            self.vueltas.append((m.data, round(t, 2)))

    def consultar_modo(self):
        # el modo puede cambiar desde aquí o con ros2 param set: se consulta cada segundo
        if self.pc_mux.services_are_ready():
            def listo_mux(f):
                try:
                    self.modo = parameter_value_to_python(f.result().values[0])
                except Exception:  # noqa: BLE001
                    pass
            self.pc_mux.get_parameters(['modo']).add_done_callback(listo_mux)
        if self.pc_wall.services_are_ready():
            def listo_wall(f):
                try:
                    self.wall_activo = parameter_value_to_python(f.result().values[0])
                except Exception:  # noqa: BLE001
                    pass
            self.pc_wall.get_parameters(['activo']).add_done_callback(listo_wall)
        else:
            self.wall_activo = None

    def cambiar_modo(self, modo):
        """Manual <-> autónomo sin relanzar: `modo` de /aeb_mux y `activo` de /wall_follow."""
        if modo not in ('teleop', 'auto'):
            raise ValueError("modo debe ser 'teleop' o 'auto'")
        if not self.pc_mux.services_are_ready():
            raise RuntimeError('/aeb_mux no responde')
        if modo == 'auto':
            # primero el mux (el robot queda en cero hasta que wall_follow publique), luego activar
            self.esperar(self.pc_mux.set_parameters([Parameter('modo', value='auto')]))
            self.wall_escribir({'activo': True})
        else:
            # primero detener el wall_follow, luego devolver el mux al teclado/volante
            if self.pc_wall.services_are_ready():
                self.wall_escribir({'activo': False})
            self.esperar(self.pc_mux.set_parameters([Parameter('modo', value='teleop')]))
        self.modo = modo
        self.wall_activo = modo == 'auto'
        self.get_logger().warn(f'Modo cambiado desde la consola: {modo.upper()}')

    # ================================================================ cámara
    def cb_cam(self, m):
        if self.cam_clientes <= 0:
            return                               # nadie mirando: no guardar
        with self.cam_cond:
            self.cam_msg, self.cam_n = m, self.cam_n + 1
            self.cam_cond.notify_all()

    def jpeg(self, m):
        ch = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(m.encoding)
        if ch is None:
            raise ValueError(f'encoding no soportado: {m.encoding}')
        buf = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)[:, :m.width * ch]
        img = buf.reshape(m.height, m.width, ch) if ch > 1 else buf.reshape(m.height, m.width)
        if cv2 is not None:
            if m.encoding == 'rgb8':
                img = img[:, :, ::-1]
            elif m.encoding == 'rgba8':
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            elif m.encoding == 'bgra8':
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            ok, j = cv2.imencode('.jpg', np.ascontiguousarray(img), [cv2.IMWRITE_JPEG_QUALITY, self.cam_q])
            return j.tobytes()
        if PILImage is not None:
            import io
            if m.encoding in ('bgr8', 'bgra8'):
                img = img[:, :, 2::-1]
            elif ch == 4:
                img = img[:, :, :3]
            out = io.BytesIO()
            PILImage.fromarray(np.ascontiguousarray(img)).save(out, 'JPEG', quality=self.cam_q)
            return out.getvalue()
        raise RuntimeError('no hay OpenCV ni Pillow: sudo apt install python3-opencv')

    def esperar(self, fut, timeout=2.0):
        ev = threading.Event()
        fut.add_done_callback(lambda _: ev.set())
        if not ev.wait(timeout):
            raise TimeoutError('sin respuesta de /wall_follow')
        return fut.result()

    def wall_leer(self):
        if not self.pc_wall.services_are_ready():
            raise RuntimeError('/wall_follow no está corriendo (lanza con modo:=auto)')
        nombres = list(TIPOS)
        r = self.esperar(self.pc_wall.get_parameters(nombres))
        return {n: parameter_value_to_python(v) for n, v in zip(nombres, r.values)}

    def wall_escribir(self, cambios):
        if not self.pc_wall.services_are_ready():
            raise RuntimeError('/wall_follow no está corriendo (lanza con modo:=auto)')
        prm = []
        for n, v in cambios.items():
            if n not in TIPOS:
                raise ValueError(f'parámetro desconocido: {n}')
            prm.append(Parameter(n, value=TIPOS[n](v)))
        r = self.esperar(self.pc_wall.set_parameters(prm))
        malos = [f'{p.name}: {res.reason}' for p, res in zip(prm, r.results) if not res.successful]
        if malos:
            raise RuntimeError('; '.join(malos))
        self.get_logger().info('wall_follow ← ' + ', '.join(f'{p.name}={p.value}' for p in prm))

    # ================================================================ /datos
    def datos(self, desde_odom, desde_wall):
        with self.lock:
            io = max(0, desde_odom - self.odom_base)
            iw = max(0, desde_wall - self.wall_base)
            od = self.odom_pts[io:]
            wl = self.wall_pts[iw:]
            n_od = self.odom_base + len(self.odom_pts)
            n_wl = self.wall_base + len(self.wall_pts)
            vueltas = list(self.vueltas)
        def num(x):  # noqa: E306
            return None if x is None or not math.isfinite(x) else round(x, 3)
        return dict(
            modo=self.modo, publicando=self.publicando, wall_activo=self.wall_activo,
            cmd=[round(self.cmd_out[0], 2), round(self.cmd_out[1], 2)],
            v_odom=round(self.v_odom, 2), dist=round(self.dist, 2),
            aeb=self.aeb_activo, aeb_cuenta=self.aeb_cuenta, ttc=num(self.ttc),
            vueltas=vueltas, wall_ok=self.pc_wall.services_are_ready(),
            k=num(self.sxy / self.sxx) if self.sxx > 0 else None, n_k=self.n_k,
            b_ef=num(self.b_ruedas * self.sxx / self.sxy) if self.sxy > 0 else None,
            odom=od, n_odom=n_od, wall=wl, n_wall=n_wl,
            reinicio_odom=desde_odom > n_od, reinicio_wall=desde_wall > n_wl)

    def reiniciar_graficas(self):
        with self.lock:
            self.odom_base += len(self.odom_pts) + 1          # el cliente detecta el reinicio
            self.odom_pts = []
            self.wall_base += len(self.wall_pts) + 1
            self.wall_pts = []
            self.aeb_cuenta = 0
            self.dist = 0.0
            self.err_prev = {}

    # ================================================================ servidor web
    def handler(self):
        nodo = self

        class H(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def responder(self, codigo, cuerpo, tipo='application/json'):
                if isinstance(cuerpo, (dict, list)):
                    cuerpo = json.dumps(cuerpo, allow_nan=False).encode()
                self.send_response(codigo)
                self.send_header('Content-Type', tipo)
                self.send_header('Content-Length', str(len(cuerpo)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(cuerpo)

            def cuerpo(self):
                n = int(self.headers.get('Content-Length', 0))
                return json.loads(self.rfile.read(n) or b'{}')

            def do_GET(self):
                u = urlparse(self.path)
                q = parse_qs(u.query)
                try:
                    if u.path in ('/', '/index.html'):
                        self.responder(200, PAGINA.encode(), 'text/html; charset=utf-8')
                    elif u.path == '/datos':
                        self.responder(200, nodo.datos(int(q.get('odom', ['0'])[0]),
                                                       int(q.get('wall', ['0'])[0])))
                    elif u.path == '/video':
                        self.video()
                    elif u.path == '/wall':
                        self.responder(200, dict(ok=True, valores=nodo.wall_leer(),
                                                 grupos=[[g, [[n, s, a] for n, _, s, a in ps]]
                                                         for g, ps in WALL_PARAMS]))
                    else:
                        self.responder(404, b'', 'text/plain')
                except Exception as e:  # noqa: BLE001
                    self.responder(200, dict(ok=False, error=str(e)))

            def do_POST(self):
                try:
                    if self.path == '/cmd':
                        d = self.cuerpo()
                        with nodo.lock:
                            nodo.ultimo, nodo.t_ultimo = d, time.monotonic()
                        v, w = nodo.calcular(d)
                        self.responder(200, dict(v=round(v, 2), w=round(w, 2),
                                                 publicando=nodo.publicando, modo=nodo.modo))
                    elif self.path == '/wall':
                        nodo.wall_escribir(self.cuerpo())
                        self.responder(200, dict(ok=True))
                    elif self.path == '/modo':
                        nodo.cambiar_modo(self.cuerpo().get('modo'))
                        self.responder(200, dict(ok=True))
                    elif self.path == '/reiniciar':
                        nodo.reiniciar_graficas()
                        self.responder(200, dict(ok=True))
                    else:
                        self.responder(404, b'', 'text/plain')
                except Exception as e:  # noqa: BLE001
                    self.responder(200, dict(ok=False, error=str(e)))

            def video(self):
                """MJPEG: un JPEG por cuadro nuevo de la cámara, hasta que el celular cierre."""
                if cv2 is None and PILImage is None:
                    self.responder(503, b'sin OpenCV ni Pillow', 'text/plain')
                    return
                self.close_connection = True
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=cuadro')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Connection', 'close')
                self.end_headers()
                nodo.cam_clientes += 1
                visto, t_ult = -1, 0.0
                try:
                    while True:
                        with nodo.cam_cond:
                            nodo.cam_cond.wait_for(lambda: nodo.cam_n != visto, timeout=2.0)
                            m, visto = nodo.cam_msg, nodo.cam_n
                        if m is None:
                            continue
                        espera = 1.0 / nodo.cam_fps - (time.monotonic() - t_ult)
                        if espera > 0:
                            time.sleep(espera)
                        t_ult = time.monotonic()
                        j = nodo.jpeg(m)
                        self.wfile.write(b'--cuadro\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                         + str(len(j)).encode() + b'\r\n\r\n' + j + b'\r\n')
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, ssl.SSLError, OSError):
                    pass
                finally:
                    nodo.cam_clientes -= 1

            def log_message(self, *a):
                pass

        return H

    def certificado(self, carpeta):
        cert, key = os.path.join(carpeta, 'cert.pem'), os.path.join(carpeta, 'key.pem')
        if not (os.path.exists(cert) and os.path.exists(key)):
            os.makedirs(carpeta, exist_ok=True)
            self.get_logger().info(f'Generando certificado autofirmado en {carpeta}')
            subprocess.run(['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes',
                            '-keyout', key, '-out', cert, '-days', '365',
                            '-subj', '/CN=volante-robot'], check=True, capture_output=True)
        return cert, key

    def cerrar(self):
        self.servidor.shutdown()


def ip_local():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        return s.getsockname()[0]
    except OSError:
        return '127.0.0.1'
    finally:
        s.close()


PAGINA = r"""<!DOCTYPE html>
<html lang="es"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no, viewport-fit=cover">
<meta name="theme-color" content="#05081a">
<title>Consola MicroROS</title>
<style>
:root{
  --bg:#05081a; --panel:#0a1130; --panel2:#0d1640; --line:#1e2a5c; --txt:#e9edff; --mut:#8790b8;
  --cian:#3fdcec; --azul:#1f5af0; --azul2:#2f8ff0; --rojo:#e0465c; --verde:#3fd98b; --ambar:#f5b84a;
  --mono: ui-monospace, "SF Mono", "JetBrains Mono", Menlo, monospace;
}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;height:100%;background:var(--bg);color:var(--txt);
  font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;overflow:hidden}
button{font:inherit;color:inherit}
#app{display:flex;flex-direction:column;height:100%;padding:max(8px,env(safe-area-inset-top)) max(10px,env(safe-area-inset-right)) max(8px,env(safe-area-inset-bottom)) max(10px,env(safe-area-inset-left));gap:8px}

/* ---------- barra superior ---------- */
#top{display:flex;align-items:center;gap:12px;min-height:38px}
#estado{font:600 .78rem var(--mono);letter-spacing:.08em;display:flex;align-items:center;gap:7px;white-space:nowrap}
#estado i{width:8px;height:8px;border-radius:50%;background:var(--rojo);box-shadow:0 0 10px var(--rojo)}
#estado.ok i{background:var(--verde);box-shadow:0 0 10px var(--verde)}
#tabs{flex:1;display:flex;justify-content:center;gap:4px;background:var(--panel);border:1px solid var(--line);
  border-radius:12px;padding:3px;overflow-x:auto}
#tabs button{border:0;background:none;color:var(--mut);padding:6px 12px;border-radius:9px;font-size:.82rem;
  font-weight:600;white-space:nowrap}
#tabs button.on{background:linear-gradient(135deg,var(--azul),var(--azul2));color:#fff;box-shadow:0 2px 12px #1f5af055}
#tabs button b{font:600 .7rem var(--mono);opacity:.6;margin-right:5px}
#vw{font:.8rem var(--mono);color:var(--cian);white-space:nowrap;text-align:right}
#modo{font:700 .66rem var(--mono);letter-spacing:.1em;padding:3px 7px;border-radius:6px;border:1px solid var(--line);color:var(--mut)}
#modo.auto{color:var(--ambar);border-color:#f5b84a66}#modo.teleop{color:var(--cian);border-color:#3fdcec55}
#aebchip{font:700 .66rem var(--mono);letter-spacing:.1em;padding:3px 7px;border-radius:6px;background:var(--rojo);color:#fff;display:none}

.vista{flex:1;min-height:0;display:none}
.vista.on{display:grid}
.card{background:var(--panel);border:1px solid var(--line);border-radius:16px}
.lbl{font:600 .66rem var(--mono);letter-spacing:.14em;color:var(--mut);text-transform:uppercase}
.btn{background:var(--panel2);border:1px solid var(--line);border-radius:10px;padding:9px 14px;font-weight:700;font-size:.85rem}
.btn:active{filter:brightness(1.4)}
.btn.pri{background:linear-gradient(135deg,var(--azul),var(--azul2));border-color:transparent}
.btn.ok{border-color:#3fd98b88;color:var(--verde)}.btn.mal{border-color:#e0465c88;color:var(--rojo)}
.aviso{font-size:.8rem;color:var(--ambar);background:#f5b84a12;border:1px solid #f5b84a44;border-radius:10px;padding:7px 10px}
input[type=range]{-webkit-appearance:none;appearance:none;width:100%;height:6px;border-radius:3px;
  background:linear-gradient(90deg,var(--cian) var(--p,50%),#2a3158 var(--p,50%));outline:none;touch-action:auto}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:22px;height:22px;border-radius:50%;background:var(--cian);
  box-shadow:0 0 0 4px #3fdcec33}
input[type=range]::-moz-range-thumb{width:22px;height:22px;border-radius:50%;background:var(--cian);border:0}
.slider{display:flex;flex-direction:column;gap:6px}
.slider .fila{display:flex;justify-content:space-between}
.slider .val{font:.75rem var(--mono);color:var(--cian)}

/* ---------- volante (SVG) ---------- */
.volante{width:100%;height:100%;max-height:100%;aspect-ratio:1;display:block;margin:auto;transition:transform 40ms linear;pointer-events:none}
.g{text-transform:none}

/* ---------- 1 Calibración ---------- */
#v-cal{grid-template-columns:1.15fr 1fr;gap:12px}
#v-cal .izq{display:flex;align-items:center;justify-content:center;min-height:0;padding:6px;position:relative;overflow:hidden}
#v-cal .der{display:flex;flex-direction:column;gap:14px;padding:16px;overflow:auto}
#barra{height:12px;border-radius:6px;background:#1a2350;position:relative;overflow:hidden}
#barra i{position:absolute;top:0;bottom:0;left:50%;background:var(--cian);border-radius:6px}
#barra:after{content:'';position:absolute;left:50%;top:-2px;bottom:-2px;width:2px;background:#fff6}
.big{font:700 1.9rem var(--mono)}
.fila2{display:flex;gap:10px}
.nota{font-size:.78rem;color:var(--mut)}

/* ---------- Operación: cámara de fondo + controles translúcidos ---------- */
#v-op{position:relative;background:#000;border-radius:16px;overflow:hidden;touch-action:none;
  -webkit-user-select:none;user-select:none}
#cam-img{position:absolute;inset:0;width:100%;height:100%;object-fit:contain;pointer-events:none}
#v-op.llenar #cam-img{object-fit:cover}
#op-hud{cursor:pointer}
#cam-nada{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;color:var(--mut);
  font:.85rem var(--mono);pointer-events:none;text-align:center;padding:0 30%}
.pedal{position:absolute;top:10px;bottom:10px;width:24%;border-radius:20px;display:flex;flex-direction:column;
  align-items:center;justify-content:center;gap:8px;-webkit-user-select:none;user-select:none;
  -webkit-touch-callout:none;touch-action:none;transition:background .08s,border-color .08s;
  background:rgba(255,255,255,.035);border:1px solid rgba(255,255,255,.10);color:rgba(255,255,255,.38)}
#rev{left:10px}#fwd{right:10px}
.pedal .flecha{font-size:2rem;line-height:1}
.pedal .t{font-size:clamp(1.1rem,3.2vw,1.8rem);font-weight:800}
.pedal .s{font:600 .6rem var(--mono);letter-spacing:.22em;opacity:.7}
#rev.on{background:rgba(224,70,92,.22);border-color:rgba(224,70,92,.7);color:#fff}
#fwd.on{background:rgba(47,143,240,.22);border-color:rgba(47,143,240,.7);color:#fff}
.vidrio{background:rgba(5,8,26,.45);border:1px solid rgba(255,255,255,.12);border-radius:12px;
  backdrop-filter:blur(6px);-webkit-backdrop-filter:blur(6px)}
#op-hud{position:absolute;top:10px;left:50%;transform:translateX(-50%);display:flex;gap:12px;align-items:center;
  padding:6px 12px;font:600 .75rem var(--mono);white-space:nowrap}
#op-giro{position:absolute;bottom:58px;left:50%;transform:translateX(-50%);width:38%;height:8px;border-radius:4px;
  background:rgba(255,255,255,.12);overflow:hidden}
#op-giro i{position:absolute;top:0;bottom:0;left:50%;background:var(--cian);opacity:.8}
#op-giro:after{content:'';position:absolute;left:50%;top:0;bottom:0;width:2px;background:#fff8}
#op-abajo{position:absolute;bottom:10px;left:50%;transform:translateX(-50%);width:44%;display:flex;gap:10px;
  align-items:center;padding:6px 10px}
#op-abajo .slider{flex:1}
#op-abajo .btn{background:transparent;padding:6px 10px;font-size:.78rem}
#op-aviso{position:absolute;top:52px;left:50%;transform:translateX(-50%);display:none;align-items:center;gap:10px;
  white-space:nowrap;background:rgba(40,28,6,.8)}

/* ---------- Inicio ---------- */
#v-ini{grid-template-columns:minmax(0,1fr) minmax(0,1.15fr);gap:10px;overflow:auto;align-content:start}
#ini-modo{padding:14px;display:flex;flex-direction:column;gap:12px}
#ini-estado{font:800 1.5rem var(--mono)}
.modos{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.modo-b{border-radius:14px;padding:12px;text-align:left;border:1px solid var(--line);background:var(--panel2);display:flex;flex-direction:column;gap:4px}
.modo-b b{font-size:1.05rem}.modo-b span{font-size:.72rem;color:var(--mut)}
.modo-b.sel{border-color:var(--cian);box-shadow:0 0 0 1px var(--cian) inset}
.modo-b.auto b{color:var(--verde)}.modo-b.stop b{color:var(--rojo)}.modo-b.man b{color:var(--cian)}
.estados{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.estados div{background:var(--panel2);border:1px solid var(--line);border-radius:10px;padding:7px 9px;display:flex;flex-direction:column;gap:2px}
.estados b{font:700 .9rem var(--mono)}
#ini-guia{padding:14px;display:flex;flex-direction:column;gap:9px;font-size:.82rem;line-height:1.35}
#ini-guia h3{margin:0;font-size:.95rem}
.paso{display:grid;grid-template-columns:26px 1fr;gap:8px;align-items:start}
.paso i{font:700 .75rem var(--mono);font-style:normal;width:22px;height:22px;border-radius:7px;background:var(--panel2);
  border:1px solid var(--line);display:flex;align-items:center;justify-content:center;color:var(--cian)}
.paso p{margin:0;color:#c6cbe6}.paso b{color:var(--txt)}
.seg{border-top:1px solid var(--line);padding-top:8px;color:var(--mut);font-size:.76rem}

/* ---------- 3 Odometría y 4 Wall ---------- */
#v-odo,#v-wall{overflow:auto;gap:10px;align-content:start}
#v-odo{grid-template-columns:repeat(6,minmax(0,1fr))}
.kpi{padding:10px 12px;display:flex;flex-direction:column;gap:3px}
.kpi .n{font:700 1.15rem var(--mono);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.kpi .u{font:.7rem var(--mono);color:var(--mut)}
.graf{padding:10px;display:flex;flex-direction:column;gap:6px}
.graf canvas{width:100%;height:210px;display:block}
.leyenda{display:flex;gap:14px;flex-wrap:wrap;font:.7rem var(--mono);color:var(--mut)}
.leyenda span:before{content:'';display:inline-block;width:14px;height:3px;border-radius:2px;margin-right:5px;vertical-align:middle;background:var(--c)}
.span2{grid-column:span 2}.span3{grid-column:span 3}.span4{grid-column:span 4}.span5{grid-column:1/-1}

#v-wall{grid-template-columns:minmax(0,1.25fr) minmax(0,1fr)}
#params{padding:12px;display:flex;flex-direction:column;gap:12px}
.grupo{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:8px}
.prm{background:var(--panel2);border:1px solid var(--line);border-radius:12px;padding:8px}
.prm.cambio{border-color:var(--ambar)}
.prm .nom{font:700 .8rem var(--mono)}
.prm .ay{font-size:.66rem;color:var(--mut);min-height:2.1em;line-height:1.05em;margin:2px 0 6px}
.prm .ctl{display:flex;gap:4px}
.prm input{flex:1;min-width:0;background:#060a22;border:1px solid var(--line);border-radius:8px;color:var(--txt);
  font:700 .95rem var(--mono);text-align:center;padding:6px 2px}
.prm button{width:34px;border-radius:8px;border:1px solid var(--line);background:#121b4a;font-weight:800}
#wall-der{display:flex;flex-direction:column;gap:10px}
.botones{display:flex;gap:8px;flex-wrap:wrap}
#vueltas{font:.8rem var(--mono);display:flex;flex-direction:column;gap:3px;max-height:110px;overflow:auto}
#vueltas div{display:flex;justify-content:space-between}
#msg{font:.75rem var(--mono);min-height:1.2em}

#modo-estado{font:700 .8rem var(--mono);letter-spacing:.06em}
.btn.grande{font-size:1rem;padding:12px 18px}

/* ---------- inicio ---------- */
#inicio{position:fixed;inset:0;background:radial-gradient(circle at 50% 40%,#0d1a55,#05081a 70%);display:flex;
  flex-direction:column;align-items:center;justify-content:center;gap:18px;text-align:center;padding:24px;z-index:9}
#inicio h1{margin:0;font-size:1.6rem}
#inicio p{margin:0;color:var(--mut);max-width:520px}
@media (max-height:420px){.graf canvas{height:170px}#top{min-height:32px}}
</style></head>
<body>
<div id="app">
  <div id="top">
    <div id="estado"><i></i><span>SIN CONEXIÓN</span></div>
    <div id="tabs">
      <button data-v="ini" class="on"><b>1</b>Inicio</button>
      <button data-v="cal"><b>2</b>Calibración</button>
      <button data-v="op"><b>3</b>Operación</button>
      <button data-v="odo"><b>4</b>Odometría</button>
      <button data-v="wall"><b>5</b>Wall following</button>
    </div>
    <span id="aebchip">AEB</span>
    <span id="modo">—</span>
    <div id="vw">v 0.00 · ω 0.00</div>
  </div>

  <!-- 1 INICIO -->
  <section class="vista on" id="v-ini">
    <div class="card" id="ini-modo">
      <div class="lbl">Modo del robot</div>
      <div id="ini-estado">—</div>
      <div class="modos">
        <button class="modo-b man" data-modo="teleop"><b>🎮 Manual</b><span>Teclado o el celular (pestaña Operación). El wall_follow queda detenido.</span></button>
        <button class="modo-b auto" data-modo="auto"><b>▶ Autónomo</b><span>El wall_follow maneja solo. Pide confirmación: el robot arranca.</span></button>
        <button class="modo-b stop" id="ini-stop"><b>■ Detener autónomo</b><span>Para el wall_follow sin salir de modo autónomo.</span></button>
        <button class="modo-b" id="ini-reset"><b>↺ Reiniciar contadores</b><span>Borra gráficas, frenos del AEB y tiempos.</span></button>
      </div>
      <div class="estados">
        <div><span class="lbl">Conexión</span><b id="e-con">—</b></div>
        <div><span class="lbl">AEB</span><b id="e-aeb">—</b></div>
        <div><span class="lbl">v real</span><b id="e-v">—</b></div>
        <div><span class="lbl">wall_follow</span><b id="e-wall">—</b></div>
        <div><span class="lbl">Vueltas</span><b id="e-vue">—</b></div>
        <div><span class="lbl">Frenos AEB</span><b id="e-fre">—</b></div>
      </div>
      <div id="msg-ini" style="font:.75rem var(--mono);min-height:1.2em"></div>
    </div>
    <div class="card" id="ini-guia">
      <h3>Cómo funciona</h3>
      <div class="paso"><i>1</i><p><b>Inicio</b> (aquí): eliges el modo. Todo lo que mueve el robot pasa primero por el <b>AEB</b>, que frena si va a chocar, en cualquier modo.</p></div>
      <div class="paso"><i>2</i><p><b>Calibración</b>: sostén el celular horizontal como un volante y toca <b>Calibrar recto</b>. Ajusta la <b>sensibilidad</b> (cuánto giras para el giro máximo). Aquí el robot <b>no</b> se mueve.</p></div>
      <div class="paso"><i>3</i><p><b>Operación</b> (modo Manual): ves la <b>cámara del robot</b>. Mantén <b>Adelante</b> o <b>Reversa</b> e inclina el celular para girar. Al soltar, el robot frena.</p></div>
      <div class="paso"><i>4</i><p><b>Odometría</b>: error en vivo de la odometría propia (ω_I, ω_D) y del DiffDrive frente a la pose real de Gazebo.</p></div>
      <div class="paso"><i>5</i><p><b>Wall following</b>: cambia las ganancias y velocidades del controlador mientras corre, y mira e(t), los tiempos por vuelta y los frenos del AEB.</p></div>
      <div class="seg">Seguridad: el robot solo obedece al celular en la pestaña Operación. Si cambias de pestaña, bloqueas la pantalla o se cae el WiFi, se detiene. Al cambiar de modo se descarta el último comando.</div>
    </div>
  </section>

  <!-- 2 CALIBRACIÓN -->
  <section class="vista" id="v-cal">
    <div class="card izq"><svg class="volante" data-vol viewBox="0 0 200 200"></svg></div>
    <div class="card der">
      <div><div class="lbl">Ángulo del celular</div><div class="big" id="cal-ang">+0°</div></div>
      <div><div class="lbl" style="margin-bottom:6px"><span class="g">ω</span> que se mandaría</div>
        <div id="barra"><i></i></div>
        <div class="slider"><div class="fila"><span class="nota">izquierda</span><span class="val" id="cal-w">0.00 rad/s</span><span class="nota">derecha</span></div></div></div>
      <div class="slider"><div class="fila"><span class="lbl">Sensibilidad (giro para <span class="g">ω</span> máx)</span><span class="val" id="sens-v"></span></div>
        <input type="range" id="sens" min="15" max="90" step="1" value="45"></div>
      <div class="slider"><div class="fila"><span class="lbl">Zona muerta</span><span class="val" id="zm-v"></span></div>
        <input type="range" id="zm" min="0" max="15" step="0.5" value="4"></div>
      <div class="fila2"><button class="btn pri" id="cal">Calibrar recto</button><button class="btn" id="inv">Invertir sentido</button></div>
      <div class="nota">En esta pestaña el robot <b>no se mueve</b>: solo ves cómo reacciona el celular. Los ajustes se guardan en el celular.</div>
    </div>
  </section>

  <!-- 2 OPERACIÓN -->
  <section class="vista" id="v-op">
    <img id="cam-img" alt="">
    <div id="cam-nada">Esperando la cámara (/image_raw)…</div>
    <div class="pedal" id="rev"><div class="flecha">‹</div><div class="t">Reversa</div><div class="s">MANTENER</div></div>
    <div class="pedal" id="fwd"><div class="flecha">›</div><div class="t">Adelante</div><div class="s">MANTENER</div></div>
    <div id="op-hud" class="vidrio"><span id="op-vw">v 0.00 · ω 0.00</span><span id="op-ang">+0°</span>
      <span id="cam-aeb" style="color:var(--rojo);display:none">● AEB</span><span style="opacity:.6">⤢</span></div>
    <div id="op-aviso" class="aviso"><span>Modo AUTÓNOMO: el celular no mueve el robot.</span>
      <button class="btn" data-modo="teleop">Volver a manual</button></div>
    <div id="op-giro"><i></i></div>
    <div id="op-abajo" class="vidrio">
      <button class="btn" id="cal2">Calibrar</button>
      <div class="slider"><div class="fila"><span class="lbl">Acel</span><span class="val" id="acel-v"></span></div>
        <input type="range" id="acel" min="0.1" max="1" step="0.05" value="0.6"></div>
    </div>
  </section>

  <!-- 4 ODOMETRÍA -->
  <section class="vista" id="v-odo">
    <div class="card kpi"><span class="lbl">Distancia real</span><span class="n" id="k-dist">0.00</span><span class="u">m</span></div>
    <div class="card kpi"><span class="lbl">Error propia</span><span class="n" id="k-el" style="color:var(--rojo)">—</span><span class="u" id="k-el-u">= DiffDrive</span></div>
    <div class="card kpi"><span class="lbl">Error corregida</span><span class="n" id="k-ek" style="color:var(--verde)">—</span><span class="u" id="k-ek-u">skid-steer</span></div>
    <div class="card kpi"><span class="lbl"><span class="g">ψ</span> propia</span><span class="n" id="k-sl">—</span><span class="u">grados</span></div>
    <div class="card kpi"><span class="lbl"><span class="g">ψ</span> corregida</span><span class="n" id="k-sk">—</span><span class="u">grados</span></div>
    <div class="card kpi"><span class="lbl">k en vivo</span><span class="n" id="k-k" style="color:var(--ambar)">—</span><span class="u" id="k-bef">b_ef —</span></div>
    <div class="card graf span4"><div class="lbl" id="t-epos">‖error de posición‖ vs distancia recorrida</div><canvas id="g-epos"></canvas>
      <div class="leyenda"><span style="--c:var(--rojo)">propia (ω_I, ω_D)</span><span style="--c:var(--azul2)">DiffDrive /odom (punteada)</span><span style="--c:var(--verde)">corregida skid-steer</span></div></div>
    <div class="card graf span2"><div class="lbl">Trayectoria</div><canvas id="g-xy"></canvas>
      <div class="leyenda"><span style="--c:#ffffff">real</span><span style="--c:var(--rojo)">propia</span><span style="--c:var(--verde)">corregida</span></div></div>
    <div class="card graf span4"><div class="lbl">error de orientación vs distancia [°] (sin saltos en ±180°)</div><canvas id="g-epsi"></canvas>
      <div class="leyenda"><span style="--c:var(--rojo)">propia</span><span style="--c:var(--azul2)">DiffDrive (punteada)</span><span style="--c:var(--verde)">corregida</span></div></div>
    <div class="card kpi span2" style="gap:8px">
      <div class="lbl">Cómo leerlas</div>
      <div class="nota"><b>Propia</b> y <b>DiffDrive</b> usan la misma cinemática sin deslizamiento: coinciden (la roja queda debajo de la azul).
        Al girar, el skid-steer <b>desliza</b>: el robot gira solo <b>k</b> veces lo que dicen las ruedas, la orientación estimada se desvía y la posición "se va".
        La <b>corregida</b> usa la vía efectiva b_ef = b/k (/odom_skid). <b>k en vivo</b> se calcula con /ground_truth mientras manejas.</div>
      <button class="btn" id="reset-odo">Reiniciar gráficas</button></div>
  </section>

  <!-- 5 WALL FOLLOWING -->
  <section class="vista" id="v-wall">
    <div class="card" id="params"><div class="nota" id="params-nota">Cargando parámetros de /wall_follow…</div></div>
    <div id="wall-der">
      <div class="aviso" id="wall-aviso" style="display:none"></div>
      <div class="card" style="padding:12px;display:flex;flex-direction:column;gap:10px">
        <div><span class="lbl">Estado </span><span id="modo-estado">—</span>
          <span class="nota"> · el modo se elige en Inicio</span></div>
        <div class="botones"><button class="btn pri" id="w-apl">Aplicar cambios</button></div>
        <div class="botones">
          <span class="lbl" style="align-self:center">Presets</span>
          <button class="btn" data-pre="seguro">Seguro</button><button class="btn" data-pre="rapido">Rápido</button>
          <button class="btn" data-pre="inicial">Valores al abrir</button>
        </div>
        <div id="msg"></div>
      </div>
      <div style="display:grid;grid-template-columns:repeat(3,1fr);gap:10px">
        <div class="card kpi"><span class="lbl">Vuelta</span><span class="n" id="w-vuelta">—</span><span class="u" id="w-ult">última —</span></div>
        <div class="card kpi"><span class="lbl">Frenos AEB</span><span class="n" id="w-aeb">0</span><span class="u">desde el reinicio</span></div>
        <div class="card kpi"><span class="lbl">v real</span><span class="n" id="w-v">0.00</span><span class="u">m/s</span></div>
      </div>
      <div class="card" style="padding:10px"><div class="lbl">Tiempos por vuelta (aprox.)</div><div id="vueltas"></div></div>
      <div class="card graf"><div class="lbl g" style="letter-spacing:.06em">e(t), D_t y d_obj [cm] — últimos 40 s</div><canvas id="g-wall"></canvas>
        <div class="leyenda"><span style="--c:var(--cian)">e</span><span style="--c:#ffffff">D_t</span><span style="--c:var(--ambar)">d_obj</span></div></div>
      <button class="btn" id="reset-w">Reiniciar contadores y gráficas</button>
    </div>
  </section>
</div>

<div id="inicio">
  <h1>Consola MicroROS</h1>
  <p>Gira el celular horizontal. Para manejar, sostenlo como un volante. Toca para empezar (el navegador pedirá permiso para los sensores).</p>
  <button class="btn pri" id="go" style="font-size:1.15rem;padding:14px 30px">Empezar</button>
  <div id="aviso-ini" style="color:#f99"></div>
</div>

<script>
const $ = id => document.getElementById(id);
const store = { get(k, d){ try { const v = localStorage.getItem('vol_'+k); return v===null ? d : JSON.parse(v); } catch(_) { return d; } },
                set(k, v){ try { localStorage.setItem('vol_'+k, JSON.stringify(v)); } catch(_){} } };
let theta = null, theta0 = store.get('theta0', 0), signo = store.get('signo', 1), giro = 0;
let adelante = false, reversa = false, vista = 'ini';
let sens = store.get('sens', 45), zm = store.get('zm', 4);
const W_MAX = 4.0;

// ---------------- volante SVG ----------------
const VOL = `
 <defs><linearGradient id="gr" x1="0" y1="0" x2="1" y2="1">
   <stop offset="0" stop-color="#14309a"/><stop offset=".55" stop-color="#1f5af0"/><stop offset="1" stop-color="#3fc6ec"/></linearGradient></defs>
 <circle cx="100" cy="100" r="88" fill="none" stroke="#1a2556" stroke-width="22"/>
 <circle cx="100" cy="100" r="88" fill="none" stroke="url(#gr)" stroke-width="16"/>
 <circle cx="100" cy="100" r="78" fill="none" stroke="#0a1130" stroke-width="3"/>
 <g stroke="#2b3a7c" stroke-width="12" stroke-linecap="round">
   <line x1="20" y1="106" x2="72" y2="99"/><line x1="180" y1="106" x2="128" y2="99"/><line x1="100" y1="128" x2="100" y2="172"/></g>
 <circle cx="100" cy="100" r="30" fill="#0b1438" stroke="#2b3a7c" stroke-width="3"/>
 <rect x="95" y="4" width="10" height="22" rx="4" fill="#3fdcec"/>
 <text class="ang" x="100" y="104" text-anchor="middle" fill="#e9edff" font-family="ui-monospace,monospace" font-size="10">+0°</text>`;
document.querySelectorAll('[data-vol]').forEach((s, i) => s.innerHTML = VOL.replaceAll('id="gr"', `id="gr${i}"`).replaceAll('url(#gr)', `url(#gr${i})`));

function wDe(g){ // misma fórmula que el servidor (solo para mostrar)
  let x = Math.abs(g) < zm ? 0 : Math.sign(g)*(Math.abs(g)-zm);
  x = Math.max(-1, Math.min(1, x/(sens-zm)));
  return -W_MAX*x;
}
function pintarVolante(){
  document.querySelectorAll('[data-vol]').forEach(s => {
    s.style.transform = `rotate(${giro}deg)`;
    s.querySelector('.ang').textContent = (giro>=0?'+':'') + giro.toFixed(0) + '°';
  });
  $('cal-ang').textContent = (giro>=0?'+':'') + giro.toFixed(1) + '°';
  const w = wDe(giro), f = Math.min(1, Math.abs(w)/W_MAX)*50;
  const b = $('barra').firstElementChild;
  b.style.width = f + '%'; b.style.left = (w > 0 ? 50 - f : 50) + '%';
  $('cal-w').textContent = w.toFixed(2) + ' rad/s';
  const og = $('op-giro').firstElementChild;                  // barra de giro sobre la cámara
  og.style.width = f + '%'; og.style.left = (w > 0 ? 50 - f : 50) + '%';
  $('op-ang').textContent = (giro>=0?'+':'') + giro.toFixed(0) + '°';
}
function onMotion(e){
  const g = e.accelerationIncludingGravity;
  if (!g || g.x === null) return;
  theta = Math.atan2(g.x, g.y) * 180 / Math.PI;
  let d = ((theta - theta0 + 540) % 360) - 180;
  giro = -d * signo;
  requestAnimationFrame(pintarVolante);
}
function calibrar(){ if (theta !== null){ theta0 = theta; store.set('theta0', theta0); } }

// ---------------- sliders ----------------
function slider(id, val, fmt, cb){
  const el = $(id), out = $(id + '-v');
  const pintar = () => { const p = (el.value-el.min)/(el.max-el.min)*100; el.style.setProperty('--p', p+'%');
                         if (out) out.textContent = fmt(+el.value); };
  el.value = val; pintar();
  el.addEventListener('input', () => { pintar(); cb && cb(+el.value); });
}
slider('sens', sens, v => v.toFixed(0)+'°', v => { sens = v; store.set('sens', v); pintarVolante(); });
slider('zm', zm, v => v.toFixed(1)+'°', v => { zm = v; store.set('zm', v); pintarVolante(); });
slider('acel', store.get('acel', 0.6), v => Math.round(v*100)+'%', v => store.set('acel', v));
$('cal').onclick = $('cal2').onclick = calibrar;
$('inv').onclick = () => { signo = -signo; store.set('signo', signo); };

// ---------------- pestañas ----------------
function irA(v){
  vista = v; adelante = reversa = false;
  document.querySelectorAll('#tabs button').forEach(b => b.classList.toggle('on', b.dataset.v === v));
  document.querySelectorAll('.vista').forEach(s => s.classList.toggle('on', s.id === 'v-' + v));
  if (v === 'wall') cargarParams();
  const img = $('cam-img');
  if (v === 'op'){ img.src = '/video?' + Date.now(); }         // la cámara solo se transmite en Operación
  else if (img.getAttribute('src')){ img.removeAttribute('src'); $('cam-nada').style.display = 'flex'; }
  if (v === 'odo' || v === 'wall') requestAnimationFrame(dibujarTodo);
}
document.querySelectorAll('#tabs button').forEach(b => b.onclick = () => irA(b.dataset.v));

// ---------------- pedales ----------------
function pedal(id, set){
  const b = $(id);
  b.addEventListener('pointerdown', e => { e.preventDefault(); b.setPointerCapture(e.pointerId); set(true); b.classList.add('on');
                                           if (navigator.vibrate) navigator.vibrate(8); });
  ['pointerup','pointercancel','lostpointercapture'].forEach(t => b.addEventListener(t, () => { set(false); b.classList.remove('on'); }));
  b.addEventListener('contextmenu', e => e.preventDefault());
}
pedal('fwd', x => adelante = x);
pedal('rev', x => reversa = x);
document.addEventListener('visibilitychange', () => { adelante = reversa = false; });

// ---------------- envío del volante (20 Hz) ----------------
let enVuelo = false, conectado = false;
function estado(ok){
  conectado = ok; $('estado').classList.toggle('ok', ok);
  $('estado').lastElementChild.textContent = ok ? 'CONECTADO' : 'SIN CONEXIÓN';
  if (!ok){ $('e-con').textContent = 'SIN CONEXIÓN'; $('e-con').style.color = css('--rojo'); }
}
async function enviar(){
  if (enVuelo) return; enVuelo = true;
  try {
    const r = await fetch('/cmd', {method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({giro, adelante, reversa, acelerador:+$('acel').value, sensibilidad:sens, zona_muerta:zm,
                            operar: vista === 'op' && !document.hidden})});
    const c = await r.json(); estado(true);
    if (vista === 'op'){ $('vw').textContent = `v ${c.v.toFixed(2)} · ω ${c.w.toFixed(2)}`;
                         $('op-vw').textContent = `v ${c.v.toFixed(2)} · ω ${c.w.toFixed(2)}`; }
  } catch(_) { estado(false); }
  enVuelo = false;
}

// ---------------- gráficas (canvas) ----------------
function chart(cv, series, o = {}){
  const dpr = window.devicePixelRatio || 1, W = cv.clientWidth, H = cv.clientHeight;
  if (!W || !H) return;
  if (cv.width !== W*dpr || cv.height !== H*dpr){ cv.width = W*dpr; cv.height = H*dpr; }
  const c = cv.getContext('2d'); c.setTransform(dpr,0,0,dpr,0,0); c.clearRect(0,0,W,H);
  const L = 44, R = 8, T = 8, B = 22;
  let xs = [], ys = [];
  series.forEach(s => s.p.forEach(([x,y]) => { if (y !== null && isFinite(y)) { xs.push(x); ys.push(y); } }));
  if (!xs.length){ c.fillStyle='#8790b8'; c.font='12px ui-monospace,monospace'; c.fillText('esperando datos…', L+8, H/2); return; }
  let x0 = o.x0 ?? Math.min(...xs), x1 = o.x1 ?? Math.max(...xs), y0 = o.y0 ?? Math.min(...ys), y1 = o.y1 ?? Math.max(...ys);
  if (o.cero){ y0 = Math.min(y0, 0); y1 = Math.max(y1, 0); }
  if (x1 - x0 < 1e-9) x1 = x0 + 1; if (y1 - y0 < 1e-9){ y1 += 1; y0 -= 1; }
  const pad = (y1-y0)*0.08; y0 -= pad; y1 += pad;
  if (o.igual){ // misma escala en x e y (trayectoria)
    const sx = (W-L-R)/(x1-x0), sy = (H-T-B)/(y1-y0), s = Math.min(sx, sy);
    const cx = (x0+x1)/2, cy = (y0+y1)/2; x0 = cx-(W-L-R)/2/s; x1 = cx+(W-L-R)/2/s; y0 = cy-(H-T-B)/2/s; y1 = cy+(H-T-B)/2/s;
  }
  const X = x => L + (x-x0)/(x1-x0)*(W-L-R), Y = y => H-B - (y-y0)/(y1-y0)*(H-T-B);
  c.strokeStyle='#1e2a5c'; c.lineWidth=1; c.fillStyle='#8790b8'; c.font='10px ui-monospace,monospace';
  for (let i=0;i<=4;i++){ const yv = y0+(y1-y0)*i/4, yy = Y(yv); c.beginPath(); c.moveTo(L,yy); c.lineTo(W-R,yy); c.stroke();
    c.fillText(Math.abs(yv)>=100?yv.toFixed(0):yv.toFixed(1), 2, yy+3); }
  for (let i=0;i<=4;i++){ const xv = x0+(x1-x0)*i/4; c.fillText(Math.abs(xv)>=100?xv.toFixed(0):xv.toFixed(1), X(xv)-8, H-6); }
  if (o.xl){ c.fillText(o.xl, W-R-c.measureText(o.xl).width, H-B-4); }
  series.forEach(s => { c.strokeStyle = s.c; c.lineWidth = s.w || 1.8; c.globalAlpha = s.a || 1; c.setLineDash(s.dash || []); c.beginPath(); let m = false;
    s.p.forEach(([x,y]) => { if (y===null || !isFinite(y)) { m = false; return; } m ? c.lineTo(X(x),Y(y)) : c.moveTo(X(x),Y(y)); m = true; });
    c.stroke(); c.globalAlpha = 1; c.setLineDash([]); });
}
const css = v => getComputedStyle(document.documentElement).getPropertyValue(v).trim();

// ---------------- datos del servidor ----------------
let nOdom = 0, nWall = 0, odom = [], wall = [], ultimo = {};
async function pedirDatos(){
  try {
    const d = await (await fetch(`/datos?odom=${nOdom}&wall=${nWall}`)).json();
    if (d.reinicio_odom){ odom = []; nOdom = 0; } if (d.reinicio_wall){ wall = []; nWall = 0; }
    odom.push(...d.odom); nOdom = d.n_odom; if (odom.length > 6000) odom = odom.slice(-6000);
    wall.push(...d.wall); nWall = d.n_wall; if (wall.length > 2000) wall = wall.slice(-2000);
    ultimo = d; actualizar(d);
  } catch(_) {}
}
function actualizar(d){
  const m = $('modo'); m.textContent = d.modo ? d.modo.toUpperCase() : '—'; m.className = d.modo || '';
  $('aebchip').style.display = d.aeb ? 'inline-block' : 'none';
  if (vista !== 'op') $('vw').textContent = `v ${d.v_odom.toFixed(2)} m/s`;
  $('op-aviso').style.display = d.modo === 'auto' ? 'flex' : 'none';
  const aw = $('wall-aviso');
  aw.style.display = d.wall_ok ? 'none' : 'block';
  aw.textContent = '/wall_follow no responde (¿se lanzó con algoritmo:=ninguno?).';
  const me = $('modo-estado');
  if (d.modo === 'auto'){ me.textContent = d.wall_activo ? 'AUTÓNOMO · wall_follow corriendo' : 'AUTÓNOMO · detenido';
                          me.style.color = d.wall_activo ? css('--verde') : css('--ambar'); }
  else { me.textContent = d.modo === 'teleop' ? 'MANUAL (teclado / volante)' : '—'; me.style.color = css('--cian'); }
  $('cam-aeb').style.display = d.aeb ? 'inline' : 'none';
  // Inicio
  const ie = $('ini-estado');
  ie.textContent = d.modo === 'auto' ? (d.wall_activo ? 'AUTÓNOMO ▶' : 'AUTÓNOMO ■ detenido') : (d.modo === 'teleop' ? 'MANUAL 🎮' : 'esperando /aeb_mux…');
  ie.style.color = d.modo === 'auto' ? (d.wall_activo ? css('--verde') : css('--ambar')) : css('--cian');
  document.querySelectorAll('.modo-b[data-modo]').forEach(b => b.classList.toggle('sel', b.dataset.modo === d.modo));
  $('e-con').textContent = 'OK'; $('e-con').style.color = css('--verde');
  $('e-aeb').textContent = d.aeb ? 'FRENANDO' : 'libre'; $('e-aeb').style.color = d.aeb ? css('--rojo') : css('--txt');
  $('e-v').textContent = d.v_odom.toFixed(2) + ' m/s';
  $('e-wall').textContent = d.wall_activo === null ? 'no responde' : (d.wall_activo ? 'corriendo' : 'detenido');
  $('e-vue').textContent = Math.max(0, (d.vueltas || []).length - 1);
  $('e-fre').textContent = d.aeb_cuenta;
  // odometría
  const u = odom[odom.length-1];
  $('k-dist').textContent = d.dist.toFixed(2);
  const f = (x, n) => x === null || x === undefined ? '—' : (+x).toFixed(n);
  const mm = (x, id) => { if (x === null || x === undefined){ $(id).textContent = '—'; return; }
    $(id).textContent = x >= 1000 ? (x/1000).toFixed(2) + ' m' : x.toFixed(0) + ' mm'; };
  if (u){ mm(u.el, 'k-el'); mm(u.ek, 'k-ek');
          $('k-sl').textContent = f(u.sl, 1) + '°'; $('k-sk').textContent = f(u.sk, 1) + '°';
          $('k-el-u').textContent = u.ed !== null && u.el !== null ? `DiffDrive: ${u.ed >= 1000 ? (u.ed/1000).toFixed(2)+' m' : u.ed.toFixed(0)+' mm'}` : '= DiffDrive';
          $('k-ek-u').textContent = u.ek === null ? 'sin /odom_skid' : 'skid-steer (b_ef)'; }
  $('k-k').textContent = d.k === null ? '—' : d.k.toFixed(3);
  $('k-bef').textContent = d.b_ef === null ? `girando… (${d.n_k})` : `b_ef ≈ ${d.b_ef.toFixed(3)} m`;
  // wall
  $('w-aeb').textContent = d.aeb_cuenta; $('w-v').textContent = d.v_odom.toFixed(2);
  const V = d.vueltas || [];
  $('w-vuelta').textContent = V.length ? (V[V.length-1][0] === 0 ? 'salida' : 'V' + V[V.length-1][0]) : '—';
  const tiempos = []; for (let i=1;i<V.length;i++) tiempos.push([V[i][0], V[i][1]-V[i-1][1]]);
  $('w-ult').textContent = tiempos.length ? `última ${tiempos[tiempos.length-1][1].toFixed(1)} s` : 'última —';
  $('vueltas').innerHTML = tiempos.map(([k,t]) => `<div><span>Vuelta ${k}</span><span>${t.toFixed(2)} s</span></div>`).join('')
    || '<span class="nota">Cruza la línea de salida para empezar a medir.</span>';
  dibujarTodo();
}
function dibujarTodo(){
  if (vista === 'odo'){
    // error de posición: en mm, o en m si pasa de 2 m
    const mx = Math.max(0, ...odom.map(q => Math.max(q.el ?? 0, q.ed ?? 0, q.ek ?? 0)));
    const fe = mx > 2000 ? 0.001 : 1, un = mx > 2000 ? 'm' : 'mm';
    $('t-epos').textContent = `‖error de posición‖ [${un}] vs distancia recorrida`;
    const E = k => odom.map(q => [q.d, q[k] === null ? null : q[k]*fe]);
    chart($('g-epos'), [{p: E('el'), c:css('--rojo'), w:4, a:.55}, {p: E('ed'), c:css('--azul2'), w:1.6, dash:[6,5]},
                        {p: E('ek'), c:css('--verde'), w:2.2}], {cero:true, xl:'m'});
    chart($('g-epsi'), [{p: odom.map(q=>[q.d,q.sl]), c:css('--rojo'), w:4, a:.55}, {p: odom.map(q=>[q.d,q.sd]), c:css('--azul2'), w:1.6, dash:[6,5]},
                        {p: odom.map(q=>[q.d,q.sk]), c:css('--verde'), w:2.2}], {cero:true, xl:'m'});
    chart($('g-xy'), [{p: odom.map(q=>[q.xl,q.yl]), c:css('--rojo'), a:.7}, {p: odom.map(q=>[q.xr,q.yr]), c:'#ffffff', w:3.5},
                      {p: odom.map(q=>[q.xk,q.yk]), c:css('--verde'), w:1.6}], {igual:true});
  } else if (vista === 'wall'){
    const t1 = wall.length ? wall[wall.length-1].t : 0, w = wall.filter(q => q.t > t1 - 40);
    chart($('g-wall'), [{p: w.map(q=>[q.t-t1,q.e]), c:css('--cian'), w:2.2}, {p: w.map(q=>[q.t-t1,q.D]), c:'#ffffff', a:.8},
                        {p: w.map(q=>[q.t-t1,q.o]), c:css('--ambar'), a:.9}], {x0:-40, x1:0, cero:true, xl:'s'});
  }
}
window.addEventListener('resize', () => requestAnimationFrame(dibujarTodo));
async function reiniciar(){ await fetch('/reiniciar', {method:'POST'}); odom = []; wall = []; nOdom = nWall = 0; dibujarTodo(); }
$('reset-odo').onclick = reiniciar; $('reset-w').onclick = reiniciar;

// ---------------- tuning del wall_follow ----------------
const PRESETS = {
  seguro: {kp:3.0, ki:0.0, kd:0.4, L:0.4, d_des:0.30, v_max:1.0, v_min:0.15, w_max:2.5, v_estrecho:0.5, k_frente:1.5, d_seg:0.25},
  rapido: {kp:4.5, ki:0.0, kd:0.4, L:0.4, d_des:0.30, v_max:1.0, v_min:0.4, w_max:4.0, v_estrecho:0.8, k_frente:1.5, d_seg:0.25},
};
let inicial = null, actuales = {};
function msg(t, ok){ $('msg').textContent = t; $('msg').style.color = ok ? css('--verde') : css('--rojo'); }
async function cargarParams(){
  try {
    const d = await (await fetch('/wall')).json();
    if (!d.ok){ $('params-nota').textContent = d.error; return; }
    actuales = d.valores; if (!inicial) inicial = {...d.valores};
    const P = $('params'); P.innerHTML = '';
    d.grupos.forEach(([g, ps]) => {
      P.insertAdjacentHTML('beforeend', `<div class="lbl">${g}</div>`);
      const G = document.createElement('div'); G.className = 'grupo'; P.appendChild(G);
      ps.forEach(([n, paso, ayuda]) => {
        G.insertAdjacentHTML('beforeend', `<div class="prm" data-n="${n}" data-paso="${paso}"><div class="nom">${n}</div><div class="ay">${ayuda}</div>
          <div class="ctl"><button data-d="-1">−</button><input inputmode="decimal" value="${+(+d.valores[n]).toFixed(3)}"><button data-d="1">+</button></div></div>`);
      });
    });
    P.querySelectorAll('.prm').forEach(p => {
      const inp = p.querySelector('input'), n = p.dataset.n, paso = +p.dataset.paso;
      const marcar = () => p.classList.toggle('cambio', Math.abs(+inp.value - actuales[n]) > 1e-9);
      inp.addEventListener('input', marcar);
      p.querySelectorAll('button').forEach(b => b.onclick = () => { inp.value = +(+inp.value + paso*b.dataset.d).toFixed(3); marcar(); });
    });
    msg(d.valores.activo ? 'wall_follow activo' : 'wall_follow detenido', d.valores.activo);
  } catch(e){ $('params-nota').textContent = 'Error: ' + e; }
}
async function enviarWall(cambios){
  const d = await (await fetch('/wall', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(cambios)})).json();
  if (d.ok){ msg('Aplicado: ' + Object.entries(cambios).map(([k,v]) => `${k}=${v}`).join(', '), true); await cargarParams(); }
  else msg(d.error, false);
}
$('w-apl').onclick = () => {
  const c = {}; document.querySelectorAll('.prm').forEach(p => { const v = +p.querySelector('input').value;
    if (isFinite(v) && Math.abs(v - actuales[p.dataset.n]) > 1e-9) c[p.dataset.n] = v; });
  Object.keys(c).length ? enviarWall(c) : msg('No hay cambios', true);
};
$('ini-stop').onclick = async () => { await enviarWall({activo: false}); msgIni($('msg').textContent, true); pedirDatos(); };
$('ini-reset').onclick = async () => { await reiniciar(); msgIni('Contadores y gráficas reiniciados', true); };
function msgIni(t, ok){ $('msg-ini').textContent = t; $('msg-ini').style.color = ok ? css('--verde') : css('--rojo'); }
document.querySelectorAll('[data-pre]').forEach(b => b.onclick = () => {
  const p = b.dataset.pre === 'inicial' ? inicial : PRESETS[b.dataset.pre]; if (!p) return;
  document.querySelectorAll('.prm').forEach(e => { const n = e.dataset.n; if (n in p){ e.querySelector('input').value = +(+p[n]).toFixed(3);
    e.classList.toggle('cambio', Math.abs(p[n] - actuales[n]) > 1e-9); } });
  msg('Preset cargado: revisa y toca "Aplicar cambios"', true);
});

// ---------------- arranque ----------------
async function empezar(){
  if (!window.isSecureContext){ $('aviso-ini').textContent = 'Abre la página con https:// para leer los sensores.'; return; }
  try { if (typeof DeviceMotionEvent !== 'undefined' && DeviceMotionEvent.requestPermission){
          if (await DeviceMotionEvent.requestPermission() !== 'granted') throw 0; } }
  catch(_){ $('aviso-ini').textContent = 'Sin permiso para los sensores de movimiento (el resto funciona).'; }
  window.addEventListener('devicemotion', onMotion);
  try { await document.documentElement.requestFullscreen(); } catch(_){}
  try { await screen.orientation.lock('landscape'); } catch(_){}
  try { await navigator.wakeLock.request('screen'); } catch(_){}
  $('inicio').style.display = 'none';
  if (!store.get('calibrado', false)) setTimeout(() => { calibrar(); store.set('calibrado', true); }, 300);
  setInterval(enviar, 50);
  setInterval(pedirDatos, 400);
  pintarVolante();
}
$('go').onclick = empezar;
$('op-hud').onclick = () => { const o = $('v-op'); o.classList.toggle('llenar'); store.set('llenar', o.classList.contains('llenar')); };
if (store.get('llenar', false)) $('v-op').classList.add('llenar');
$('cam-img').addEventListener('load', () => { $('cam-nada').style.display = 'none'; });
$('cam-img').addEventListener('error', () => { if (vista === 'op') $('cam-nada').textContent = 'Sin video (¿OpenCV instalado? ¿llega /image_raw?). Igual puedes manejar.'; });
document.querySelectorAll('[data-modo]').forEach(b => b.onclick = async () => {
  const modo = b.dataset.modo;
  if (modo === 'auto' && !confirm('¿Pasar a modo AUTÓNOMO? El robot arranca solo con el wall_follow.')) return;
  const d = await (await fetch('/modo', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({modo})})).json();
  const t = d.ok ? (modo === 'auto' ? 'Modo AUTÓNOMO: wall_follow activo' : 'Modo MANUAL: wall_follow detenido') : d.error;
  msg(t, d.ok); msgIni(t, d.ok);
  pedirDatos(); if (vista === 'wall') cargarParams();
});
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
