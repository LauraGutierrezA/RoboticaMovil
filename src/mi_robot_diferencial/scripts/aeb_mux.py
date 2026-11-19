#!/usr/bin/python3
"""
aeb_mux.py — Mux de comandos + Frenado Automático de Emergencia (AEB) con LiDAR
Robot MicroROS (Yahboom) en Gazebo (ROS 2 Jazzy + ros_gz)

Arquitectura (este es el ÚNICO nodo que publica en /cmd_vel):

    /cmd_vel_teleop ─┐
                     ├─► [ mux: modo=teleop|auto ] ─► [ AEB ] ─► /cmd_vel ─► DiffDrive
    /cmd_vel_auto  ──┘                                  ▲    ▲
                                              /scan ────┘    └── /odom (v, ω medidas)

Teleop con remap:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop

Idea física (fórmula del reporte):
    Para un obstáculo estático visto en el rayo i (rango r_i, ángulo th_i), la tasa de
    cambio del rango es  r_dot_i = -v*cos(th_i).
    TTC_i = r_i / max(-r_dot_i, 0)      ->  frenar si min(TTC_i) < umbral
    r_i se mide desde el BORDE del robot y no desde el LiDAR (que está en el centro):
    así el TTC es el tiempo hasta el contacto real.

    Umbral mínimo físicamente necesario (latencia + desaceleración constante a):
        d_frenado = v*t_lat + v^2/(2a)   y   d = v*TTC   ->   TTC_min = t_lat + v/(2a)
    MicroROS: v_max = 1.0 m/s, a_max = 2.0 m/s², t_lat = 0.10 s (LiDAR a 10 Hz)
        -> TTC_min(v_max) = 0.35 s.  Umbral base 0.5 s = 0.25 (frenado) + 0.10 (LiDAR)
           + 0.15 (holgura ≈ 0.15 m a v_max).

Modos (parámetro `mode`):
    'corridor' (por defecto): solo cuentan los rayos dentro de la franja que barrerá el
        robot (semiancho + margen) a lo largo de la trayectoria predicha con (v, ω):
        recta si ω ≈ 0, arco de radio v/ω si no. Así no frenan los muros laterales del
        estrechamiento de 0.40 m ni el muro exterior de una curva que sí se está tomando.
    'ittc': todos los rayos que se acercan (la fórmula pura). OJO: frena en el
        estrechamiento (muros a 0.2 m dan TTC ≈ 0.4/v). Solo para comparar en el reporte.

Decisión (cada 1/rate s, aunque no lleguen comandos nuevos: el DiffDrive mantiene el último):
    ttc_meas = TTC con (v, ω) medidas     ->  ¿ya voy hacia algo?            (emergencia)
    ttc_cmd  = TTC con (v, ω) comandadas  ->  ¿el comando me lleva hacia algo? (inseguro)
    La v lineal pasa solo si el comando es seguro y, en emergencia, solo si va en sentido
    contrario al movimiento actual (alejarse).
    Giro: con la v lineal en cero, girar en el sitio solo se permite si cabe. El robot es
    un rectángulo: al girar, sus esquinas barren un círculo de radio
    R_giro = hypot(max(frente, atras), semiancho) ≈ 0.14 m. Si hay un punto del LiDAR dentro
    de R_giro + margen_giro, girar choca y se anula ω (proteger_giro).
    Rearme (solo teleop): tras un frenado, el MISMO comando que se frenó sigue bloqueado
    aunque el corredor quede libre (p. ej. porque el robot rotó). Se libera cuando cambia
    el comando (otra tecla, k, girar). Sin esto el robot reanuda solo hacia el siguiente muro.
    Evaluar también el comando evita los tirones: al frenar v_meas -> 0 y ttc_meas -> inf,
    pero mientras se siga pidiendo avanzar hacia el obstáculo ttc_cmd mantiene el freno.
    Histéresis: estando activo, el umbral se multiplica por `histeresis` para soltar.

Mux:
    `modo` = teleop | auto elige la fuente (la otra se ignora).
    Si la fuente activa calla más de timeout_<modo> s manda cero. En teleop viene en 0
    (se mantiene el último comando: teleop_twist_keyboard solo publica al presionar teclas).
    Sin /scan reciente (timeout_scan) no se deja avanzar.

Publica:
    /cmd_vel      (geometry_msgs/Twist)  comando final
    /aeb/activo   (std_msgs/Bool)        True mientras frena
    /aeb/ttc_min  (std_msgs/Float64)     min(ttc_meas, ttc_cmd) [s], con tope ttc_tope para rqt_plot

Uso (lo lanza robot_yahboom.launch.py con modo:=teleop|auto):
    ros2 run mi_robot_diferencial aeb_mux.py --ros-args -p use_sim_time:=true -p modo:=teleop
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64


class AEBNode(Node):
    def __init__(self):
        super().__init__('aeb_mux')

        # %% Parámetros (ajustables con --ros-args -p nombre:=valor o ros2 param set)
        dp = self.declare_parameter
        dp('modo', 'teleop')            # teleop | auto     (fuente del mux)
        dp('mode', 'corridor')          # 'corridor' | 'ittc' (qué rayos cuentan)
        dp('ttc_threshold', 0.5)        # [s] umbral base
        dp('a_max', 2.0)                # [m/s^2] desaceleración del MicroROS (tabla)
        dp('t_latency', 0.10)           # [s] LiDAR a 10 Hz: hasta un escaneo de retraso
        dp('histeresis', 1.3)           # suelta cuando TTC > histeresis * umbral
        # Huella vista desde el LiDAR (las pasa el launch según robot:=microros|pro)
        dp('frente', 0.118)             # [m] LiDAR -> borde delantero (lente de la cámara)
        dp('atras', 0.0805)             # [m] LiDAR -> borde trasero
        dp('semiancho', 0.0765)         # [m] mitad del ancho (cara exterior de las llantas)
        dp('margen_lateral', 0.03)      # [m] margen del corredor a cada lado
        dp('w_min', 0.05)               # [rad/s] por debajo el corredor es recto
        dp('v_min', 0.01)               # [m/s] por debajo no se evalúa
        dp('permitir_giro', True)       # dejar pasar ω al frenar (para alejarse)...
        dp('proteger_giro', True)       # ...pero solo si girar en el sitio no choca
        dp('margen_giro', 0.02)         # [m] margen sobre el radio que barren las esquinas
        dp('rearmar_teleop', True)      # tras frenar, el mismo comando sigue bloqueado
        dp('timeout_teleop', 0.0)       # [s] 0 = mantener el último comando
        dp('timeout_auto', 0.5)         # [s]
        dp('timeout_scan', 0.5)         # [s] sin LiDAR no se avanza
        dp('ttc_tope', 10.0)            # [s] tope de /aeb/ttc_min para graficar
        dp('rate', 20.0)                # [Hz] lazo de control

        self.modo = self.p('modo')
        if self.modo not in ('teleop', 'auto'):
            raise ValueError(f"modo debe ser 'teleop' o 'auto', no '{self.modo}'")

        # %% Estado
        self.r = None          # distancia desde el BORDE del robot (rayos válidos)
        self.th = None         # ángulos correspondientes
        self.x = self.y = None # puntos en el marco del LiDAR
        self.autohits = None   # ángulos de rayos que pegan en el propio robot
        self.t_scan = None
        self.v_meas = 0.0      # velocidad lineal medida (/odom)
        self.w_meas = 0.0      # velocidad angular medida (/odom)
        self.cmd = Twist()     # último comando de la fuente activa
        self.t_cmd = None
        self.activo = False
        self.rho = None        # distancia de cada punto al LiDAR (sin restar el borde)
        self.bloqueado = None  # (v, ω) del comando que frenó el AEB (rearme)

        # %% Comunicación
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel_' + self.modo, self.cmd_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_active = self.create_publisher(Bool, '/aeb/activo', 10)
        self.pub_ttc = self.create_publisher(Float64, '/aeb/ttc_min', 10)

        self.create_timer(1.0 / self.p('rate'), self.loop)
        self.get_logger().info(
            f"Modo {self.modo.upper()}: /cmd_vel_{self.modo} -> AEB ({self.p('mode')}) -> /cmd_vel"
            f" | umbral {self.p('ttc_threshold'):.2f} s")

    def p(self, nombre):
        return self.get_parameter(nombre).value

    def ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ callbacks
    def scan_cb(self, msg: LaserScan):
        r = np.asarray(msg.ranges, dtype=float)
        th = msg.angle_min + np.arange(r.size) * msg.angle_increment
        # Descartar inf, nan y lecturas fuera de rango del sensor
        valid = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        r, th = r[valid], th[valid]

        # Distancia del LiDAR al borde de la huella (rectángulo) en la dirección th
        c, s = np.cos(th), np.sin(th)
        inf = np.full_like(r, np.inf)
        tx = np.where(c > 1e-9, np.divide(self.p('frente'), c, out=inf.copy(), where=c > 1e-9),
                      np.divide(self.p('atras'), -c, out=inf.copy(), where=c < -1e-9))
        ty = np.divide(self.p('semiancho'), np.abs(s), out=inf.copy(), where=np.abs(s) > 1e-9)
        borde = np.minimum(tx, ty)

        # Un retorno dentro de la huella (+1 cm) no es un obstáculo: es el propio robot
        auto = r <= borde + 0.01
        self.autohits = th[auto]
        keep = ~auto
        self.th = th[keep]
        self.x, self.y = r[keep] * c[keep], r[keep] * s[keep]
        self.rho = r[keep]                      # distancia al LiDAR (para el giro)
        self.r = r[keep] - borde[keep]          # r_i desde el borde del robot
        self.t_scan = self.ahora()

    def odom_cb(self, msg: Odometry):
        self.v_meas = msg.twist.twist.linear.x
        self.w_meas = msg.twist.twist.angular.z

    def cmd_cb(self, msg: Twist):
        self.cmd, self.t_cmd = msg, self.ahora()

    # ------------------------------------------------------------------ TTC
    def en_corredor(self, v: float, w: float) -> np.ndarray:
        """Máscara: ¿el punto (x, y) está en la franja que barrerá el robot con (v, ω)?"""
        x, y = self.x, self.y
        ancho = self.p('semiancho') + self.p('margen_lateral')
        if abs(w) < self.p('w_min') or abs(v / w) > 50.0:
            # Recta: franja del ancho del robot hacia adelante (v>0) o atrás (v<0)
            return ((x > 0.0) if v > 0 else (x < 0.0)) & (np.abs(y) < ancho)
        # Arco de radio con signo R = v/ω, centro en (0, R)
        R = v / w
        rho = np.hypot(x, y - R)
        dentro = np.abs(rho - abs(R)) < ancho
        # Ángulo recorrido sobre el arco hasta el punto, en el sentido del movimiento
        ang = np.arctan2(x, (R - y) if R > 0 else (y - R)) * (1.0 if v > 0 else -1.0)
        ang = np.mod(ang, 2.0 * math.pi)
        return dentro & (ang > 0.0) & (ang < math.pi)      # adelante, hasta media vuelta

    def compute_ttc(self, v: float, w: float):
        """(TTC mínimo [s], ángulo del rayo [rad]) si el robot se mueve con (v, ω)."""
        if self.r is None or self.r.size == 0 or abs(v) < self.p('v_min'):
            return math.inf, None

        closing = v * np.cos(self.th)            # = -r_dot
        mask = closing > 0.0                     # solo rayos que se acercan
        if self.p('mode') == 'corridor':
            mask &= self.en_corredor(v, w)       # y que estén en la trayectoria
        if not mask.any():
            return math.inf, None

        ttc = np.clip(self.r[mask], 0.0, None) / closing[mask]
        i = int(np.argmin(ttc))
        return float(ttc[i]), float(self.th[mask][i])

    def giro_choca(self) -> bool:
        """¿Girar en el sitio barre algún punto? (esquinas: círculo de radio R_giro)."""
        if self.rho is None or self.rho.size == 0:
            return False
        r_giro = math.hypot(max(self.p('frente'), self.p('atras')), self.p('semiancho'))
        return bool(np.any(self.rho < r_giro + self.p('margen_giro')))

    def threshold(self, v: float) -> float:
        """Umbral dinámico: nunca menor que t_lat + |v|/(2a); con histeresis si ya frena."""
        base = max(self.p('ttc_threshold'), self.p('t_latency') + abs(v) / (2.0 * self.p('a_max')))
        return base * (self.p('histeresis') if self.activo else 1.0)

    # ------------------------------------------------------------------ lazo
    def loop(self):
        t = self.ahora()

        # 1) Mux: último comando de la fuente activa (o cero si está vencido / no ha llegado)
        timeout = self.p('timeout_' + self.modo)
        v_cmd = w_cmd = 0.0
        if self.t_cmd is not None and (timeout <= 0.0 or t - self.t_cmd <= timeout):
            v_cmd, w_cmd = self.cmd.linear.x, self.cmd.angular.z

        # 2) AEB
        out = Twist()
        out.angular.z = w_cmd
        if self.t_scan is None or t - self.t_scan > self.p('timeout_scan'):
            # Sin LiDAR no se deja avanzar
            ttc, ang = math.nan, None
            active = abs(v_cmd) > 0.0
            out.linear.x = 0.0
        else:
            ttc_meas, ang_meas = self.compute_ttc(self.v_meas, self.w_meas)  # ¿ya voy hacia algo?
            ttc_cmd, ang_cmd = self.compute_ttc(v_cmd, w_cmd)               # ¿el comando me lleva?

            emergency = ttc_meas < self.threshold(self.v_meas)
            unsafe_cmd = ttc_cmd < self.threshold(v_cmd)

            # La v lineal pasa solo si es segura y, en emergencia, solo si va en sentido
            # contrario al movimiento actual (alejarse).
            allow_linear = (not unsafe_cmd) and not (emergency and v_cmd * self.v_meas > 0.0)
            out.linear.x = v_cmd if allow_linear else 0.0

            active = emergency or (unsafe_cmd and abs(v_cmd) > 0.0)
            ttc, ang = (ttc_meas, ang_meas) if ttc_meas <= ttc_cmd else (ttc_cmd, ang_cmd)

            # Rearme (teleop): el mismo comando que se frenó sigue frenado hasta que cambie
            if self.modo == 'teleop' and self.p('rearmar_teleop'):
                if abs(v_cmd) > 0.0 and out.linear.x == 0.0 and active:
                    self.bloqueado = (v_cmd, w_cmd)
                elif self.bloqueado is not None:
                    if (v_cmd, w_cmd) == self.bloqueado:
                        out.linear.x, active = 0.0, True
                    else:
                        self.bloqueado = None

        # Giro: pasa al frenar (para alejarse), pero solo si girar en el sitio no choca
        if out.linear.x == 0.0 and abs(out.angular.z) > 0.0:
            if (active and not self.p('permitir_giro')) or \
               (self.p('proteger_giro') and self.giro_choca()):
                out.angular.z = 0.0

        self.pub_cmd.publish(out)
        self.pub_active.publish(Bool(data=bool(active)))
        self.pub_ttc.publish(Float64(data=float(min(ttc, self.p('ttc_tope')))
                                     if not math.isnan(ttc) else math.nan))

        # Mensajes solo en los cambios de estado
        if active != self.activo:
            if active:
                self.get_logger().warn(
                    f'AEB: FRENO  (TTC mín {ttc:.2f} s' +
                    (f', rayo a {math.degrees(ang):.0f}°)' if ang is not None else ', sin /scan)'))
            else:
                self.get_logger().info('AEB: libre')
            self.activo = active

        if self.autohits is not None and self.autohits.size:
            a = np.degrees(self.autohits)
            self.get_logger().warn(
                f'{a.size} rayos del LiDAR pegan en el propio robot (entre {a.min():.0f}° y '
                f'{a.max():.0f}°): se ignoran en el AEB', throttle_duration_sec=10.0)


def main():
    rclpy.init()
    node = AEBNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.pub_cmd.publish(Twist())   # dejar el robot quieto al salir
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
