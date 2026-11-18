#!/usr/bin/python3
"""
aeb_mux.py — Selector de comandos (mux) + frenado autónomo de emergencia (AEB).

Requisito 7: los comandos de teleoperación y de autonomía van en tópicos separados y un nodo
los lleva a /cmd_vel pasando por el AEB. Requisito 3: el AEB tiene prioridad sobre cualquier
comando, en los dos modos.

    /cmd_vel_teleop ─┐
                     ├─► [ mux: elige según `modo` ] ─► [ AEB ] ─► /cmd_vel ─► DiffDrive
    /cmd_vel_auto  ──┘                                     ▲
                                              /scan, /odom ┘

AEB (fórmula del reporte), para cada rayo i del LiDAR con ángulo θ_i y distancia r_i:
    ṙ_i = −v cos θ_i            TTC_i = r_i / max(−ṙ_i, 0)
  y se frena si mín_i TTC_i < ttc_umbral.

Decisiones de diseño (para evitar frenados falsos en curvas y en el estrechamiento):
  1. Solo cuentan los rayos que caen en el CORREDOR que va a barrer el robot: una franja del
     ancho del robot + margen a lo largo de la trayectoria predicha con (v, ω). Si ω ≈ 0 es una
     franja recta; si no, es un arco de radio v/ω. Así los muros laterales del estrechamiento
     (0.40 m de ancho, robot de 0.153 m) y el muro exterior de una curva que el robot sí está
     tomando no disparan el freno.
  2. r_i se mide desde el BORDE del robot y no desde el centro del LiDAR (que está en medio
     del robot): r_i ← r_i − e(θ_i), con e la distancia del LiDAR al borde de la huella en esa
     dirección. Así TTC es el tiempo hasta el contacto real.
  3. v = la mayor (en magnitud) entre la velocidad medida (/odom) y la comandada, si van en el
     mismo sentido. Con solo la medida, al frenar v → 0, TTC → ∞, se libera, acelera y vuelve a
     frenar (avance a tirones contra el obstáculo). Con la comandada el freno se mantiene mientras
     se siga pidiendo avanzar hacia el obstáculo.
  4. Al frenar se anula la velocidad lineal; el giro (ω) puede pasar (permitir_giro) para que el
     operador o el controlador puedan girar y alejarse. Reversa también pasa si atrás está libre
     (el mismo cálculo con v < 0 usa los rayos traseros).
  5. Histéresis: se suelta cuando mín TTC > histeresis · ttc_umbral, para no tartamudear.

Umbral por defecto (0.5 s), con v_max = 1.0 m/s y a_max = 2.0 m/s² (tabla del reporte):
    t_frenado = v_max / (2·a_max) = 0.25 s   (frenando a a_max se recorre v²/2a = 0.25 m,
                                              que a v_max equivalen a 0.25 s de TTC)
  + t_lidar   = 0.10 s                       (el LiDAR va a 10 Hz: hasta 1 escaneo de retraso)
  + t_margen  = 0.15 s                       (≈ 0.15 m de holgura a v_max)
  = 0.50 s

Mux:
  - `modo` = teleop | auto elige la fuente. La otra se ignora.
  - Publica /cmd_vel a 20 Hz con el último comando de la fuente activa, revisado por el AEB
    con el escaneo más reciente (el DiffDrive mantiene el último comando, así que el AEB tiene
    que poder frenar aunque no lleguen comandos nuevos).
  - Timeout: si la fuente activa deja de publicar más de `timeout_<modo>` s, manda cero.
    En teleop viene desactivado (0): teleop_twist_keyboard publica solo al presionar teclas y
    el robot debe seguir con el último comando, como sin mux.

Publica además, para depurar y para el pitch:
  /aeb/activo   (std_msgs/Bool)     True mientras frena
  /aeb/ttc_min  (std_msgs/Float64)  mínimo TTC del corredor [s] (inf si no hay peligro)

Uso (lo lanza robot_yahboom.launch.py con modo:=teleop|auto):
  ros2 run mi_robot_diferencial aeb_mux.py --ros-args -p use_sim_time:=true -p modo:=teleop
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64


class AEB:
    """Cálculo puro del AEB (sin ROS)."""

    def __init__(self, ttc_umbral=0.5, histeresis=1.3, frente=0.118, atras=0.0805,
                 semiancho=0.0765, margen_lateral=0.03, w_min=0.05):
        self.ttc_umbral = ttc_umbral
        self.histeresis = histeresis
        self.frente, self.atras, self.semiancho = frente, atras, semiancho  # huella desde el LiDAR
        self.margen = margen_lateral
        self.w_min = w_min
        self.activo = False
        self.autohits = []   # ángulos [rad] de rayos que pegaron dentro de la huella del robot
        self._cache = None   # (n, angle_min, inc) -> (cos, sin, borde)

    def _geometria(self, n, angle_min, inc):
        clave = (n, round(angle_min, 6), round(inc, 9))
        if self._cache is None or self._cache[0] != clave:
            cs, sn, borde = [], [], []
            for i in range(n):
                th = angle_min + i * inc
                c, s = math.cos(th), math.sin(th)
                # distancia del LiDAR al borde de la huella (rectángulo) en la dirección θ
                t = math.inf
                if c > 1e-9:
                    t = min(t, self.frente / c)
                elif c < -1e-9:
                    t = min(t, self.atras / -c)
                if abs(s) > 1e-9:
                    t = min(t, self.semiancho / abs(s))
                cs.append(c); sn.append(s); borde.append(t)
            self._cache = (clave, cs, sn, borde)
        return self._cache[1:]

    def en_corredor(self, x, y, v, w):
        """¿El punto (x, y) [marco del robot] está en la franja que barrerá el robot?"""
        ancho = self.semiancho + self.margen
        if abs(w) < self.w_min or abs(v / w) > 50.0:
            return (x > 0.0 if v > 0 else x < 0.0) and abs(y) < ancho
        R = v / w                                   # radio con signo; centro en (0, R)
        rho = math.hypot(x, y - R)
        if abs(rho - abs(R)) >= ancho:
            return False
        # ángulo recorrido sobre el arco hasta el punto, en el sentido del movimiento
        ang = math.atan2(x, (R - y) if R > 0 else (y - R)) * (1 if v > 0 else -1)
        return 0.0 < (ang % (2 * math.pi)) < math.pi   # adelante, hasta media vuelta

    def evaluar(self, ranges, angle_min, inc, range_min, range_max, v, w):
        """Devuelve (ttc_min, angulo_del_rayo, frenar)."""
        cs, sn, borde = self._geometria(len(ranges), angle_min, inc)
        ttc_min, ang_min = math.inf, None
        self.autohits = []
        for i, r in enumerate(ranges):
            if not (range_min <= r <= range_max) or math.isnan(r):
                continue
            # Un retorno dentro de la huella del robot (+1 cm) no puede ser un obstáculo:
            # es el propio robot.
            if r <= borde[i] + 0.01:
                self.autohits.append(angle_min + i * inc)
                continue
            if abs(v) > 1e-3:
                cierre = v * cs[i]                    # −ṙ_i = v cos θ_i
                if cierre <= 0.0:
                    continue
                x, y = r * cs[i], r * sn[i]
                if not self.en_corredor(x, y, v, w):
                    continue
                ttc = (r - borde[i]) / cierre          # TTC_i = r_i / max(−ṙ_i, 0), r desde el borde
                if ttc < ttc_min:
                    ttc_min, ang_min = ttc, angle_min + i * inc
        lim = self.ttc_umbral * (self.histeresis if self.activo else 1.0)
        self.activo = ttc_min < lim
        return ttc_min, ang_min, self.activo


def velocidad_a_evaluar(v_cmd, v_med):
    """Decisión 3: la mayor entre la comandada y la medida si van en el mismo sentido."""
    if v_cmd * v_med <= 0.0:
        return v_cmd if abs(v_cmd) > 1e-3 else v_med
    return math.copysign(max(abs(v_cmd), abs(v_med)), v_cmd)


class NodoAEBMux(Node):
    def __init__(self):
        super().__init__('aeb_mux')
        dp = self.declare_parameter
        dp('modo', 'teleop')                 # teleop | auto
        dp('ttc_umbral', 0.5)                # [s]  (justificación en el encabezado)
        dp('histeresis', 1.3)
        dp('frente', 0.118)                  # [m] LiDAR -> borde delantero (lente de la cámara)
        dp('atras', 0.0805)                  # [m] LiDAR -> borde trasero
        dp('semiancho', 0.0765)              # [m] mitad del ancho (cara exterior de las llantas)
        dp('margen_lateral', 0.03)           # [m]
        dp('permitir_giro', True)
        dp('timeout_teleop', 0.0)            # [s] 0 = mantener el último comando
        dp('timeout_auto', 0.5)              # [s]
        dp('timeout_scan', 0.5)              # [s] sin LiDAR: no se deja avanzar
        dp('frecuencia', 20.0)               # [Hz]
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        self.modo = gp('modo')
        if self.modo not in ('teleop', 'auto'):
            raise ValueError(f"modo debe ser 'teleop' o 'auto', no '{self.modo}'")
        self.aeb = AEB(gp('ttc_umbral'), gp('histeresis'), gp('frente'), gp('atras'),
                       gp('semiancho'), gp('margen_lateral'))
        self.permitir_giro = gp('permitir_giro')
        self.timeout = gp('timeout_' + self.modo)
        self.timeout_scan = gp('timeout_scan')

        self.cmd, self.t_cmd = Twist(), None
        self.scan, self.t_scan = None, None
        self.v_med = 0.0
        self.estaba_activo = False

        qos_s = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Twist, '/cmd_vel_' + self.modo, self.cb_cmd, 10)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, qos_s)
        self.create_subscription(Odometry, '/odom', self.cb_odom, qos_s)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_activo = self.create_publisher(Bool, '/aeb/activo', 10)
        self.pub_ttc = self.create_publisher(Float64, '/aeb/ttc_min', 10)
        self.create_timer(1.0 / gp('frecuencia'), self.cb_timer)
        self.get_logger().info(
            f"Modo {self.modo.upper()}: /cmd_vel_{self.modo} -> AEB -> /cmd_vel | "
            f"TTC umbral {gp('ttc_umbral'):.2f} s | timeout {self.timeout:.1f} s")

    def ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_cmd(self, msg):
        self.cmd, self.t_cmd = msg, self.ahora()

    def cb_scan(self, msg):
        self.scan, self.t_scan = msg, self.ahora()

    def cb_odom(self, msg):
        self.v_med = msg.twist.twist.linear.x

    def cb_timer(self):
        t = self.ahora()
        salida = Twist()
        # 1) mux: comando de la fuente activa (o cero si está vencido / no ha llegado)
        if self.t_cmd is not None and (self.timeout <= 0.0 or t - self.t_cmd <= self.timeout):
            salida.linear.x = self.cmd.linear.x
            salida.angular.z = self.cmd.angular.z

        # 2) AEB
        if self.scan is None or t - self.t_scan > self.timeout_scan:
            ttc, ang, frenar = math.nan, None, abs(salida.linear.x) > 0.0   # sin LiDAR no se avanza
        else:
            v = velocidad_a_evaluar(salida.linear.x, self.v_med)
            s = self.scan
            ttc, ang, frenar = self.aeb.evaluar(s.ranges, s.angle_min, s.angle_increment,
                                                s.range_min, s.range_max, v, salida.angular.z)
        if self.aeb.autohits:
            a = [math.degrees(x) for x in self.aeb.autohits]
            self.get_logger().warn(
                f'{len(a)} rayos del LiDAR pegan en el propio robot (entre {min(a):.0f}° y {max(a):.0f}°): '
                'se ignoran en el AEB', throttle_duration_sec=10.0)
        if frenar:
            salida.linear.x = 0.0
            if not self.permitir_giro:
                salida.angular.z = 0.0

        if frenar != self.estaba_activo:
            if frenar:
                self.get_logger().warn(
                    f'AEB: FRENO  (TTC mín {ttc:.2f} s' +
                    (f', rayo a {math.degrees(ang):.0f}°)' if ang is not None else ', sin /scan)'))
            else:
                self.get_logger().info('AEB: libre')
            self.estaba_activo = frenar

        self.pub.publish(salida)
        self.pub_activo.publish(Bool(data=bool(frenar)))
        self.pub_ttc.publish(Float64(data=float(ttc)))


def main():
    rclpy.init()
    nodo = NodoAEBMux()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            nodo.pub.publish(Twist())        # dejar el robot quieto al cerrar
        except Exception:  # noqa: BLE001
            pass
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
