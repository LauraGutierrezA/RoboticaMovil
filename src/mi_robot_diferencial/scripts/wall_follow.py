#!/usr/bin/python3
"""
wall_follow.py — Seguidor de pared izquierda (interior) con LiDAR: dos haces + PID.
Modo autónomo de la competencia: publica en /cmd_vel_auto y corre DETRÁS del AEB.

    /scan ─► [ wall_follow ] ─► /cmd_vel_auto ─► [ aeb_mux (modo:=auto) ] ─► /cmd_vel

Navegación reactiva: cada barrido de /scan se convierte en (v, ω). No usa mapa, /odom ni
/ground_truth.

Geometría de dos haces (pista antihoraria ⇒ la pared interior queda a la IZQUIERDA):
    b = rango a +90°            (perpendicular, a la izquierda)
    a = rango a +90° − θ        (adelantado θ respecto a b; θ = 45°)
    α  = atan( (a cos θ − b) / (a sin θ) )     ángulo del robot respecto a la pared
                                               (α > 0: se aleja de la pared; α < 0: se acerca)
    D_t   = b cos α                            distancia perpendicular actual
    D_t+1 = D_t + L sin α                      proyectada una distancia L hacia adelante
    e     = D_t+1 − d_obj                      e > 0: lejos ⇒ girar a la izquierda (ω > 0)

Distancia objetivo (adaptación al estrechamiento de 0.40 m):
    d_obj = min(d_des, frac_centro · ancho),   ancho = D_t + D_der,  D_der = r(−90°)·cos α
    En el carril normal (0.80 m) el robot va a d_des de la pared interior (línea corta);
    en el estrechamiento d_obj baja a la mitad del ancho: se centra solo.

Control:
    ω = Kp e + Ki ∫e dt + Kd ė      (ė con Δt del header del /scan, filtrado), |ω| ≤ w_max
    v = v_max (1 − |ω|/w_max) ≥ v_min      y además:
        v ≤ v_estrecho                si ancho < ancho_estrecho   (más lento en el estrechamiento)
        v ≤ k_frente·(d_frente − d_seg) si hay algo al frente (frenado suave antes del AEB)

Publica para depurar / graficar (rqt_plot, ros2 bag):
    /wall/error  /wall/dist  /wall/alpha  /wall/d_obj  (std_msgs/Float64)

Uso:
    ros2 launch mi_robot_diferencial robot_yahboom.launch.py pista:=competencia modo:=auto
    (o a mano)  ros2 run mi_robot_diferencial wall_follow.py --ros-args -p use_sim_time:=true
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


def rango_en(ranges, angle_min, inc, range_max, ang, vecinos=2):
    """Promedio de los haces finitos alrededor del ángulo `ang` [rad] (filtra el ruido)."""
    n = len(ranges)
    i = int(round((ang - angle_min) / inc)) % n
    idx = [(i + k) % n for k in range(-vecinos, vecinos + 1)]
    v = [ranges[j] for j in idx if math.isfinite(ranges[j]) and ranges[j] > 0.0]
    return float(np.mean(v)) if v else float(range_max)


class SeguidorPared:
    """Lógica pura (sin ROS): se puede probar aparte."""

    def __init__(self, **p):
        self.p = p
        self.e_prev = None
        self.t_prev = None
        self.de_f = 0.0          # derivada filtrada
        self.integral = 0.0

    def reiniciar(self):
        self.e_prev = self.t_prev = None
        self.de_f = self.integral = 0.0

    def paso(self, ranges, angle_min, inc, range_max, t):
        p = self.p
        th = math.radians(p['theta_deg'])
        lado = 1.0 if p['lado'] == 'izquierda' else -1.0          # +90° izquierda, −90° derecha
        r = lambda ang: rango_en(ranges, angle_min, inc, range_max, ang, p['vecinos'])  # noqa: E731

        # --- geometría de dos haces
        b = r(lado * math.pi / 2)
        a = r(lado * (math.pi / 2 - th))
        alpha = math.atan2(a * math.cos(th) - b, a * math.sin(th))
        D = b * math.cos(alpha)
        D_fut = D + p['L'] * math.sin(alpha)

        # --- distancia objetivo: d_des, o el centro si el carril es angosto
        D_otro = r(-lado * math.pi / 2) * math.cos(alpha)
        ancho = D + D_otro
        d_obj = min(p['d_des'], p['frac_centro'] * ancho)
        e = D_fut - d_obj

        # --- PID sobre ω
        de = 0.0
        if self.t_prev is not None:
            dt = t - self.t_prev
            if dt > 1e-4:
                de = (e - self.e_prev) / dt
                self.integral += e * dt
                lim = p['i_max']
                self.integral = max(-lim, min(lim, self.integral))
        k = p['filtro_d']                                   # 0 = sin filtro
        self.de_f = k * self.de_f + (1.0 - k) * de
        self.e_prev, self.t_prev = e, t

        w = lado * (p['kp'] * e + p['ki'] * self.integral + p['kd'] * self.de_f)
        w = max(-p['w_max'], min(p['w_max'], w))

        # --- velocidad según el giro, el ancho y lo que haya al frente
        v = max(p['v_min'], p['v_max'] * (1.0 - abs(w) / p['w_max']))
        if ancho < p['ancho_estrecho']:
            v = min(v, p['v_estrecho'])
        frente = r(0.0)
        v = min(v, max(p['v_min'], p['k_frente'] * (frente - p['d_seg'])))

        info = dict(a=a, b=b, alpha=alpha, D=D, D_fut=D_fut, ancho=ancho, d_obj=d_obj,
                    e=e, de=self.de_f, frente=frente)
        return v, w, info


PARAMS = dict(
    lado='izquierda',      # pared que se sigue: izquierda (interior, pista antihoraria) | derecha
    theta_deg=45.0,        # [°] separación entre los haces a y b
    vecinos=2,             # haces vecinos a cada lado que se promedian (5 en total)
    L=0.4,                 # [m] proyección hacia adelante
    d_des=0.30,            # [m] distancia deseada a la pared interior
    frac_centro=0.5,       # d_obj ≤ frac_centro·ancho (0.5 = centrarse si el carril es angosto)
    kp=3.0,                # [rad/s por m]
    ki=0.0,
    kd=0.4,
    filtro_d=0.5,          # filtro pasa-bajos de ė: 0 = nada, 0.9 = mucho
    i_max=0.3,             # [m·s] anti-windup
    w_max=2.5,             # [rad/s]
    v_max=1.0,             # [m/s]
    v_min=0.15,            # [m/s]
    ancho_estrecho=0.55,   # [m] por debajo, el carril se considera estrechamiento
    v_estrecho=0.5,        # [m/s] velocidad máxima en el estrechamiento
    k_frente=1.5,          # [1/s] v ≤ k_frente·(d_frente − d_seg)
    d_seg=0.25,            # [m] distancia al frente (desde el LiDAR) donde v llega a v_min
)


class NodoWallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow')
        for k, v in PARAMS.items():
            self.declare_parameter(k, v)
        self.seg = SeguidorPared(**self.leer())
        self.add_on_set_parameters_callback(self.cb_params)

        self.create_subscription(LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        self.dbg = {n: self.create_publisher(Float64, f'/wall/{n}', 10)
                    for n in ('error', 'dist', 'alpha', 'd_obj')}
        self.get_logger().info(
            'Wall following ({lado}): d_des={d_des} m, L={L} m, Kp={kp}, Ki={ki}, Kd={kd}, '
            'v_max={v_max} m/s -> /cmd_vel_auto'.format(**self.leer()))

    def leer(self):
        return {k: self.get_parameter(k).value for k in PARAMS}

    def cb_params(self, params):
        # ros2 param set /wall_follow kp 4.0 -> se aplica en el siguiente /scan
        from rcl_interfaces.msg import SetParametersResult
        nuevos = self.leer()
        for prm in params:
            if prm.name in nuevos:
                nuevos[prm.name] = prm.value
        self.seg.p = nuevos
        if any(prm.name in ('kp', 'ki', 'kd') for prm in params):
            self.seg.integral = 0.0
        return SetParametersResult(successful=True)

    def cb_scan(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v, w, info = self.seg.paso(msg.ranges, msg.angle_min, msg.angle_increment,
                                   msg.range_max, t)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.pub.publish(cmd)
        for n, k in (('error', 'e'), ('dist', 'D'), ('alpha', 'alpha'), ('d_obj', 'd_obj')):
            self.dbg[n].publish(Float64(data=float(info[k])))


def main():
    rclpy.init()
    nodo = NodoWallFollow()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            nodo.pub.publish(Twist())       # detener al cerrar
        except Exception:  # noqa: BLE001
            pass
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
