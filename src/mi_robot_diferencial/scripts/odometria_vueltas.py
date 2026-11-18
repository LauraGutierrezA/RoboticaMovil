#!/usr/bin/python3
"""
odometria_vueltas.py — Odometría por llantas para el análisis de varias vueltas.

Requisito 2 del reporte: "tomando ω_D y ω_I como el promedio de las dos ruedas de cada lado".

    ω_I = (ω_izq_del + ω_izq_tra) / 2        ω_D = (ω_der_del + ω_der_tra) / 2
    u   = a (ω_D + ω_I) / 2                  r   = a (ω_D − ω_I) / b
    x_{k+1} = x_k + u cos ψ_k Δt,  y_{k+1} = y_k + u sin ψ_k Δt,  ψ_{k+1} = ψ_k + r Δt   (Euler)

  a = radio de rueda, b = vía (distancia entre centros de llanta izquierda y derecha).
  Es la misma cinemática de odometria_llantas.py: con las 4 llantas iguales, la pseudoinversa
  W = (QᵀQ)⁻¹Qᵀ da exactamente estos u y r (W·w promedia las llantas de cada lado).

  a y b se leen del URDF publicado en /robot_description (colisión cilíndrica de la rueda y
  posición en y de los joints), así que siguen a parametros_microros.xacro sin tocar este nodo.

  La pose arranca en la pose REAL inicial (primer mensaje de /ground_truth), para que el error
  medido después sea solo deriva de la odometría.

Publica:  /odom_llantas  (nav_msgs/Odometry, frame "world", stamp = el del /joint_states)

Uso:  ros2 run mi_robot_diferencial odometria_vueltas.py --ros-args -p use_sim_time:=true
"""
import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def yaw_de_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_a_seg(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def geometria_desde_urdf(urdf_xml, joints_izq, joints_der):
    """Devuelve (a, b): radio de la rueda y vía, leídos del URDF expandido."""
    root = ET.fromstring(urdf_xml)
    joints = {j.get('name'): j for j in root.findall('joint')}
    links = {l.get('name'): l for l in root.findall('link')}
    radios, y_izq, y_der = [], [], []
    for nombre, lista in [(n, y_izq) for n in joints_izq] + [(n, y_der) for n in joints_der]:
        j = joints[nombre]
        o = j.find('origin')
        lista.append(float(o.get('xyz', '0 0 0').split()[1]))
        link = links[j.find('child').get('link')]
        for tag in ('collision', 'visual'):
            cil = link.find(f'{tag}/geometry/cylinder')
            if cil is not None:
                radios.append(float(cil.get('radius')))
                break
    if len(radios) != len(joints_izq) + len(joints_der):
        raise ValueError('no se encontró un cilindro en la colisión de todas las ruedas')
    a = sum(radios) / len(radios)
    b = abs(sum(y_izq) / len(y_izq) - sum(y_der) / len(y_der))
    return a, b


class OdometriaVueltas:
    """Integrador puro (sin ROS): se puede probar aparte."""

    def __init__(self, a, b):
        self.a, self.b = a, b
        self.x = self.y = self.psi = 0.0
        self.t = None
        self.u = self.r = 0.0            # velocidades del último /joint_states
        self.u_prev = self.r_prev = 0.0  # velocidades del anterior (Euler explícito)

    def iniciar(self, x, y, psi, t):
        self.x, self.y, self.psi, self.t = x, y, psi, t

    def paso(self, t, w_izq, w_der):
        """w_izq, w_der: listas con las velocidades angulares [rad/s] de las ruedas de cada lado."""
        w_i = sum(w_izq) / len(w_izq)
        w_d = sum(w_der) / len(w_der)
        self.u = self.a * (w_d + w_i) / 2.0
        self.r = self.a * (w_d - w_i) / self.b
        dt = t - self.t
        if dt > 0.0:
            # Euler con la velocidad del instante anterior → η_{k+1} = η_k + η̇_k Δt
            self.x += self.u_prev * math.cos(self.psi) * dt
            self.y += self.u_prev * math.sin(self.psi) * dt
            self.psi = math.atan2(math.sin(self.psi + self.r_prev * dt),
                                  math.cos(self.psi + self.r_prev * dt))
        self.u_prev, self.r_prev = self.u, self.r
        self.t = t


class NodoOdometriaVueltas(Node):
    def __init__(self):
        super().__init__('odometria_vueltas')
        self.declare_parameter('joints_izq', ['joint_rueda_izq_del', 'joint_rueda_izq_tra'])
        self.declare_parameter('joints_der', ['joint_rueda_der_del', 'joint_rueda_der_tra'])
        self.declare_parameter('a', 0.0)   # > 0 para forzar el radio en vez de leerlo del URDF
        self.declare_parameter('b', 0.0)   # > 0 para forzar la vía
        self.declare_parameter('init_desde_ground_truth', True)

        self.j_izq = list(self.get_parameter('joints_izq').value)
        self.j_der = list(self.get_parameter('joints_der').value)
        self.odo = None
        self.listo = not self.get_parameter('init_desde_ground_truth').value
        self.pose0 = None

        qos_datos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST)
        qos_latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self.cb_urdf, qos_latched)
        self.create_subscription(JointState, '/joint_states', self.cb_joints, qos_datos)
        self.create_subscription(Odometry, '/ground_truth', self.cb_gt, qos_datos)
        self.pub = self.create_publisher(Odometry, '/odom_llantas', 10)

    def cb_urdf(self, msg):
        if self.odo is not None:
            return
        a, b = self.get_parameter('a').value, self.get_parameter('b').value
        try:
            a_u, b_u = geometria_desde_urdf(msg.data, self.j_izq, self.j_der)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'No pude leer a y b del URDF: {e}')
            return
        a = a if a > 0 else a_u
        b = b if b > 0 else b_u
        self.odo = OdometriaVueltas(a, b)
        self.get_logger().info(f'Geometría: a = {a:.4f} m, b = {b:.4f} m  (u = a(ωD+ωI)/2, r = a(ωD−ωI)/b)')

    def cb_gt(self, msg):
        if self.pose0 is None:
            p = msg.pose.pose
            self.pose0 = (p.position.x, p.position.y, yaw_de_quat(p.orientation))
            self.get_logger().info('Pose inicial real: x = %.3f, y = %.3f, ψ = %.3f' % self.pose0)

    def cb_joints(self, msg):
        if self.odo is None:
            return
        t = stamp_a_seg(msg.header.stamp)
        idx = {n: i for i, n in enumerate(msg.name)}
        if any(n not in idx for n in self.j_izq + self.j_der) or len(msg.velocity) != len(msg.name):
            return
        if self.odo.t is None:
            if not self.listo:
                if self.pose0 is None:
                    return          # esperando la pose real inicial
                self.odo.iniciar(*self.pose0, t)
            else:
                self.odo.iniciar(0.0, 0.0, 0.0, t)
        self.odo.paso(t, [msg.velocity[idx[n]] for n in self.j_izq],
                      [msg.velocity[idx[n]] for n in self.j_der])

        o = Odometry()
        o.header.stamp = msg.header.stamp
        o.header.frame_id = 'world'
        o.child_frame_id = 'base_footprint'
        o.pose.pose.position.x = self.odo.x
        o.pose.pose.position.y = self.odo.y
        o.pose.pose.orientation.z = math.sin(self.odo.psi / 2.0)
        o.pose.pose.orientation.w = math.cos(self.odo.psi / 2.0)
        o.twist.twist.linear.x = self.odo.u
        o.twist.twist.angular.z = self.odo.r
        self.pub.publish(o)


def main():
    rclpy.init()
    nodo = NodoOdometriaVueltas()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
