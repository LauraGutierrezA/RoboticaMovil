#!/usr/bin/python3
"""
odometria_llantas.py — Odometría propia por cinemática de llantas (robot de 4 llantas)

Implementa los "Cálculos cinemáticos" del Notion:

    w_i = [1/a_i, tan(phi_i)/a_i] · R(theta_Bi)^T · [[1, 0, -dy_i], [0, 1, dx_i]] · [u, v, r]^T
    Q   = filas w_i  (4x3)  -> si la columna de v es cero se elimina (4x2)
    W   = (Q^T Q)^-1 Q^T    (pseudoinversa)
    zeta    = W · w           (velocidades del cuerpo [u, r])
    eta_dot = J(psi) · zeta   (velocidades en el marco global)
    eta_{k+1} = eta_k + eta_dot · dt   (Euler, dt = tiempo entre mensajes de /joint_states)

Comparación con /odom (plugin DiffDrive) SINCRONIZADA POR MARCA DE TIEMPO:
  cada pose propia se guarda con el instante de simulación del /joint_states que la produjo.
  Cada mensaje de /odom se compara con la pose propia de ESE MISMO instante (si no hay una
  con la marca exacta, se interpola entre las dos más cercanas). Si /odom llega antes que el
  /joint_states de su instante, se espera a que llegue.
  Se reporta también, solo como referencia, el error "sin sincronizar" (comparando contra la
  última pose propia disponible al llegar /odom), que era el método anterior.

Uso:
  ros2 run mi_robot_diferencial odometria_llantas.py --ros-args -p use_sim_time:=true

Para terminar: cierra la ventana de la gráfica (o Ctrl+C). Al salir guarda un CSV
e imprime el resumen de errores.
"""

import csv
import math
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

PALABRAS_LLANTA = ('wheel', 'rueda', 'llanta')
TOL_STAMP = 1e-6  # [s] dos marcas de tiempo a menos de esto se consideran el mismo instante


# ----------------------------------------------------------------------------- utilidades
def rpy_a_matriz(r, p, y):
    """Matriz de rotación a partir de roll-pitch-yaw (convención URDF: Rz·Ry·Rx)."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def fila_llanta(a, phi, theta, dx, dy):
    """
    Fila de Q para una llanta, sobre zeta = [u, v, r]:
        w_i = [1/a, tan(phi)/a] · R(theta)^T · [[1, 0, -dy], [0, 1, dx]]
    Con theta = phi = 0 queda [1/a, 0, -dy/a], igual que en el Notion.
    """
    rodadura = np.array([1.0 / a, math.tan(phi) / a])
    c, s = math.cos(theta), math.sin(theta)
    Rt = np.array([[c, s], [-s, c]])
    M = np.array([[1.0, 0.0, -dy], [0.0, 1.0, dx]])
    return rodadura @ Rt @ M


def envolver(ang):
    """Lleva un ángulo a [-pi, pi]."""
    return math.atan2(math.sin(ang), math.cos(ang))


def yaw_de_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def radio_de_link(link):
    """Radio de la llanta desde la geometría del link (collision primero, luego visual)."""
    for tag in ('collision', 'visual'):
        for elem in link.findall(tag):
            g = elem.find('geometry')
            if g is None:
                continue
            for forma in ('cylinder', 'sphere'):
                f = g.find(forma)
                if f is not None and f.get('radius'):
                    return float(f.get('radius'))
    return None  # p. ej. si la llanta es un mesh


def parametros_desde_urdf(urdf_xml, nombres_joints, base_frame):
    """
    Saca (a, theta, dx, dy) de cada llanta del URDF ya expandido.
      - dx, dy : posición del joint de la llanta respecto a base_frame
      - theta  : ángulo de la dirección de rodadura (x_llanta) respecto a x_cuerpo
      - a      : radio de la geometría del link hijo (None si no se puede leer)
    """
    root = ET.fromstring(urdf_xml)
    joints = {j.get('name'): j for j in root.findall('joint')}
    joint_por_hijo = {j.find('child').get('link'): j for j in root.findall('joint')}
    links = {l.get('name'): l for l in root.findall('link')}

    def origen(elem):
        o = elem.find('origin')
        xyz, rpy = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        if o is not None:
            xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
            rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
        return np.array(xyz), rpy_a_matriz(*rpy)

    resultado = []
    for nombre in nombres_joints:
        j = joints.get(nombre)
        if j is None:
            raise ValueError(f"el joint '{nombre}' no está en el URDF")

        p, R = origen(j)
        padre = j.find('parent').get('link')
        while padre != base_frame:
            jp = joint_por_hijo.get(padre)
            if jp is None:
                raise ValueError(f"no hay cadena desde '{nombre}' hasta '{base_frame}'")
            pp, Rp = origen(jp)
            p = pp + Rp @ p
            R = Rp @ R
            padre = jp.find('parent').get('link')

        eje_elem = j.find('axis')
        eje = np.array([float(v) for v in
                        (eje_elem.get('xyz') if eje_elem is not None else '1 0 0').split()])
        eje_cuerpo = R @ eje
        rodadura = np.cross(eje_cuerpo, [0.0, 0.0, 1.0])  # eje +y  ->  rodadura +x
        if np.linalg.norm(rodadura) < 1e-6:
            raise ValueError(f"el eje del joint '{nombre}' es vertical; no parece una llanta")
        theta = math.atan2(rodadura[1], rodadura[0])

        hijo = links.get(j.find('child').get('link'))
        a = radio_de_link(hijo) if hijo is not None else None

        resultado.append(dict(a=a, phi=0.0, theta=theta, dx=float(p[0]), dy=float(p[1])))
    return resultado


def stamp_a_seg(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


# ----------------------------------------------------------------------------- nodo
class OdometriaLlantas(Node):
    def __init__(self):
        super().__init__('odometria_llantas')

        # Parámetros (valores por defecto = los del Notion y tu robot)
        self.declare_parameter('wheel_joints', ['joint_rueda_izq_tra', 'joint_rueda_izq_del',
                                                'joint_rueda_der_tra', 'joint_rueda_der_del'])
        self.declare_parameter('from_urdf', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('urdf_timeout', 5.0)             # s esperando /robot_description
        self.declare_parameter('a', 0.024)                      # radio [m]
        self.declare_parameter('phi', [0.0, 0.0, 0.0, 0.0])     # [rad]
        self.declare_parameter('theta', [0.0, 0.0, 0.0, 0.0])   # [rad]
        self.declare_parameter('dx', [0.0, 0.095, 0.0, 0.095])  # [m]
        self.declare_parameter('dy', [0.06575, 0.06575, -0.06575, -0.06575])  # [m]
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('robot_description_topic', '/robot_description')
        self.declare_parameter('init_from_odom', False)  # True: arranca en la primera pose de /odom
        self.declare_parameter('csv_path', 'odometria_comparacion.csv')

        gp = lambda n: self.get_parameter(n).value
        self.from_urdf = gp('from_urdf')
        self.base_frame = gp('base_frame')
        self.urdf_timeout = gp('urdf_timeout')
        self.csv_path = gp('csv_path')

        # Estado del modelo
        self.urdf = None
        self.modelo_listo = False
        self.nombres = None
        self.W = None
        self.usa_v = False
        self.t_inicio_wall = time.monotonic()

        # Estado de la integración: eta = (x, y, psi), arranca en (0, 0, 0)
        self.x = self.y = self.psi = 0.0
        self.t_prev = None
        self.q_prev = None
        self.pose_lista = not gp('init_from_odom')

        # Sincronización por marca de tiempo
        self.historial = deque()                   # (t, x, y, psi) de la pose propia
        self.odom_pendientes = deque(maxlen=2000)  # (t, x, y, psi) de /odom esperando su pose
        self.n_exactas = 0                         # comparaciones con la misma marca de tiempo
        self.n_interp = 0                          # comparaciones interpoladas

        # Registro para gráfica / CSV
        self.lock = threading.Lock()
        self.t0 = None
        self.datos = {k: [] for k in ('t', 'x_p', 'y_p', 'psi_p', 'x_o', 'y_o', 'psi_o',
                                      'e_x', 'e_y', 'e_psi', 'e_pos', 'rmse_pos')}
        self.n = 0
        self.suma_ep2 = 0.0
        self.suma_epsi2 = 0.0
        self.dist_odom = 0.0
        self.xy_odom_prev = None
        # Referencia: error sin sincronizar (método anterior)
        self.n_naive = 0
        self.suma_ep2_naive = 0.0
        self.suma_epsi2_naive = 0.0

        # Suscripciones. Best-effort recibe tanto de publicadores reliable como best-effort.
        qos_datos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST)
        qos_latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, gp('robot_description_topic'),
                                 self.cb_robot_description, qos_latched)
        self.create_subscription(JointState, gp('joint_states_topic'),
                                 self.cb_joint_states, qos_datos)
        self.create_subscription(Odometry, gp('odom_topic'), self.cb_odom, qos_datos)

        self.get_logger().info('Nodo de odometría por llantas iniciado. Esperando datos...')

    # ------------------------------------------------------------------ modelo (Q y W)
    def cb_robot_description(self, msg):
        self.urdf = msg.data

    def _parametros_manuales(self):
        gp = lambda n: self.get_parameter(n).value
        a = gp('a')
        return [dict(a=a, phi=gp('phi')[i], theta=gp('theta')[i], dx=gp('dx')[i], dy=gp('dy')[i])
                for i in range(4)]

    def _intentar_construir_modelo(self, msg):
        log = self.get_logger()

        # 1) Nombres de los joints de las llantas
        nombres = list(self.get_parameter('wheel_joints').value)
        if nombres == ['auto'] or not nombres:
            nombres = sorted(n for n in msg.name if any(p in n.lower() for p in PALABRAS_LLANTA))
            if len(nombres) != 4:
                log.error(f'Detecté {len(nombres)} joints de llanta: {nombres}. Fija el parámetro '
                          f'wheel_joints con los 4 nombres en orden [1, 2, 3, 4].',
                          throttle_duration_sec=5.0)
                return False
            log.warn(f'wheel_joints detectados automáticamente (orden 1..4): {nombres}.')
        faltan = [n for n in nombres if n not in msg.name]
        if faltan:
            log.error(f'Estos joints no vienen en /joint_states: {faltan}. '
                      f'Disponibles: {list(msg.name)}', throttle_duration_sec=5.0)
            return False

        # 2) Parámetros por llanta (a, phi, theta, dx, dy)
        params, origen = None, 'parámetros manuales'
        if self.from_urdf:
            if self.urdf is None:
                if time.monotonic() - self.t_inicio_wall < self.urdf_timeout:
                    log.info('Esperando /robot_description...', throttle_duration_sec=2.0)
                    return False
                log.warn('No llegó /robot_description; uso los parámetros manuales.')
            else:
                try:
                    params = parametros_desde_urdf(self.urdf, nombres, self.base_frame)
                    phi = self.get_parameter('phi').value
                    a_manual = self.get_parameter('a').value
                    for i, p in enumerate(params):
                        p['phi'] = phi[i]
                        if p['a'] is None:
                            log.warn(f'No pude leer el radio de {nombres[i]} del URDF; uso a={a_manual}')
                            p['a'] = a_manual
                    origen = f'URDF (respecto a {self.base_frame})'
                except Exception as e:
                    log.warn(f'No pude sacar los parámetros del URDF ({e}); uso los manuales.')
        if params is None:
            params = self._parametros_manuales()

        # 3) Q, W  (igual que en el Notion)
        Q3 = np.array([fila_llanta(**p) for p in params])     # 4x3 sobre [u, v, r]
        self.usa_v = not np.allclose(Q3[:, 1], 0.0, atol=1e-9)
        Q = Q3 if self.usa_v else Q3[:, [0, 2]]               # sin columna de v -> 4x2
        self.W = np.linalg.pinv(Q)                            # = (Q^T Q)^-1 Q^T
        self.nombres = nombres
        self.modelo_listo = True

        # 4) Mostrar lo calculado para compararlo con el Notion
        np.set_printoptions(precision=6, suppress=True)
        tabla = '\n'.join(
            f'  llanta {i + 1} ({nombres[i]}): a={p["a"]:.4f}  phi={math.degrees(p["phi"]):.1f}°  '
            f'theta={math.degrees(p["theta"]):.1f}°  dx={p["dx"]:+.5f}  dy={p["dy"]:+.5f}'
            for i, p in enumerate(params))
        zeta_txt = '[u, v, r]' if self.usa_v else '[u, r]'
        log.info(f'Parámetros por llanta ({origen}):\n{tabla}\n'
                 f'Q ({Q.shape[0]}x{Q.shape[1]}, sobre {zeta_txt}):\n{Q}\n'
                 f'W = pinv(Q):\n{self.W}\n'
                 f'A = {self.W[0, 0]:.10f}   B = {abs(self.W[-1, 0]):.10f}')
        return True

    # ------------------------------------------------------------------ integración (Euler)
    def cb_joint_states(self, msg):
        if not self.modelo_listo and not self._intentar_construir_modelo(msg):
            return
        if not self.pose_lista:  # esperando la primera /odom (init_from_odom)
            return

        t = stamp_a_seg(msg.header.stamp) or self.get_clock().now().nanoseconds * 1e-9
        idx = {n: i for i, n in enumerate(msg.name)}
        try:
            q_actual = [msg.position[idx[n]] for n in self.nombres] if msg.position else None
        except (KeyError, IndexError):
            return

        if self.t_prev is None:
            self.t_prev, self.q_prev = t, q_actual
            self.historial.append((t, self.x, self.y, self.psi))
            self._procesar_odom_pendientes()
            return
        dt = t - self.t_prev
        if dt <= 0.0:
            return
        if dt > 0.5:  # salto de tiempo (pausa o reinicio de la simulación): no integrar
            self.t_prev, self.q_prev = t, q_actual
            self.historial.append((t, self.x, self.y, self.psi))
            self._procesar_odom_pendientes()
            return

        # w = velocidades angulares de las 4 llantas (velocity; si no viene, diferencia de position)
        try:
            if msg.velocity and len(msg.velocity) == len(msg.name):
                w = np.array([msg.velocity[idx[n]] for n in self.nombres])
            elif q_actual is not None and self.q_prev is not None:
                w = (np.array(q_actual) - np.array(self.q_prev)) / dt
            else:
                return
        except (KeyError, IndexError):
            return
        self.t_prev, self.q_prev = t, q_actual

        # zeta = W · w
        zeta = self.W @ w
        if self.usa_v:
            u, v, r = zeta
        else:
            (u, r), v = zeta, 0.0

        # eta_dot = J(psi) · zeta  y  Euler
        c, s = math.cos(self.psi), math.sin(self.psi)
        with self.lock:
            self.x += (c * u - s * v) * dt
            self.y += (s * u + c * v) * dt
            self.psi += r * dt

        # Guardar la pose con su instante y comparar las /odom que estaban esperando
        self.historial.append((t, self.x, self.y, self.psi))
        self._procesar_odom_pendientes()

    # ------------------------------------------------------------------ comparación con /odom
    def cb_odom(self, msg):
        xo = msg.pose.pose.position.x
        yo = msg.pose.pose.position.y
        psio = yaw_de_quat(msg.pose.pose.orientation)
        t = stamp_a_seg(msg.header.stamp) or self.get_clock().now().nanoseconds * 1e-9

        if not self.pose_lista:
            with self.lock:
                self.x, self.y, self.psi = xo, yo, psio
            self.pose_lista = True
            self.get_logger().info(f'Pose inicial tomada de /odom: ({xo:.3f}, {yo:.3f}, '
                                   f'{math.degrees(psio):.1f}°)')
        if not self.modelo_listo or self.t_prev is None:
            return

        # Referencia: error sin sincronizar (contra la última pose propia disponible)
        self.n_naive += 1
        self.suma_ep2_naive += math.hypot(self.x - xo, self.y - yo) ** 2
        self.suma_epsi2_naive += envolver(self.psi - psio) ** 2

        # Sincronizado: se compara con la pose propia del mismo instante
        self.odom_pendientes.append((t, xo, yo, psio))
        self._procesar_odom_pendientes()

    def _pose_propia_en(self, t_o):
        """
        Pose propia en el instante t_o. Devuelve (x, y, psi, exacta) o None si no se puede.
        Supone que t_o ya está cubierto por el historial (t_o <= último instante guardado).
        Descarta del historial lo que ya no se va a necesitar (las /odom llegan en orden).
        """
        h = self.historial
        while len(h) >= 2 and h[1][0] <= t_o + TOL_STAMP:
            h.popleft()
        t0, x0, y0, p0 = h[0]
        if abs(t_o - t0) <= TOL_STAMP:
            return x0, y0, p0, True
        if t_o < t0 or len(h) < 2:
            return None  # /odom más vieja que la primera pose propia
        t1, x1, y1, p1 = h[1]
        a = (t_o - t0) / (t1 - t0)
        return x0 + a * (x1 - x0), y0 + a * (y1 - y0), p0 + a * (p1 - p0), False

    def _procesar_odom_pendientes(self):
        while self.odom_pendientes and self.historial:
            t_o, xo, yo, psio = self.odom_pendientes[0]
            if t_o > self.historial[-1][0] + TOL_STAMP:
                break  # aún no llega el /joint_states de ese instante: esperar
            self.odom_pendientes.popleft()
            pose = self._pose_propia_en(t_o)
            if pose is None:
                continue
            x, y, psi, exacta = pose
            if exacta:
                self.n_exactas += 1
            else:
                self.n_interp += 1
            self._registrar(t_o, x, y, psi, xo, yo, psio)

    def _registrar(self, t, x, y, psi, xo, yo, psio):
        with self.lock:
            if self.t0 is None:
                self.t0 = t

            # Error = propia - odom
            e_x = x - xo
            e_y = y - yo
            e_psi = envolver(psi - psio)
            e_pos = math.hypot(e_x, e_y)
            self.n += 1
            self.suma_ep2 += e_pos ** 2
            self.suma_epsi2 += e_psi ** 2
            rmse = math.sqrt(self.suma_ep2 / self.n)

            if self.xy_odom_prev is not None:
                self.dist_odom += math.hypot(xo - self.xy_odom_prev[0], yo - self.xy_odom_prev[1])
            self.xy_odom_prev = (xo, yo)

            d = self.datos
            for k, val in (('t', t - self.t0), ('x_p', x), ('y_p', y),
                           ('psi_p', envolver(psi)), ('x_o', xo), ('y_o', yo),
                           ('psi_o', psio), ('e_x', e_x), ('e_y', e_y), ('e_psi', e_psi),
                           ('e_pos', e_pos), ('rmse_pos', rmse)):
                d[k].append(val)

    # ------------------------------------------------------------------ salida
    def snapshot(self):
        with self.lock:
            return {k: np.array(v) for k, v in self.datos.items()}, self.n

    def rmse_psi(self):
        return math.sqrt(self.suma_epsi2 / self.n) if self.n else 0.0

    def rmse_naive(self):
        if not self.n_naive:
            return 0.0, 0.0
        return (math.sqrt(self.suma_ep2_naive / self.n_naive),
                math.sqrt(self.suma_epsi2_naive / self.n_naive))

    def guardar_csv(self):
        d, n = self.snapshot()
        if n == 0:
            return
        claves = list(d.keys())
        with open(self.csv_path, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(claves)
            for i in range(len(d['t'])):
                wr.writerow([f'{d[k][i]:.6f}' for k in claves])
        print(f'\nDatos guardados en {self.csv_path}')

    def imprimir_resumen(self):
        d, n = self.snapshot()
        if n == 0:
            print('No se registraron datos para comparar.')
            return
        e_final = d['e_pos'][-1]
        pct = 100.0 * e_final / self.dist_odom if self.dist_odom > 1e-6 else float('nan')
        rmse_p_naive, rmse_psi_naive = self.rmse_naive()
        print('\n========== Resumen: odometría propia (Euler) vs /odom (DiffDrive) ==========')
        print(f'  Muestras comparadas      : {n}  ({self.n_exactas} con la misma marca de tiempo, '
              f'{self.n_interp} interpoladas)')
        print(f'  Duración                 : {d["t"][-1]:.2f} s')
        print(f'  Distancia recorrida      : {self.dist_odom:.3f} m (según /odom)')
        print('  --- Sincronizado por marca de tiempo ---')
        print(f'  Error final  e_x, e_y    : {d["e_x"][-1]:+.6f} m, {d["e_y"][-1]:+.6f} m')
        print(f'  Error final  ‖e_pos‖     : {e_final:.6f} m  ({pct:.4f} % de la distancia)')
        print(f'  Error final  e_psi       : {math.degrees(d["e_psi"][-1]):+.5f}°')
        print(f'  Error máximo ‖e_pos‖     : {d["e_pos"].max():.6f} m')
        print(f'  RMSE posición            : {d["rmse_pos"][-1]:.6f} m')
        print(f'  RMSE orientación         : {math.degrees(self.rmse_psi()):.5f}°')
        print('  --- Referencia: sin sincronizar (método anterior) ---')
        print(f'  RMSE posición            : {rmse_p_naive:.6f} m')
        print(f'  RMSE orientación         : {math.degrees(rmse_psi_naive):.5f}°')
        print('============================================================================')


# ----------------------------------------------------------------------------- main + gráfica
def main():
    rclpy.init()
    nodo = OdometriaLlantas()

    def girar():
        try:
            rclpy.spin(nodo)
        except (ExternalShutdownException, KeyboardInterrupt):
            pass

    threading.Thread(target=girar, daemon=True).start()

    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    fig.canvas.manager.set_window_title('Odometría propia vs /odom (sincronizada)')
    ax_xy, ax_exy, ax_epsi, ax_ep = axs.flat

    l_p, = ax_xy.plot([], [], '-', lw=2, label='Propia (Euler)')
    l_o, = ax_xy.plot([], [], '--', lw=2, color='black', label='/odom (DiffDrive)')
    ax_xy.set(title='Trayectoria', xlabel='x [m]', ylabel='y [m]')
    ax_xy.set_aspect('equal', adjustable='datalim')

    l_ex, = ax_exy.plot([], [], label='e_x')
    l_ey, = ax_exy.plot([], [], label='e_y')
    ax_exy.set(title='Error en x, y', xlabel='t [s]', ylabel='[m]')

    l_epsi, = ax_epsi.plot([], [], color='tab:red', label='e_ψ')
    ax_epsi.set(title='Error de orientación', xlabel='t [s]', ylabel='[°]')

    l_ep, = ax_ep.plot([], [], label='‖e_pos‖')
    l_rmse, = ax_ep.plot([], [], '--', color='black', label='RMSE pos')
    ax_ep.set(title='Norma del error de posición y RMSE', xlabel='t [s]', ylabel='[m]')

    for ax in axs.flat:
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper left', fontsize=8)
    fig.tight_layout(rect=(0, 0, 1, 0.93))

    def actualizar(_):
        if not rclpy.ok():
            plt.close(fig)
            return
        d, n = nodo.snapshot()
        if n == 0:
            fig.suptitle('Esperando /joint_states y /odom...')
            return
        l_p.set_data(d['x_p'], d['y_p'])
        l_o.set_data(d['x_o'], d['y_o'])
        l_ex.set_data(d['t'], d['e_x'])
        l_ey.set_data(d['t'], d['e_y'])
        l_epsi.set_data(d['t'], np.degrees(d['e_psi']))
        l_ep.set_data(d['t'], d['e_pos'])
        l_rmse.set_data(d['t'], d['rmse_pos'])
        for ax in axs.flat:
            ax.relim()
            ax.autoscale_view()
        rp_n, rpsi_n = nodo.rmse_naive()
        fig.suptitle(f'Sincronizado: ‖e_pos‖ = {d["e_pos"][-1] * 1000:.4f} mm    '
                     f'RMSE pos = {d["rmse_pos"][-1] * 1000:.4f} mm    '
                     f'RMSE ψ = {math.degrees(nodo.rmse_psi()):.4f}°\n'
                     f'(sin sincronizar: RMSE pos = {rp_n * 1000:.4f} mm, '
                     f'RMSE ψ = {math.degrees(rpsi_n):.4f}°)', fontsize=11)

    anim = FuncAnimation(fig, actualizar, interval=200, cache_frame_data=False)  # noqa: F841
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.guardar_csv()
        nodo.imprimir_resumen()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()