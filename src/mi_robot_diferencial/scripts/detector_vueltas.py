#!/usr/bin/python3
"""
detector_vueltas.py — Tiempos por vuelta y error de cierre de la odometría.

Requisitos del reporte que cubre:
  - 3.2.2  error de cierre en cada vuelta: pose estimada al cruzar la línea de salida/meta
           frente a la pose real.
  - 3.2.6  tiempo de cada vuelta y total, en tiempo de simulación (/clock).
  - 4      el tiempo corre desde que el robot cruza la línea de salida hasta que la cruza
           por tercera vez.

SOLO MIDE: ningún controlador usa /ground_truth (el modo autónomo debe ser reactivo).

Cómo detecta un cruce, con la pose REAL (/ground_truth, del motor de física de Gazebo):
  - La línea es un segmento: punto (linea_x, linea_y), dirección de avance linea_yaw y
    semiancho linea_semiancho. s = distancia con signo a la línea (positiva hacia adelante),
    l = distancia a lo largo de la línea.
  - Cruce = s pasa de < 0 a >= 0 con |l| < semiancho (descarta otros tramos de la pista que
    pasen por la misma recta) y han pasado al menos t_min_vuelta s desde el anterior.
  - El instante del cruce se interpola entre las dos muestras: t = t0 + (0 − s0)/(s1 − s0)·(t1 − t0).
  - Cruce 0 = salida (arranca el cronómetro). Cruce k = fin de la vuelta k.

En cada cruce compara, en ESE MISMO instante de simulación, la pose real con:
  - /odom_llantas : odometría propia (odometria_vueltas.py, ω_I y ω_D promediados)
  - /odom         : odometría del plugin DiffDrive, llevada al marco del mundo con la pose
                    real inicial (las dos asumen rodadura sin deslizamiento)

Al completar `vueltas` vueltas (y al cerrar con Ctrl+C) guarda en carpeta_salida:
  vueltas.csv, trayectoria.csv, trayectoria.png, error_vs_distancia.png, error_cierre.png

Publica /vuelta (std_msgs/Int32): vueltas completadas (0 al cruzar la salida).

Uso: lo lanza robot_yahboom.launch.py con la línea de cada pista. A mano:
  ros2 run mi_robot_diferencial detector_vueltas.py --ros-args -p use_sim_time:=true \\
      -p linea_x:=0.0 -p linea_y:=0.0 -p linea_yaw:=0.0 -p linea_semiancho:=0.6
"""
import bisect
import csv
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32


def yaw_de_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def envolver(a):
    return math.atan2(math.sin(a), math.cos(a))


def stamp_a_seg(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Historial:
    """Poses (t, x, y, psi) en orden de tiempo, con interpolación."""

    def __init__(self):
        self.t, self.x, self.y, self.psi = [], [], [], []

    def agregar(self, t, x, y, psi):
        if self.t and t <= self.t[-1]:
            return
        self.t.append(t); self.x.append(x); self.y.append(y); self.psi.append(psi)

    def ultimo_t(self):
        return self.t[-1] if self.t else -math.inf

    def en(self, t):
        """Pose interpolada en t (None si no hay datos o t está fuera del rango)."""
        if not self.t or t < self.t[0] or t > self.t[-1]:
            return None
        i = bisect.bisect_left(self.t, t)
        if self.t[i] == t or i == 0:
            return self.x[i], self.y[i], self.psi[i]
        t0, t1 = self.t[i - 1], self.t[i]
        f = (t - t0) / (t1 - t0)
        dpsi = envolver(self.psi[i] - self.psi[i - 1])
        return (self.x[i - 1] + f * (self.x[i] - self.x[i - 1]),
                self.y[i - 1] + f * (self.y[i] - self.y[i - 1]),
                envolver(self.psi[i - 1] + f * dpsi))


def error_pose(est, real):
    """(‖Δp‖, Δψ) de una pose estimada frente a la real."""
    if est is None:
        return math.nan, math.nan
    return math.hypot(est[0] - real[0], est[1] - real[1]), envolver(est[2] - real[2])


class DetectorVueltas:
    """Lógica pura (sin ROS)."""

    def __init__(self, linea_x, linea_y, linea_yaw, semiancho, t_min_vuelta):
        self.p0 = (linea_x, linea_y)
        self.d = (math.cos(linea_yaw), math.sin(linea_yaw))     # avance
        self.n = (-self.d[1], self.d[0])                          # a lo largo de la línea
        self.semiancho = semiancho
        self.t_min = t_min_vuelta
        self.real = Historial()
        self.est = {'llantas': Historial(), 'diffdrive': Historial()}
        self.dist = [0.0]                       # distancia real recorrida, alineada con self.real
        self.pose0 = None                       # pose real inicial (marco del /odom del DiffDrive)
        self.cruces = []                        # dicts ya resueltos
        self.pendientes = []                    # cruces esperando a que lleguen las estimaciones
        self._s_prev = None

    # ---------------------------------------------------------------- entradas
    def agregar_real(self, t, x, y, psi):
        if self.pose0 is None:
            self.pose0 = (x, y, psi)
        if self.real.t:
            self.dist.append(self.dist[-1] + math.hypot(x - self.real.x[-1], y - self.real.y[-1]))
            t_prev, x_prev, y_prev, psi_prev = (self.real.t[-1], self.real.x[-1],
                                                self.real.y[-1], self.real.psi[-1])
        self.real.agregar(t, x, y, psi)
        if len(self.dist) > len(self.real.t):   # muestra repetida descartada
            self.dist.pop()
            return None
        rx, ry = x - self.p0[0], y - self.p0[1]
        s = rx * self.d[0] + ry * self.d[1]
        lat = rx * self.n[0] + ry * self.n[1]
        nuevo = None
        if self._s_prev is not None and self._s_prev < 0.0 <= s and abs(lat) < self.semiancho:
            f = (0.0 - self._s_prev) / (s - self._s_prev)
            t_c = t_prev + f * (t - t_prev)
            ultimo = (self.pendientes[-1]['t'] if self.pendientes
                      else self.cruces[-1]['t'] if self.cruces else -math.inf)
            if t_c - ultimo >= self.t_min:
                real_c = (x_prev + f * (x - x_prev), y_prev + f * (y - y_prev),
                          envolver(psi_prev + f * envolver(psi - psi_prev)))
                nuevo = {'t': t_c, 'real': real_c,
                         'dist': self.dist[-2] + f * (self.dist[-1] - self.dist[-2])}
                self.pendientes.append(nuevo)
        self._s_prev = s
        return nuevo

    def agregar_estimada(self, fuente, t, x, y, psi):
        if fuente == 'diffdrive':              # /odom arranca en (0,0,0) en la pose inicial
            if self.pose0 is None:
                return
            x0, y0, p0 = self.pose0
            x, y, psi = (x0 + x * math.cos(p0) - y * math.sin(p0),
                         y0 + x * math.sin(p0) + y * math.cos(p0), envolver(p0 + psi))
        self.est[fuente].agregar(t, x, y, psi)

    # ---------------------------------------------------------------- resolución
    def resolver(self, forzar=False):
        """Completa los cruces pendientes cuyas estimaciones ya llegaron. Devuelve los resueltos."""
        listos = []
        while self.pendientes:
            c = self.pendientes[0]
            if not forzar and any(h.t and h.ultimo_t() < c['t'] for h in self.est.values()):
                break                           # aún falta una estimación de ese instante
            self.pendientes.pop(0)
            k = len(self.cruces)
            c['k'] = k
            c['t_total'] = c['t'] - self.cruces[0]['t'] if self.cruces else 0.0
            c['t_vuelta'] = c['t'] - self.cruces[-1]['t'] if self.cruces else 0.0
            for f, h in self.est.items():
                c[f] = h.en(c['t'])
                c['err_' + f] = error_pose(c[f], c['real'])
            self.cruces.append(c)
            listos.append(c)
        return listos

    def vueltas_completas(self):
        return max(len(self.cruces) - 1, 0)


# ====================================================================== salida
def guardar_resultados(det, carpeta, nombre_pista, log=print):
    os.makedirs(carpeta, exist_ok=True)
    fuentes = list(det.est)

    with open(os.path.join(carpeta, 'vueltas.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['cruce', 't_cruce_s', 't_vuelta_s', 't_total_s', 'distancia_m',
                    'x_real', 'y_real', 'psi_real']
                   + [c for fu in fuentes for c in (f'x_{fu}', f'y_{fu}', f'psi_{fu}',
                                                    f'err_pos_{fu}_m', f'err_psi_{fu}_rad')])
        for c in det.cruces:
            fila = [c['k'], f"{c['t']:.4f}", f"{c['t_vuelta']:.4f}", f"{c['t_total']:.4f}",
                    f"{c['dist']:.3f}", *[f'{v:.6f}' for v in c['real']]]
            for fu in fuentes:
                p = c[fu] or (math.nan,) * 3
                fila += [f'{v:.6f}' for v in p] + [f"{c['err_' + fu][0]:.6f}", f"{c['err_' + fu][1]:.6f}"]
            w.writerow(fila)

    # trayectoria a ~20 Hz, con el error continuo de cada estimación
    R = det.real
    filas, paso, t_ult = [], 0.05, -math.inf
    for i, t in enumerate(R.t):
        if t - t_ult < paso:
            continue
        t_ult = t
        real = (R.x[i], R.y[i], R.psi[i])
        fila = [t, det.dist[i], *real]
        for fu in fuentes:
            e = det.est[fu].en(t)
            fila += list(e or (math.nan,) * 3) + [error_pose(e, real)[0]]
        filas.append(fila)
    with open(os.path.join(carpeta, 'trayectoria.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'distancia_m', 'x_real', 'y_real', 'psi_real']
                   + [c for fu in fuentes for c in (f'x_{fu}', f'y_{fu}', f'psi_{fu}', f'err_pos_{fu}_m')])
        w.writerows([[f'{v:.6f}' for v in fila] for fila in filas])

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        log('matplotlib no disponible: solo se guardaron los CSV')
        return
    col = {'llantas': 'tab:red', 'diffdrive': 'tab:blue'}
    nom = {'llantas': 'Odometría propia (ω_I, ω_D)', 'diffdrive': 'Odometría DiffDrive (/odom)'}
    n_v = det.vueltas_completas()

    fig, ax = plt.subplots(figsize=(9, 7))
    ax.plot(R.x, R.y, 'k-', lw=2, label='Real (/ground_truth)')
    for fu in fuentes:
        h = det.est[fu]
        if h.t:
            ax.plot(h.x, h.y, '-' if fu == 'llantas' else '--', color=col[fu],
                    lw=3.0 if fu == 'llantas' else 1.2, alpha=0.7 if fu == 'llantas' else 1.0, label=nom[fu])
    L = det.semiancho
    ax.plot([det.p0[0] - L * det.n[0], det.p0[0] + L * det.n[0]],
            [det.p0[1] - L * det.n[1], det.p0[1] + L * det.n[1]], 'g-', lw=4, label='Línea de salida/meta')
    for c in det.cruces:
        ax.plot(*c['real'][:2], 'ko', ms=5)
        for fu in fuentes:
            if c[fu]:
                ax.plot(*c[fu][:2], 'x', color=col[fu], ms=8)
    ax.set_aspect('equal'); ax.grid(True); ax.legend(loc='best', fontsize=8)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title(f'{nombre_pista}: trayectoria de {n_v} vuelta(s)')
    fig.tight_layout(); fig.savefig(os.path.join(carpeta, 'trayectoria.png'), dpi=130); plt.close(fig)

    if filas:
        fig, ax = plt.subplots(figsize=(10, 4.5))
        d = [f[1] for f in filas]
        for j, fu in enumerate(fuentes):
            ax.plot(d, [f[5 + 4 * j + 3] * 1000 for f in filas], color=col[fu], label=nom[fu],
                    lw=3.0 if fu == 'llantas' else 1.2, alpha=0.7 if fu == 'llantas' else 1.0)
        for c in det.cruces:
            ax.axvline(c['dist'], color='g', ls=':', lw=1)
            ax.text(c['dist'], ax.get_ylim()[1] * 0.95, 'salida' if c['k'] == 0 else f"V{c['k']}",
                    color='g', ha='center', fontsize=8)
        ax.set_xlabel('Distancia real recorrida [m]'); ax.set_ylabel('‖error de posición‖ [mm]')
        ax.set_title('Crecimiento del error de odometría'); ax.grid(True); ax.legend(fontsize=8)
        fig.tight_layout(); fig.savefig(os.path.join(carpeta, 'error_vs_distancia.png'), dpi=130)
        plt.close(fig)

    vueltas = [c for c in det.cruces if c['k'] > 0]
    if vueltas:
        fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 4))
        k = [c['k'] for c in vueltas]
        for j, fu in enumerate(fuentes):
            off = (j - (len(fuentes) - 1) / 2) * 0.35
            a1.bar([x + off for x in k], [c['err_' + fu][0] * 1000 for c in vueltas], 0.35,
                   color=col[fu], label=nom[fu])
            a2.bar([x + off for x in k], [math.degrees(c['err_' + fu][1]) for c in vueltas], 0.35,
                   color=col[fu])
        a1.set_title('Error de cierre de posición'); a1.set_ylabel('[mm]')
        a2.set_title('Error de cierre de orientación'); a2.set_ylabel('[°]')
        for a in (a1, a2):
            a.set_xlabel('Vuelta'); a.set_xticks(k); a.grid(True, axis='y')
        a1.legend(fontsize=8)
        fig.tight_layout(); fig.savefig(os.path.join(carpeta, 'error_cierre.png'), dpi=130); plt.close(fig)


def tabla_resumen(det):
    lineas = ['', '  Cruce | t vuelta [s] | t total [s] | dist [m] | e_pos propia [mm] | e_ψ propia [°]'
              ' | e_pos DiffDrive [mm] | e_ψ DiffDrive [°]',
              ' ' + '-' * 118]
    for c in det.cruces:
        el, ed = c['err_llantas'], c['err_diffdrive']
        lineas.append(' %6s | %12s | %11.3f | %8.2f | %17.2f | %14.3f | %20.2f | %17.3f' % (
            'salida' if c['k'] == 0 else f"V{c['k']}",
            '-' if c['k'] == 0 else f"{c['t_vuelta']:.3f}", c['t_total'], c['dist'],
            el[0] * 1000, math.degrees(el[1]), ed[0] * 1000, math.degrees(ed[1])))
    return '\n'.join(lineas)


# ====================================================================== nodo
class NodoDetectorVueltas(Node):
    def __init__(self):
        super().__init__('detector_vueltas')
        self.declare_parameter('linea_x', 0.0)
        self.declare_parameter('linea_y', 0.0)
        self.declare_parameter('linea_yaw', 0.0)        # dirección de avance al cruzar [rad]
        self.declare_parameter('linea_semiancho', 0.6)  # [m]
        self.declare_parameter('vueltas', 3)
        self.declare_parameter('t_min_vuelta', 10.0)    # [s] contra cruces dobles
        self.declare_parameter('nombre_pista', 'pista')
        self.declare_parameter('carpeta_salida', '~/resultados_vueltas')
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        self.det = DetectorVueltas(gp('linea_x'), gp('linea_y'), gp('linea_yaw'),
                                   gp('linea_semiancho'), gp('t_min_vuelta'))
        self.n_vueltas = gp('vueltas')
        self.pista = gp('nombre_pista')
        self.carpeta = os.path.join(os.path.expanduser(gp('carpeta_salida')),
                                    f"{self.pista}_{time.strftime('%Y%m%d_%H%M%S')}")
        self.guardado_en = -1

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, '/ground_truth', self.cb_real, qos)
        self.create_subscription(Odometry, '/odom_llantas', lambda m: self.cb_est('llantas', m), qos)
        self.create_subscription(Odometry, '/odom', lambda m: self.cb_est('diffdrive', m), qos)
        self.pub = self.create_publisher(Int32, '/vuelta', 10)
        self.create_timer(0.2, self.cb_timer)
        self.get_logger().info(
            f"Línea de salida/meta en ({gp('linea_x'):.2f}, {gp('linea_y'):.2f}), avance "
            f"{math.degrees(gp('linea_yaw')):.0f}°, ±{gp('linea_semiancho'):.2f} m. "
            f"Resultados en {self.carpeta}")

    @staticmethod
    def _pose(msg):
        p = msg.pose.pose
        return stamp_a_seg(msg.header.stamp), p.position.x, p.position.y, yaw_de_quat(p.orientation)

    def cb_real(self, msg):
        self.det.agregar_real(*self._pose(msg))

    def cb_est(self, fuente, msg):
        self.det.agregar_estimada(fuente, *self._pose(msg))

    def cb_timer(self, forzar=False):
        # si una estimación no llega en 1 s de simulación, se resuelve con lo que haya
        t_ahora = self.det.real.ultimo_t()
        forzar = forzar or (self.det.pendientes and t_ahora - self.det.pendientes[0]['t'] > 1.0)
        for c in self.det.resolver(forzar=forzar):
            el = c['err_llantas']
            if c['k'] == 0:
                self.get_logger().info(f"SALIDA en t = {c['t']:.3f} s (sim). Cronómetro en marcha.")
            else:
                self.get_logger().info(
                    f"VUELTA {c['k']}: {c['t_vuelta']:.3f} s | total {c['t_total']:.3f} s | "
                    f"error de cierre (propia) {el[0] * 1000:.1f} mm, {math.degrees(el[1]):.2f}°")
            self.pub.publish(Int32(data=c['k']))
            if c['k'] == self.n_vueltas:
                self.guardar(f'{self.n_vueltas} vueltas completas')

    def guardar(self, motivo):
        if not self.det.real.t or self.guardado_en == len(self.det.cruces):
            return
        guardar_resultados(self.det, self.carpeta, self.pista, self.get_logger().info)
        self.guardado_en = len(self.det.cruces)
        self.get_logger().info(f'{motivo}: resultados guardados en {self.carpeta}'
                               + tabla_resumen(self.det))


def main():
    rclpy.init()
    nodo = NodoDetectorVueltas()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.det.resolver(forzar=True)
        nodo.guardar('Cierre del nodo')
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
