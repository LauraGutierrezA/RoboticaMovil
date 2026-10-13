#!/usr/bin/env python3
"""
Genera worlds/pista_pro.sdf: pista en forma de "8" con PUENTE en el cruce.

Geometría:
  - Dos lóbulos circulares (radio de la línea central R) con centros en (-D, 0) y (+D, 0).
  - Dos rectas que se cruzan en el origen a +45° (recta A) y -45° (recta B).
  - Con D = R*sqrt(2) las rectas quedan tangentes a los dos círculos y cada lóbulo es un arco de 270°.
  - Los muros curvos se aproximan con cajas cortas (cuerdas del arco).

Puente:
  - La recta A sube por una rampa, cruza por encima de la recta B sobre un tablero plano
    y baja por otra rampa. Rampas y tablero llevan barandas a los dos lados.
  - La recta B pasa por debajo, a nivel del piso. Sus muros siguen de largo bajo el puente,
    pero más bajitos (MURO_BAJO_H) para que quepan debajo del tablero.

Requisitos del reporte que cubre:
  - Curvas (lóbulos de 270°), estrechamiento (lóbulo izquierdo), obstáculos (slalom en el
    lóbulo derecho + el puente), muros con colisión y punto de spawn definido.

Uso:  python3 scripts/generar_pista_pro.py > worlds/pista_pro.sdf
"""
import math

# ---------------- PARÁMETROS DE LA PISTA (ajústalos a tu robot) ----------------
R = 2.5            # radio de la línea central de cada lóbulo [m]
W = 1.2            # ancho del carril entre ejes de muro [m]
W_NARROW = 0.35    # ancho entre ejes de muro en el estrechamiento [m]
T = 0.10           # espesor de los muros [m]
H = 0.50           # altura de los muros [m]
SEG_DEG = 10       # cada cuántos grados se pone una caja en las curvas
NARROW_FROM, NARROW_TO = 160, 200   # sector (grados, lóbulo izq.) del estrechamiento
OBST_R = 0.15      # radio de los cilindros obstáculo [m]
OBSTACULOS = [(60, +0.30), (0, -0.30), (-60, +0.30)]  # (ángulo en lóbulo der., desplazamiento radial) para slalom

# ---------------- PARÁMETROS DEL PUENTE ----------------
# Tu robot (guía): ruedas r=0.05 → base_link a ~0.07 m del piso; el LiDAR llega a ~0.17 m de alto.
PUENTE_H = 0.30        # altura de la superficie del tablero sobre el piso [m]
TABLERO_E = 0.04       # espesor del tablero y de las rampas [m]
TABLERO_MEDIO = 0.70   # medio largo del tablero, medido sobre la recta A desde el cruce [m]
BARANDA_H = 0.20       # altura de las barandas sobre la superficie que pisan [m]
BARANDA_E = 0.05       # espesor de las barandas [m]
MURO_BAJO_H = 0.22     # altura de los muros de la recta B debajo del puente [m]
# Espacio libre bajo el tablero = PUENTE_H - TABLERO_E = 0.26 m (> 0.17 m del robot)

D = R * math.sqrt(2)     # distancia de cada centro de lóbulo al origen
C_IZQ = (-D, 0.0)
C_DER = (+D, 0.0)

MADERA = "0.87 0.72 0.53 1"
MADERA_OSCURA = "0.65 0.45 0.28 1"
TRONCO = "0.40 0.26 0.13 1"
COLORES_OBST = ["0.85 0.15 0.15 1", "0.95 0.80 0.10 1", "0.20 0.40 0.85 1"]
VERDES = ["0.10 0.45 0.15 1", "0.15 0.55 0.20 1", "0.25 0.65 0.25 1"]

modelos = []


def caja(nombre, x, y, yaw, largo, ancho=T, alto=H, color=MADERA, z=None, pitch=0.0):
    """Caja estática. Si no se da z, la apoya en el piso (z = alto/2)."""
    if z is None:
        z = alto / 2
    modelos.append(f"""
    <model name="{nombre}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 {pitch:.4f} {yaw:.4f}</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>{largo:.4f} {ancho:.4f} {alto:.4f}</size></box></geometry></collision>
        <visual name="vis">
          <geometry><box><size>{largo:.4f} {ancho:.4f} {alto:.4f}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>""")


def cilindro(nombre, x, y, radio, alto, color):
    modelos.append(f"""
    <model name="{nombre}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {alto/2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><cylinder><radius>{radio}</radius><length>{alto}</length></cylinder></geometry></collision>
        <visual name="vis">
          <geometry><cylinder><radius>{radio}</radius><length>{alto}</length></cylinder></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>""")


def arbol(nombre, x, y, escala=1.0):
    """Árbol: tronco (cilindro) + 3 conos verdes superpuestos.
    Cada <visual> tiene su propia <pose> relativa a la base del árbol.
    La colisión es solo un cilindro (más simple y estable para la física que 3 conos)."""
    s = escala
    tronco_h, tronco_r = 0.15 * s, 0.04 * s
    # (radio de la base del cono, altura del cono, z del centro del cono)
    conos = [(0.25 * s, 0.30 * s, 0.27 * s),
             (0.20 * s, 0.28 * s, 0.42 * s),
             (0.14 * s, 0.24 * s, 0.56 * s)]
    alto_total = conos[-1][2] + conos[-1][1] / 2
    visuales = [f"""
        <visual name="tronco">
          <pose>0 0 {tronco_h/2:.3f} 0 0 0</pose>
          <geometry><cylinder><radius>{tronco_r:.3f}</radius><length>{tronco_h:.3f}</length></cylinder></geometry>
          <material><ambient>{TRONCO}</ambient><diffuse>{TRONCO}</diffuse></material>
        </visual>"""]
    for k, (rc, hc, zc) in enumerate(conos):
        visuales.append(f"""
        <visual name="cono_{k}">
          <pose>0 0 {zc:.3f} 0 0 0</pose>
          <geometry><cone><radius>{rc:.3f}</radius><length>{hc:.3f}</length></cone></geometry>
          <material><ambient>{VERDES[k]}</ambient><diffuse>{VERDES[k]}</diffuse></material>
        </visual>""")
    modelos.append(f"""
    <model name="{nombre}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 0 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <pose>0 0 {alto_total/2:.3f} 0 0 0</pose>
          <geometry><cylinder><radius>{conos[0][0]:.3f}</radius><length>{alto_total:.3f}</length></cylinder></geometry>
        </collision>{''.join(visuales)}
      </link>
    </model>""")


def arco(prefijo, centro, r, a0_deg, a1_deg):
    """Muro curvo de radio r entre los ángulos a0 y a1 (grados), hecho de cajas-cuerda."""
    n = max(1, round(abs(a1_deg - a0_deg) / SEG_DEG))
    da = math.radians(a1_deg - a0_deg) / n
    for i in range(n):
        phi = math.radians(a0_deg) + (i + 0.5) * da
        rc = r * math.cos(abs(da) / 2)                  # punto medio de la cuerda
        largo = 2 * r * math.sin(abs(da) / 2) + T       # cuerda + T para que no queden huecos
        x = centro[0] + rc * math.cos(phi)
        y = centro[1] + rc * math.sin(phi)
        caja(f"{prefijo}_{i:02d}", x, y, phi + math.pi / 2, largo)


def radial(nombre, centro, ang_deg, r1, r2):
    """Muro corto en dirección radial (tapa los escalones del estrechamiento)."""
    a = math.radians(ang_deg)
    rm = (r1 + r2) / 2
    caja(nombre, centro[0] + rm * math.cos(a), centro[1] + rm * math.sin(a), a, abs(r2 - r1) + T)


def punto_recta(ang_deg, s, off):
    """Punto a distancia s sobre la recta (ángulo ang_deg, pasa por el origen) y desplazado off a su izquierda."""
    a = math.radians(ang_deg)
    u = (math.cos(a), math.sin(a))
    n = (-u[1], u[0])
    return (s * u[0] + off * n[0], s * u[1] + off * n[1])


# ---------------- LÓBULO DERECHO: arco de -135° a 135° (pasa por 0°) ----------------
arco("der_ext", C_DER, R + W / 2, -135, 135)
arco("der_int", C_DER, R - W / 2, -135, 135)

# ---------------- LÓBULO IZQUIERDO: arco de 45° a 315° con estrechamiento ----------------
r_ext, r_int = R + W / 2, R - W / 2
r_ext_n, r_int_n = R + W_NARROW / 2, R - W_NARROW / 2
arco("izq_ext_a", C_IZQ, r_ext, 45, NARROW_FROM)
arco("izq_ext_n", C_IZQ, r_ext_n, NARROW_FROM, NARROW_TO)
arco("izq_ext_b", C_IZQ, r_ext, NARROW_TO, 315)
arco("izq_int_a", C_IZQ, r_int, 45, NARROW_FROM)
arco("izq_int_n", C_IZQ, r_int_n, NARROW_FROM, NARROW_TO)
arco("izq_int_b", C_IZQ, r_int, NARROW_TO, 315)
for k, ang in enumerate((NARROW_FROM, NARROW_TO)):
    radial(f"estrech_ext_{k}", C_IZQ, ang, r_ext_n, r_ext)
    radial(f"estrech_int_{k}", C_IZQ, ang, r_int, r_int_n)

# ---------------- RECTA B (-45°): a nivel del piso, pasa POR DEBAJO del puente ----------------
# Debajo del puente (|s| < W/2 + margen) los muros son bajitos para caber bajo el tablero.
ANG_B = -45
s_bajo = W / 2 + BARANDA_E          # hasta dónde llega por los lados el ancho del puente
for lado, off in (("L", +W / 2), ("R", -W / 2)):
    # tramo bajo, centrado en el cruce
    x, y = punto_recta(ANG_B, 0.0, off)
    caja(f"rectaB_{lado}_bajo", x, y, math.radians(ANG_B), 2 * s_bajo, alto=MURO_BAJO_H, color=MADERA_OSCURA)
    # tramos altos, desde el borde del puente hasta la tangencia con cada lóbulo
    for tramo, (s0, s1) in (("p", (s_bajo, R)), ("n", (-R, -s_bajo))):
        x, y = punto_recta(ANG_B, (s0 + s1) / 2, off)
        caja(f"rectaB_{lado}{tramo}", x, y, math.radians(ANG_B), (s1 - s0) + T / 2)

# ---------------- RECTA A (+45°): PUENTE (rampa - tablero - rampa) con barandas ----------------
ANG_A = 45
yaw_A = math.radians(ANG_A)
ancho_puente = W + BARANDA_E          # tablero un poco más ancho que el carril, barandas encima del borde

# Tablero plano centrado en el cruce
caja("puente_tablero", 0.0, 0.0, yaw_A, 2 * TABLERO_MEDIO, ancho=ancho_puente, alto=TABLERO_E,
     color=MADERA, z=PUENTE_H - TABLERO_E / 2)
for lado, off in (("L", +W / 2), ("R", -W / 2)):
    x, y = punto_recta(ANG_A, 0.0, off)
    caja(f"puente_baranda_{lado}", x, y, yaw_A, 2 * TABLERO_MEDIO, ancho=BARANDA_E, alto=BARANDA_H,
         color=MADERA_OSCURA, z=PUENTE_H + BARANDA_H / 2)

# Rampas: van desde la tangencia con el lóbulo (s = ±R, en el piso) hasta el borde del tablero (s = ±TABLERO_MEDIO)
largo_h = R - TABLERO_MEDIO                       # largo horizontal de cada rampa
alfa = math.atan2(PUENTE_H, largo_h)              # ángulo de la pendiente
largo_inc = math.hypot(largo_h, PUENTE_H)         # largo inclinado (sobre la rampa)
for nombre, signo in (("rampa_p", +1), ("rampa_n", -1)):
    # yaw de la rampa: su eje +x apunta "cuesta arriba", hacia el cruce
    yaw = yaw_A if signo < 0 else yaw_A + math.pi
    s_medio = signo * (TABLERO_MEDIO + R) / 2     # punto medio horizontal de la rampa
    z_sup = PUENTE_H / 2                          # altura de la superficie en el punto medio
    # El centro de la caja queda medio espesor por debajo de la superficie, perpendicular a ella
    d_h = (TABLERO_E / 2) * math.sin(alfa)        # corrimiento horizontal (hacia la parte alta)
    d_z = (TABLERO_E / 2) * math.cos(alfa)        # corrimiento vertical (hacia abajo)
    xs, ys = punto_recta(ANG_A, s_medio - signo * d_h, 0.0)
    caja(nombre, xs, ys, yaw, largo_inc, ancho=ancho_puente, alto=TABLERO_E,
         color=MADERA, z=z_sup - d_z, pitch=-alfa)  # pitch negativo levanta el extremo +x
    # Barandas de la rampa: misma inclinación, encima de la superficie
    b_h = (BARANDA_H / 2) * math.sin(alfa)
    b_z = (BARANDA_H / 2) * math.cos(alfa)
    for lado, off in (("L", +W / 2), ("R", -W / 2)):
        xb, yb = punto_recta(ANG_A, s_medio + signo * b_h, off)
        caja(f"{nombre}_baranda_{lado}", xb, yb, yaw, largo_inc, ancho=BARANDA_E, alto=BARANDA_H,
             color=MADERA_OSCURA, z=z_sup + b_z, pitch=-alfa)

# ---------------- OBSTÁCULOS (slalom en el lóbulo derecho) ----------------
for k, (ang, desp) in enumerate(OBSTACULOS):
    a = math.radians(ang)
    r = R + desp
    cilindro(f"obstaculo_{k}", C_DER[0] + r * math.cos(a), C_DER[1] + r * math.sin(a),
             OBST_R, 0.4, COLORES_OBST[k % len(COLORES_OBST)])

# ---------------- DECORACIÓN (dentro de los lóbulos: el robot no llega, la cámara la ve) ----------------
caja("casa_izq", C_IZQ[0], C_IZQ[1], 0.3, 0.6, 0.4, 0.5, "0.30 0.50 0.85 1")
caja("casa_der", C_DER[0] - 0.3, C_DER[1] + 0.3, -0.2, 0.4, 0.4, 0.4, "0.95 0.85 0.20 1")
for k, (x, y, esc) in enumerate([(C_IZQ[0] + 0.8, 0.6, 1.0), (C_IZQ[0] - 0.7, -0.7, 1.2),
                                 (C_DER[0] + 0.8, -0.5, 1.1), (C_DER[0] + 0.3, 0.9, 0.9)]):
    arbol(f"arbol_{k}", x, y, esc)

# ---------------- SPAWN (en la recta B, a nivel del piso, mirando hacia el cruce) ----------------
SPAWN_S = -1.5
sx, sy = punto_recta(ANG_B, SPAWN_S, 0.0)
s_yaw = math.radians(ANG_B)

# ---------------- SDF COMPLETO ----------------
print(f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="pista_pro">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>

    <light type="directional" name="sol">
      <pose>0 0 10 0 0 0</pose>
      <direction>0.3 0.3 -1</direction>
    </light>

    <model name="piso">
      <static>true</static>
      <link name="link">
        <collision name="col">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="vis">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- ===== Pista en 8 con puente: generada con generar_pista_pro.py =====
         R={R} m, carril={W} m, estrechamiento={W_NARROW} m
         Puente: altura {PUENTE_H} m, pendiente {math.degrees(alfa):.1f} grados, libre debajo {PUENTE_H - TABLERO_E:.2f} m
         Spawn: x={sx:.2f} y={sy:.2f} yaw={s_yaw:.3f} (recta B, mirando hacia el cruce) -->
{''.join(modelos)}
  </world>
</sdf>""")