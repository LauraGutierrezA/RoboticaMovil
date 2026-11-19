# Robótica Móvil — Robot de 4 llantas en Gazebo con LiDAR y cámara

Paquete ROS 2 (`mi_robot_diferencial`) con un robot móvil de tracción diferencial de 4 llantas (skid-steer), simulado en Gazebo, con LiDAR y cámara, una pista propia en forma de "8" con puente, y un nodo propio de cinemática de las llantas que compara su odometría con la del plugin DiffDrive.

Universidad EIA — Ingeniería Mecatrónica — Robótica Móvil.

> La explicación del modelo, la cinemática de las llantas, los parámetros y las decisiones de diseño están en el Notion del proyecto.

---

## Contenido del repositorio

```
RoboticaMovil/
├── README.md
├── odometria_comparacion.csv          # Resultado de una corrida del nodo de odometría
├── frames_2026-10-12_17.30.22.pdf     # Árbol de TF (view_frames)
└── src/mi_robot_diferencial/
    ├── CMakeLists.txt
    ├── package.xml
    ├── urdf/
    │   ├── robotPro.urdf.xacro        # Robot final: 4 llantas, LiDAR, cámara
    │   └── mi_robot.urdf.xacro        # Robot inicial de la guía (2 llantas)
    ├── worlds/
    │   ├── pista_pro.sdf              # Pista final: "8" con puente, estrechamiento y slalom
    │   └── pista.sdf                  # Pista inicial
    ├── launch/
    │   ├── robot_pro.launch.py        # Lanza todo con el robot y la pista finales
    │   └── mi_robot.launch.py         # Lanza el robot inicial
    ├── config/
    │   └── bridge.yaml                # Tópicos puenteados entre Gazebo y ROS 2
    └── scripts/
        ├── odometria_llantas.py       # Nodo propio de cinemática de las llantas
        └── generar_pista_pro.py       # Genera worlds/pista_pro.sdf
```

---

## Dependencias

Probado en **Ubuntu 24.04** con **ROS 2 Jazzy** y **Gazebo Harmonic** (`gz sim`).

```bash
sudo apt update
sudo apt install \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-teleop-twist-keyboard \
  python3-colcon-common-extensions \
  python3-numpy python3-matplotlib python3-tk
```

| Dependencia | Para qué se usa |
|---|---|
| `ros-jazzy-ros-gz` | Gazebo Sim, `ros_gz_sim` (spawn) y `ros_gz_bridge` (puente de tópicos) |
| `ros-jazzy-xacro` | Convertir `robotPro.urdf.xacro` en URDF |
| `ros-jazzy-robot-state-publisher` | Publicar el TF del robot a partir del URDF |
| `ros-jazzy-teleop-twist-keyboard` | Manejar el robot con el teclado |
| `rviz2` (incluido en `ros-jazzy-desktop`) | Visualizar LiDAR, cámara y TF |
| `python3-numpy`, `python3-matplotlib`, `python3-tk` | Cálculos y gráficas del nodo de odometría |

---

## Compilación

```bash
mkdir -p ~/ros2_ws_dif
cd ~/ros2_ws_dif
git clone https://github.com/LauraGutierrezA/RoboticaMovil.git .
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mi_robot_diferencial
source install/setup.bash
```

---

## Ejecución

### 1. Simulación completa (un solo launch)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws_dif/install/setup.bash
ros2 launch mi_robot_diferencial robot_pro.launch.py
```

Este launch levanta, desde una sola terminal:
- **Gazebo** con `pista_pro.sdf`, sin pausa.
- **`robot_state_publisher`** con el URDF generado por xacro.
- **Spawn** del robot en la recta B de la pista: (x, y, z) = (−1.06, 1.06, 0.1) m, yaw = −45°.
- **`ros_gz_bridge`** con `config/bridge.yaml`.
- **RViz2**.

### 2. Teleoperación

En otra terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard                                   # con robot_pro.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop  # con robot_yahboom.launch.py (pasa por el AEB)
```

### 3. Nodo de odometría por cinemática de las llantas

En otra terminal, con la simulación corriendo y antes de mover el robot:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws_dif/install/setup.bash
ros2 run mi_robot_diferencial odometria_llantas.py --ros-args -p use_sim_time:=true
```

- **Mientras corre**, se abre una ventana con las gráficas en vivo.
- **Al cerrar la ventana**, se guarda `odometria_comparacion.csv` en la carpeta desde donde se lanzó el nodo y se imprime el resumen de errores en la terminal.

---

## Modelo del robot (`robotPro.urdf.xacro`)

| Parámetro | Valor | Propiedad xacro |
|---|---|---|
| Radio de rueda | 0.024 m | `rueda_radio` |
| Ancho de rueda | 0.0215 m | `rueda_ancho` |
| Separación entre ruedas (izquierda–derecha) | 0.1315 m | `separacion_ruedas` |
| Distancia entre ejes (delantero–trasero) | 0.095 m | `distancia_ejes` |
| Masa total | 0.906 kg | `masa_total` (ver *Adaptación al MicroROS*) |

**Estructura:**
- **Chasis:** dos cajas (inferior y superior) con visual, colisión e inercia.
- **Llantas:** 4 llantas generadas con el macro `rueda`: `izq_del`, `izq_tra`, `der_del`, `der_tra`. Joints `continuous` con eje de giro lateral.
- **Tracción:** plugin `gz-sim-diff-drive-system` con dos joints por lado (configuración skid-steer). Publica `/odom` a 30 Hz con `frame_id = odom` y `child_frame_id = base_footprint`, y escucha `/cmd_vel`.
- **Estados de las llantas:** plugin `gz-sim-joint-state-publisher-system`, que publica `/joint_states`.

**Sensores:**

| Sensor | Configuración |
|---|---|
| LiDAR (`gpu_lidar`, frame `lidar_link`) | 360° (450 muestras), rango 0.03–12 m, 10 Hz, ruido gaussiano σ = 0.01 m |
| Cámara (frame `camera_link`) | 640×480, 15 FPS, FOV horizontal 60°, ruido gaussiano σ = 0.007 |

**Árbol de TF:**

```
odom
└── base_footprint
    └── base_link
        ├── rueda_izq_del, rueda_izq_tra, rueda_der_del, rueda_der_tra
        ├── lidar_link
        ├── camera_link
        └── antena_link
```

---

## Tópicos puenteados (`config/bridge.yaml`)

| Tópico ROS 2 | Tipo | Dirección |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gazebo |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo → ROS |
| `/image_raw` | `sensor_msgs/msg/Image` | Gazebo → ROS |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo → ROS |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo → ROS |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → ROS |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS |
| `/ground_truth` | `nav_msgs/msg/Odometry` | Gazebo → ROS (pose real, solo para medir) |

---

## Pista (`pista_pro.sdf`)

Pista en forma de "8" generada con `scripts/generar_pista_pro.py`:

- **Curvas:** dos lóbulos circulares de 270° con radio de línea central de 2.5 m, unidos por dos rectas que se cruzan a ±45°. El carril mide 1.2 m de ancho.
- **Estrechamiento:** en el lóbulo izquierdo, entre 160° y 200°.
- **Obstáculos:** un slalom de 3 cilindros en el lóbulo derecho.
- **Puente:** en el cruce, la recta A sube por una rampa, pasa sobre la recta B a 0.30 m de altura y baja por otra rampa. Tiene barandas a los dos lados. La recta B pasa por debajo con 0.26 m de espacio libre.
- **Muros:** todos los muros, rampas y barandas tienen colisión.

Para modificar la pista, se cambian los parámetros al inicio del script y se regenera el mundo:

```bash
cd ~/ros2_ws_dif/src/mi_robot_diferencial
python3 scripts/generar_pista_pro.py > worlds/pista_pro.sdf
```

Después de regenerar, hay que volver a compilar.

---

## Cinemática de las llantas (`odometria_llantas.py`)

### Modelo

Cada llanta relaciona su velocidad angular con la velocidad del cuerpo ζ = [u, v, r]ᵀ:

$$w_i = \begin{bmatrix}\tfrac{1}{a_i} & \tfrac{\tan\phi_i}{a_i}\end{bmatrix} R(\theta_{Bi})^T \begin{bmatrix}1 & 0 & -d_{y,i}\\ 0 & 1 & d_{x,i}\end{bmatrix} \zeta$$

Con φ = θ = 0 (llantas fijas, paralelas y sin rodillos), la columna de v es cero y se elimina. Queda w = Q·[u, r]ᵀ, con Q de 4×2:

$$Q = \begin{bmatrix}41.667 & -2.73958\\ 41.667 & -2.73958\\ 41.667 & 2.73958\\ 41.667 & 2.73958\end{bmatrix}$$

El orden de las filas es llanta 1 = `izq_tra`, 2 = `izq_del`, 3 = `der_tra`, 4 = `der_del`.

**Cinemática directa del cuerpo.** Como hay 4 llantas y solo 2 velocidades, se usa la pseudoinversa (mínimos cuadrados):

$$\zeta = \begin{bmatrix}u\\ r\end{bmatrix} = W w,\qquad W = (Q^TQ)^{-1}Q^T = \begin{bmatrix}0.006 & 0.006 & 0.006 & 0.006\\ -0.091255 & -0.091255 & 0.091255 & 0.091255\end{bmatrix}$$

**Marco global.** Como v = 0, se elimina la segunda columna de J(ψ):

$$\dot\eta = \begin{bmatrix}\dot x\\ \dot y\\ \dot\psi\end{bmatrix} = \begin{bmatrix}\cos\psi & 0\\ \sin\psi & 0\\ 0 & 1\end{bmatrix}\begin{bmatrix}u\\ r\end{bmatrix}$$

**Integración de Euler**, con Δt igual al tiempo entre mensajes de `/joint_states`:

$$\eta_{k+1} = \eta_k + \dot\eta_k\,\Delta t$$

### Qué hace el nodo

1. Lee los parámetros de cada llanta (a, θ, dx, dy) del URDF publicado en `/robot_description`. φ se toma de un parámetro.
2. Construye Q y W, y los imprime al arrancar para compararlos con el Notion.
3. Se suscribe a `/joint_states`, calcula ζ = W·w y η̇, e integra la pose (x, y, ψ) desde (0, 0, 0).
4. Se suscribe a `/odom`, la odometría del plugin DiffDrive.
5. Compara cada mensaje de `/odom` con la pose propia **del mismo instante de simulación** (sincronización por marca de tiempo).
6. Grafica en vivo la trayectoria, e_x, e_y, e_ψ, ‖e_pos‖ y su RMSE. Al cerrar, guarda el CSV y el resumen.

### Parámetros del nodo

| Parámetro | Valor por defecto | Descripción |
|---|---|---|
| `wheel_joints` | `[joint_rueda_izq_tra, joint_rueda_izq_del, joint_rueda_der_tra, joint_rueda_der_del]` | Joints de las llantas en orden 1–4 |
| `from_urdf` | `true` | Tomar a, θ, dx, dy del URDF |
| `a`, `phi`, `theta`, `dx`, `dy` | Valores del Notion | Parámetros manuales (si `from_urdf:=false` o falla el URDF) |
| `init_from_odom` | `false` | Arrancar en la primera pose de `/odom` en vez de (0, 0, 0) |
| `odom_topic` | `/odom` | Tópico de la odometría del plugin |
| `csv_path` | `odometria_comparacion.csv` | Archivo de salida |

### Resultados y análisis

**1. La primera versión comparaba mal, no calculaba mal.** Al principio el nodo comparaba cada `/odom` con la última pose propia disponible. El error era de **~0.48 mm RMSE**, con picos que eran múltiplos exactos del movimiento del robot en un paso de simulación (1 ms): 0.5 mm a 0.5 m/s y 0.001 rad a 1 rad/s. El error venía de comparar poses de instantes distintos, no de la cinemática.

**2. Con la sincronización por marca de tiempo, el error baja a micrómetros o a cero.**
- En la corrida incluida en `odometria_comparacion.csv` (120 s, 36.5 m recorridos), el error es **cero** a la resolución del CSV (10⁻⁶ m y 10⁻⁶ rad).
- En otras corridas el RMSE fue del orden de **0.002 a 0.03 mm**.

**3. El método de integración no influye.** Se comparó Euler con la integración exacta sobre el arco y dieron resultados idénticos (RMSE igual hasta la tercera cifra). Con la frecuencia de `/joint_states`, el ángulo que gira el robot en cada Δt es tan pequeño que los dos métodos coinciden.

**4. El error residual sale de un único evento de orientación.** En las corridas con error residual, todo el error proviene de un solo evento. Por ejemplo, en t = 47.5 s, en un tramo recto, la odometría propia registró una rotación de 7.6×10⁻⁶ rad que el plugin no registró.
- La causa es que el nodo propio integra la **velocidad instantánea** de las llantas, mientras que el plugin usa su **posición angular**. Una perturbación breve en la velocidad de las ruedas afecta al primero pero casi no al segundo.
- Esa diferencia de orientación no se corrige y genera un error de posición proporcional a la distancia al punto donde ocurrió (e ≈ δψ·d). Este modelo explica más del 97 % del error observado.
- Es la limitación fundamental de la odometría por llantas: un error de orientación, por pequeño que sea, crece en posición a medida que el robot avanza.

**5. Ninguna de las dos odometrías ve el deslizamiento.** Las dos salen de las llantas y asumen rodadura sin deslizamiento. Un robot de 4 llantas fijas desliza lateralmente al girar, porque las llantas delanteras y traseras no pueden moverse de lado, y ese deslizamiento no está en Q. Que las dos odometrías coincidan no implica que sean exactas respecto a la pose real.

---

## Solución de problemas

| Problema | Solución |
|---|---|
| `No executable found` al hacer `ros2 run` | En `ament_cmake` el ejecutable conserva la extensión: `odometria_llantas.py`. Revisa además el `install(PROGRAMS ...)` en `CMakeLists.txt` y vuelve a compilar. |
| `Exec format error` | La primera línea del script debe ser el shebang `#!/usr/bin/python3`, sin líneas vacías antes. |
| `ModuleNotFoundError: No module named 'numpy'` | Hay otro entorno de Python activo (conda, venv, ESP-IDF). El shebang apunta a `/usr/bin/python3`; instala `python3-numpy` con `apt`. |
| Las gráficas se quedan en "Esperando /joint_states y /odom..." | Verifica que `/odom` llegue (`ros2 topic hz /odom`) y que `gz_topic_name` en `bridge.yaml` coincida con el `<odom_topic>` del plugin. |
| Warning `gz_frame_id ... not defined in SDF` | Es inofensivo: Gazebo copia la etiqueta igual y los sensores publican con el frame correcto. |

---

## Adaptación al MicroROS (tabla 3.1 del reporte)

Todas las medidas, masas y límites están en **`urdf/parametros_microros.xacro`**, que incluyen `robotMicroROS.urdf.xacro` y `robotPro.urdf.xacro`.

| Parámetro | Tabla | Modelo | Nota |
|---|---|---|---|
| L × A × H | 198.6 × 153 × 124.2 mm | 198.5 × 153.4 × 124.2 mm | Medido sobre el modelo |
| Masa | 0.906 kg | 0.906 kg | Chasis 0.6374 + 4 ruedas 0.05 + LiDAR 0.04 + cámara 0.0186 + antena 0.01 |
| Diámetro de rueda | ≈ 49 mm | 48 mm | CAD oficial de Yahboom |
| Vía | ≈ 134 mm | 131.5 mm | CAD; con 134 mm el ancho total daría 155.5 mm ≠ 153 mm |
| Distancia entre ejes | ≈ 96 mm | 95 mm | CAD oficial de Yahboom |
| v / ω máx. | 1.0 m/s / 4.0 rad/s | igual | `<max/min_linear_velocity>`, `<max/min_angular_velocity>` del DiffDrive |
| Aceleración lineal máx. | 2.0 m/s² | igual | `<max/min_linear_acceleration>` |
| LiDAR | 360° / 450, 0.03–12 m, 10 Hz, σ = 0.01 m | igual | — |
| Altura del LiDAR | ≈ 108 mm | 108 mm | Plano de escaneo; pasa 10 mm por encima de la cámara, sin auto-oclusión |

Los valores con ≈ son estimados del plano. Se usan los del CAD oficial de Yahboom porque con ellos el modelo reproduce las medidas de L × A × H de la tabla. Para usar los de la tabla basta cambiar tres líneas en `parametros_microros.xacro`; el nodo de odometría lee las medidas del URDF y recalcula Q y W solo.

Spawn en la pista de competencia: (x, y, ψ) = (−0.60, 0, 0), detrás de la línea de salida/meta.

---

## Mux + frenado autónomo de emergencia (AEB) — `aeb_mux.py`

```
/cmd_vel_teleop ─┐
                 ├─► [ mux según modo:=teleop|auto ] ─► [ AEB ] ─► /cmd_vel ─► DiffDrive
/cmd_vel_auto  ──┘                                         ▲ /scan, /odom
```

Teleoperación y autonomía publican en tópicos separados; el nodo pasa solo la fuente del `modo` y todo comando pasa por el AEB, que tiene prioridad en los dos modos. Republica a 20 Hz con el escaneo más reciente: como el DiffDrive mantiene el último comando, el AEB frena aunque no lleguen comandos nuevos. Si la fuente autónoma deja de publicar 0.5 s, manda cero (en teleop se mantiene el último comando, como sin mux). Sin `/scan` no deja avanzar.

**TTC (fórmula del reporte)**: para cada rayo, ṙ_i = −v cos θ_i y TTC_i = r_i / máx(−ṙ_i, 0); frena si mín TTC_i < umbral. Al frenar anula la velocidad lineal y deja pasar el giro, para poder alejarse.

**Para evitar frenados falsos:**
1. Solo cuentan los rayos dentro del **corredor** que barrerá el robot (ancho del robot + 3 cm a lo largo de la trayectoria predicha con v y ω; recta si ω ≈ 0 y arco de radio v/ω si no). Los muros laterales del estrechamiento y el muro exterior de una curva que el robot está tomando no cuentan.
2. r_i se mide desde el **borde** del robot (el LiDAR está en el centro), así TTC es el tiempo hasta el contacto.
3. v es la mayor entre la medida (`/odom`) y la comandada: con solo la medida, al frenar v → 0 y el freno se soltaría y volvería a activar a tirones.
4. Histéresis: se suelta con TTC > 1.3 × umbral.

**Umbral = 0.5 s**, con v_max = 1.0 m/s y a_max = 2.0 m/s²:
- frenado: v_max / (2·a_max) = 0.25 s (distancia de frenado v²/2a = 0.25 m);
- retraso del LiDAR a 10 Hz: 0.10 s;
- holgura: 0.15 s (≈ 0.15 m a v_max).

**Validación fuera de línea** (LiDAR simulado con rayos contra el STL de la pista de competencia, a 108 mm):

| Prueba | Resultado |
|---|---|
| Caja de 0.2 m en la recta; LiDAR 10 Hz, AEB 20 Hz, desaceleración 2 m/s² | Se detiene a 18 cm (1.0 m/s), 18 cm (0.6), 12 cm (0.3), 4 cm (0.1). Sin choques. |
| Muro de frente a 1.0 m/s (teleop) | Se detiene a 18 cm |
| Caja al lado de la trayectoria | No frena |
| Vuelta completa por la línea central a 1.0 m/s con ω = v·κ | Sin frenados en curvas ni chicana |
| Estrechamiento de 0.40 m a 1.0 m/s, en línea recta, centrado y a ±8 cm | Sin frenados (TTC mín ≈ 1.5 s) |
| Estrechamiento girando a ω ≈ 0.4 rad/s | Frena, correctamente: esa trayectoria toca el muro en ≈ 0.6 m. El controlador debe ir suave y más lento ahí. |

Depuración: `ros2 topic echo /aeb/activo`, `ros2 topic echo /aeb/ttc_min`.

Prueba sin el seguidor de pared (modo auto con un comando fijo):
```bash
ros2 launch mi_robot_diferencial robot_yahboom.launch.py pista:=competencia modo:=auto
ros2 topic pub -r 10 /cmd_vel_auto geometry_msgs/msg/Twist "{linear: {x: 0.8}}"
```

---

## Odometría en varias vueltas y tiempos por vuelta

`robot_yahboom.launch.py` lanza dos nodos más:

| Nodo | Qué hace |
|---|---|
| `odometria_vueltas.py` | Odometría por llantas con ω_I y ω_D = promedio de las dos ruedas de cada lado (requisito 2): u = a(ω_D+ω_I)/2, r = a(ω_D−ω_I)/b, Euler. Lee a y b del URDF. Arranca en la pose real inicial y publica `/odom_llantas`. |
| `detector_vueltas.py` | Detecta los cruces de la línea de salida/meta con la pose real, mide los tiempos por vuelta en tiempo de simulación y el error de cierre de `/odom_llantas` y de `/odom` (DiffDrive) en el instante exacto de cada cruce. Publica `/vuelta`. |

**Pose real (`/ground_truth`)**: plugin `OdometryPublisher` de Gazebo en el robot, que publica la pose del motor de física. Hace falta porque `/odom` (DiffDrive) y la odometría propia salen las dos de las ruedas y asumen rodadura sin deslizamiento: no ven el deslizamiento lateral del skid-steer al girar. **Solo se usa para medir**; ningún controlador lo lee (el modo autónomo es reactivo).

**Cruce de la línea**: la línea de cada pista está en `PISTAS` del launch (punto, dirección de avance y semiancho). Cuenta un cruce cuando la distancia con signo a la línea pasa de negativa a positiva dentro del semiancho y pasaron al menos 10 s desde el anterior. El instante se interpola entre muestras. El cruce 0 (salida) arranca el cronómetro; el tiempo total es hasta el cruce 3.

| Pista | Línea (x, y) | Avance | Spawn |
|---|---|---|---|
| `competencia` | (0, 0) | +x (antihorario) | (−0.60, 0, 0), reporte |
| `rainbow` | (−2.00, 4.30) | +x | (−2.60, 4.30, 0) |
| `pro` | (−0.636, 0.636) | −45° | (−1.06, 1.06, −45°) |

**Resultados**: al completar las vueltas (`vueltas:=3`) y al cerrar con Ctrl+C se guardan en `~/resultados_vueltas/<pista>_<fecha>/`:
`vueltas.csv` (tiempos y error de cierre por vuelta), `trayectoria.csv`, `trayectoria.png` (real vs. estimadas), `error_vs_distancia.png` (crecimiento del error) y `error_cierre.png` (por vuelta). La tabla también se imprime en la terminal.

```bash
ros2 launch mi_robot_diferencial robot_yahboom.launch.py pista:=competencia
ros2 topic echo /vuelta        # vueltas completadas
```

---

## Robot MicroROS con geometría simple y selección de pista

`urdf/robotMicroROS.urdf.xacro` es el MicroROS con cajas y cilindros (sin el modelo detallado del fabricante): cuerpo inferior, separadores, placa de control, cubierta, LiDAR y cabeza del PTZ con la cámara. Conserva las medidas de la tabla (198.5 × 153.4 × 124.2 mm, 0.906 kg), las ruedas, los joints, los plugins y los sensores. La cabeza de la cámara queda 10 mm por debajo del plano del LiDAR para que el LiDAR no la vea.

```bash
colcon build --packages-select mi_robot_diferencial
source install/setup.bash
ros2 launch mi_robot_diferencial robot_yahboom.launch.py                     # pista_pro, robot MicroROS
ros2 launch mi_robot_diferencial robot_yahboom.launch.py pista:=rainbow
ros2 launch mi_robot_diferencial robot_yahboom.launch.py pista:=competencia
ros2 launch mi_robot_diferencial robot_yahboom.launch.py robot:=pro          # robot de cajas original
```

| `pista:=` | Mundo | Spawn (x, y, z, yaw) |
|---|---|---|
| `pro` (defecto) | `pista_pro.sdf`, el "8" con puente | (−1.06, 1.06, 0.10, −0.785) |
| `rainbow` | `rainbow_road.sdf`: vía de 5 cm con muros de 20 cm, 9.65 × 9.7 m, con franjas arcoíris de 25 cm (`scripts/generar_rainbow.py`) | (−2.60, 4.30, 0.15, 0) |
| `competencia` | `pista_competencia.sdf`: muros de 25 cm, 16.1 × 6 m | (−0.60, 0, 0.10, 0) |
| `rr_og` | `rr_og.sdf`: Rainbow Road con la isla original y chaflán a la entrada del corredor inferior | (−2.60, 4.30, 0.15, 0) |
| `rr_wide` | `rr_wide.sdf`: Rainbow Road con los ramales de la isla ensanchados (0.52 / 0.44 m) y chaflán | (−2.60, 4.30, 0.15, 0) |
| `rr_noisland` | `rr_noisland.sdf`: Rainbow Road sin isla (rellena con vía) y chaflán | (−2.60, 4.30, 0.15, 0) |

Las mallas de las pistas están en `meshes/pistas/` (en metros; el STL original de Rainbow Road venía en mm). Para agregar otra pista se pone su `.sdf` en `worlds/` y una línea en el diccionario `PISTAS` del launch.

Cambios respecto a `robotPro`:
- Chasis de 0.165 × 0.108 × 0.077 m (visual y colisión) y cabeza de la cámara al frente. Las colisiones de las llantas siguen siendo cilindros (r = 0.024 m), así que la cinemática y Q/W no cambian.
- LiDAR en (0.002, 0, 0.074) m respecto a `base_link`, centrado sobre el chasis (antes x = 0.013).
- Cámara en el lente, (0.120, 0, 0.0485) m. El PTZ queda fijo en posición central.
- Los mundos usan el detector de colisiones Bullet en vez de ODE, que es inestable con los STL de las pistas.
- El launch agrega `share/` a `GZ_SIM_RESOURCE_PATH` para que Gazebo encuentre las mallas de las pistas.
