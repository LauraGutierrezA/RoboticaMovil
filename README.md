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
ros2 run teleop_twist_keyboard teleop_twist_keyboard
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
| Masa del chasis | 0.75 kg | — |

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
