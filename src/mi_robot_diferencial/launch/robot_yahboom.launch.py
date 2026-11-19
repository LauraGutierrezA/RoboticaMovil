"""
Lanza la simulación con el robot y la pista que se elijan.

  ros2 launch mi_robot_diferencial robot_yahboom.launch.py                      # pista_pro (por defecto)
  ros2 launch mi_robot_diferencial robot_yahboom.launch.py pista:=rainbow
  ros2 launch mi_robot_diferencial robot_yahboom.launch.py pista:=competencia
  ros2 launch mi_robot_diferencial robot_yahboom.launch.py robot:=pro          # robot de cajas original (por defecto: microros)
  ros2 launch mi_robot_diferencial robot_yahboom.launch.py modo:=auto          # obedece /cmd_vel_auto
  ros2 launch mi_robot_diferencial robot_yahboom.launch.py modo:=auto algoritmo:=ninguno   # auto sin controlador
                                                                               # (para publicar /cmd_vel_auto a mano)
El algoritmo autónomo (por defecto wall_follow.py, que publica en /cmd_vel_auto y pasa por el
AEB) se lanza SIEMPRE: con modo:=auto arranca activo; con modo:=teleop arranca detenido
(activo=false). Desde la consola web (volante:=true, pestaña Wall following) se cambia de
manual a autónomo y de vuelta sin relanzar (cambia `modo` de /aeb_mux y `activo` de /wall_follow).

Los comandos pasan siempre por aeb_mux.py (mux + frenado de emergencia) antes de llegar a /cmd_vel.
Teleoperación, en otra terminal:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop
o con el celular como volante (abre https://<IP del PC>:8443 en el celular):
  ros2 launch mi_robot_diferencial robot_yahboom.launch.py volante:=true

Para agregar otra pista: poner el .sdf en worlds/ y una línea en PISTAS.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, AppendEnvironmentVariable,
                            DeclareLaunchArgument, OpaqueFunction)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Por pista: mundo, spawn (x, y, z, yaw) y línea de salida/meta para detector_vueltas.py
#   linea = (x, y, yaw de avance, semiancho): el robot cruza cuando pasa la línea en la
#   dirección yaw dentro de ±semiancho (los demás tramos de la pista no cuentan)
PISTAS = {
    'pro': dict(mundo='pista_pro.sdf', spawn=(-1.06, 1.06, 0.10, -0.785),       # recta B, hacia el cruce
                linea=(-0.636, 0.636, -0.785, 0.6)),                            # 0.6 m delante del spawn
    'rainbow': dict(mundo='rainbow_road.sdf', spawn=(-2.60, 4.30, 0.15, 0.0),   # encima de la vía de 5 cm
                    linea=(-2.00, 4.30, 0.0, 0.45)),                            # donde empieza la franja roja
    'competencia': dict(mundo='pista_competencia.sdf', spawn=(-0.60, 0.00, 0.10, 0.0),  # reporte: detrás de x=0
                        linea=(0.0, 0.0, 0.0, 0.6)),                            # reporte: línea en x=0, antihorario
}
# Por robot: xacro y huella vista desde el LiDAR para el AEB (frente, atrás, semiancho) [m]
ROBOTS = {
    'microros': dict(xacro='robotMicroROS.urdf.xacro',  # MicroROS con geometría simple (medidas de la tabla)
                     huella=(0.118, 0.0805, 0.0765)),   # LiDAR en x=0.002: lente / chasis / llantas
    'pro':      dict(xacro='robotPro.urdf.xacro',       # robot original de cajas
                     huella=(0.0863, 0.1123, 0.0765)),  # LiDAR en x=0.013: caja de 0.1986 m / llantas
}


def lanzar(context):
    pkg = get_package_share_directory('mi_robot_diferencial')
    pista = LaunchConfiguration('pista').perform(context)
    robot = LaunchConfiguration('robot').perform(context)

    cfg = PISTAS[pista]
    x, y, z, yaw = cfg['spawn']
    lx, ly, lyaw, lsemi = cfg['linea']
    world_path = os.path.join(pkg, 'worlds', cfg['mundo'])
    modo = LaunchConfiguration('modo').perform(context)
    algoritmo = LaunchConfiguration('algoritmo').perform(context)
    frente, atras, semiancho = ROBOTS[robot]['huella']
    xacro_path = os.path.join(pkg, 'urdf', ROBOTS[robot]['xacro'])
    bridge_config = os.path.join(pkg, 'config', 'bridge.yaml')

    # El URDF generado por xacro se pasa como string, no como YAML
    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    nodos_extra = []
    if LaunchConfiguration('volante').perform(context) == 'true':
        # El celular como volante -> /cmd_vel_teleop (pasa por el AEB igual que el teclado)
        nodos_extra.append(Node(package='mi_robot_diferencial', executable='volante_web.py',
                               output='screen'))
    if algoritmo != 'ninguno':
        # Controlador autónomo reactivo: /scan -> /cmd_vel_auto (el AEB va detrás, en aeb_mux).
        # En teleop arranca detenido; se activa desde la consola o con ros2 param set.
        nodos_extra.append(Node(package='mi_robot_diferencial', executable=algoritmo + '.py',
                               output='screen', parameters=[{'use_sim_time': True,
                                                             'activo': modo == 'auto'}]))

    return nodos_extra + [
        # Gazebo con la pista, arrancando sin pausa (-r)
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-v4', world_path], output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            # El nombre 'mi_robot' se mantiene: bridge.yaml usa /model/mi_robot/tf
            arguments=['-topic', 'robot_description', '-name', 'mi_robot',
                       '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw)]
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_config}]
        ),
        Node(package='rviz2', executable='rviz2', parameters=[{'use_sim_time': True}]),
        # Mux + AEB: /cmd_vel_teleop o /cmd_vel_auto (según modo) -> AEB -> /cmd_vel
        Node(package='mi_robot_diferencial', executable='aeb_mux.py', output='screen',
             parameters=[{'use_sim_time': True, 'modo': modo,
                          'frente': frente, 'atras': atras, 'semiancho': semiancho}]),
        # Odometría por llantas (ω_I, ω_D promediados) -> /odom_llantas
        Node(package='mi_robot_diferencial', executable='odometria_vueltas.py',
             output='screen', parameters=[{'use_sim_time': True}]),
        # Tiempos por vuelta y error de cierre frente a /ground_truth (solo mide)
        Node(package='mi_robot_diferencial', executable='detector_vueltas.py',
             output='screen', parameters=[{
                 'use_sim_time': True, 'nombre_pista': pista,
                 'linea_x': lx, 'linea_y': ly, 'linea_yaw': lyaw, 'linea_semiancho': lsemi,
                 'vueltas': int(LaunchConfiguration('vueltas').perform(context)),
                 'carpeta_salida': LaunchConfiguration('carpeta_salida').perform(context)}]),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('mi_robot_diferencial')
    return LaunchDescription([
        DeclareLaunchArgument('pista', default_value='pro', choices=list(PISTAS),
                              description='Pista: ' + ', '.join(PISTAS)),
        DeclareLaunchArgument('robot', default_value='microros', choices=list(ROBOTS),
                              description='Modelo del robot: ' + ', '.join(ROBOTS)),
        DeclareLaunchArgument('modo', default_value='teleop', choices=['teleop', 'auto'],
                              description='teleop: /cmd_vel_teleop pasa al robot; auto: /cmd_vel_auto'),
        DeclareLaunchArgument('volante', default_value='false', choices=['true', 'false'],
                              description='true: lanza volante_web.py (el celular como volante)'),
        DeclareLaunchArgument('algoritmo', default_value='wall_follow',
                              choices=['wall_follow', 'ninguno'],
                              description='Controlador autónomo (activo si modo:=auto; detenido si teleop)'),
        DeclareLaunchArgument('vueltas', default_value='3',
                              description='Vueltas tras las que detector_vueltas guarda resultados'),
        DeclareLaunchArgument('carpeta_salida', default_value='~/resultados_vueltas',
                              description='Carpeta para CSV y gráficas de vueltas'),
        # Gazebo resuelve package:// y model://mi_robot_diferencial/meshes/... buscando en
        # GZ_SIM_RESOURCE_PATH, así que se agrega la carpeta share/ del workspace
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg)),
        OpaqueFunction(function=lanzar),
    ])
