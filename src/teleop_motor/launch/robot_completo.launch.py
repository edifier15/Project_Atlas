import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Encontra o caminho para a pasta do seu pacote
    pkg_teleop_motor = get_package_share_directory('teleop_motor')
    
    # Encontra o caminho para o seu arquivo de configuração YAML
    # (Assumindo que ele se chama 'joystick_config.yaml' dentro da pasta 'config')
    config_filepath = os.path.join(pkg_teleop_motor, 'config', 'joystick_config.yaml')

    return LaunchDescription([
        
        # 1. Inicia o Agente micro-ROS
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            # --- Mude aqui se for Wi-Fi ---
            # Use arguments=['udp4', '--port', '8888'] para Wi-Fi
            arguments=['serial', '--dev', '/dev/ttyUSB0'], 
            output='screen'
        ),
        
        # 2. Inicia o nó do Joystick
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            # Trava o nó para usar a porta js0 e evita erros de permissão
            parameters=[{'dev': '/dev/input/js0'}] 
        ),
        
        # 3. Inicia o seu nó tradutor (O Cérebro)
        Node(
            package='teleop_motor',
            executable='joy_motor',  # O nome do 'console_script' no setup.py
            name='joystick_teleop',  # O nome que demos no arquivo YAML
            output='screen',
            # <<< CARREGA TODAS AS SUAS CONFIGURAÇÕES (velocidades, botões, etc.)
            parameters=[config_filepath] 
        ),
        
        # 4. Inicia o Publicador de Transformação Estática (TF)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            # Diz ao ROS que 'laser_link' está na mesma posição que 'base_link'
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link'],
        ),
        
        # 5. Inicia o Nó Conversor de Scan para Nuvem de Pontos
        Node(
            package='pointcloud_to_laserscan',
            executable='laserscan_to_pointcloud_node',
            name='scan_to_cloud_converter',
            # Diz ao nó para ouvir o /scan e publicar no /scan_cloud
            remappings=[('scan_in', '/scan'), ('cloud_out', '/scan_cloud')]
        ),
    ])