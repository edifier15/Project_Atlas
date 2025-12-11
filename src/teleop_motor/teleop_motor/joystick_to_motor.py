import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32 

class JoystickMotorController(Node):
    def __init__(self):
        super().__init__('joystick_motor_controller')

        # --- Parâmetros ---
        self.declare_parameter('axis_linear', 1) 
        self.declare_parameter('axis_angular', 3) 
        self.declare_parameter('button_blade', 0) 
        
        self.declare_parameter('max_speed_linear', 0.5)
        self.declare_parameter('max_speed_angular', 1.0)
        
        self.declare_parameter('acceleration', 1.0) 
        # Mantive seus 2.0Hz para teste, mas para dirigir suave o ideal é ~10.0Hz
        self.declare_parameter('publish_rate_hz', 20.0) 

        # --- Variáveis de Estado ---
        self.target_linear = 0.0
        self.current_linear = 0.0
        
        self.target_angular = 0.0
        self.current_angular = 0.0
        
        # Variável para guardar o estado da lâmina
        self.blade_state = 0 

        # --- Publishers e Subscribers ---
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_blade = self.create_publisher(Int32, 'cmd_blade', 10)
        
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Loop Principal
        self.publish_period = 1.0 / self.get_parameter('publish_rate_hz').value
        self.control_timer = self.create_timer(self.publish_period, self.control_loop_callback)

        self.get_logger().info("Joystick Controller OTIMIZADO iniciado.")

    def joy_callback(self, msg):
        """
        Função Leve: Apenas lê os inputs e salva em variáveis.
        NÃO PUBLICA NADA AQUI.
        """
        
        # 1. Leitura dos Eixos
        axis_lin_idx = self.get_parameter('axis_linear').value
        axis_ang_idx = self.get_parameter('axis_angular').value
        max_lin = self.get_parameter('max_speed_linear').value
        max_ang = self.get_parameter('max_speed_angular').value

        self.target_linear = msg.axes[axis_lin_idx] * max_lin
        self.target_angular = msg.axes[axis_ang_idx] * max_ang

        # 2. Leitura do Botão da Lâmina
        btn_blade_idx = self.get_parameter('button_blade').value
        
        if msg.buttons[btn_blade_idx] == 1:
            self.blade_state = 100
        else:
            self.blade_state = 0
            
    def control_loop_callback(self):
        """
        Função Rítmica: Roda a 2Hz (ou o que configurar).
        Processa a rampa e publica TODOS os comandos.
        """
        
        # --- A. Lógica da Rampa (Motion) ---
        acceleration = self.get_parameter('acceleration').value
        step = acceleration * self.publish_period

        # Rampa Linear
        if self.current_linear < self.target_linear:
            self.current_linear = min(self.current_linear + step, self.target_linear)
        elif self.current_linear > self.target_linear:
            self.current_linear = max(self.current_linear - step, self.target_linear)

        # Rampa Angular
        if self.current_angular < self.target_angular:
            self.current_angular = min(self.current_angular + step, self.target_angular)
        elif self.current_angular > self.target_angular:
            self.current_angular = max(self.current_angular - step, self.target_angular)

        # Publica CMD_VEL
        twist_msg = Twist()
        twist_msg.linear.x = self.current_linear
        twist_msg.angular.z = self.current_angular
        self.pub_cmd_vel.publish(twist_msg)
        
        # --- B. Lógica da Lâmina ---
        # Publica CMD_BLADE no mesmo ritmo
        blade_msg = Int32()
        blade_msg.data = self.blade_state
        self.pub_blade.publish(blade_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickMotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()