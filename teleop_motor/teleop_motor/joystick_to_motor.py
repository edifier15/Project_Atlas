import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickMotorController(Node):
    def __init__(self):
        super().__init__('joystick_motor_controller')

        # --- Parâmetros Configuráveis ---
        # Mapeamento
        self.declare_parameter('axis_linear', 1)
        self.declare_parameter('button_mode_select', 4)
        self.declare_parameter('button_preset_A', 0)
        self.declare_parameter('button_preset_B', 1)
        self.declare_parameter('button_preset_X', 2)
        self.declare_parameter('button_preset_Y', 3)
        
        # Velocidades (Presets para frente)
        self.declare_parameter('speed_multiplier', 5.0)
        self.declare_parameter('speed_preset_A', 3.5)
        self.declare_parameter('speed_preset_X', 4.0)
        self.declare_parameter('speed_preset_B', 4.5)
        self.declare_parameter('speed_preset_Y', 5.0)
        
        # <<< NOVO: Parâmetro da Rampa de Aceleração >>>
        # Define o quão rápido a velocidade muda (em rad/s por segundo)
        self.declare_parameter('acceleration', 1.0) 
        
        # Parâmetro para a lógica de 20Hz
        self.declare_parameter('publish_rate_hz', 5.0)

        # --- Variáveis de Estado ---
        self.control_mode = 'ANALOG'
        self.mode_button_last_state = 0
        self.target_speed = 0.0      # A velocidade que DESEJAMOS atingir
        self.current_speed = 0.0     # A velocidade ATUAL que estamos enviando
        
        # --- Subscriber e Publisher ---
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # --- O Timer agora é nosso loop de controle principal ---
        self.publish_period = 1.0 / self.get_parameter('publish_rate_hz').value
        self.control_timer = self.create_timer(self.publish_period, self.control_loop_callback)

        self.get_logger().info("Joystick controller with ACCELERATION RAMP started.")


    def joy_callback(self, msg):
        """Esta função agora apenas define a VELOCIDADE ALVO (target_speed)."""
        
        mode_select_button = self.get_parameter('button_mode_select').value
        
        # Lógica para Trocar de Modo
        if msg.buttons[mode_select_button] == 1 and self.mode_button_last_state == 0:
            if self.control_mode == 'ANALOG': self.control_mode = 'PRESET'
            else: self.control_mode = 'ANALOG'
            self.get_logger().info(f"Switched to '{self.control_mode}' mode.")
        self.mode_button_last_state = msg.buttons[mode_select_button]

        # Determina a velocidade alvo com base no modo
        temp_target = 0.0
        if self.control_mode == 'ANALOG':
            axis_linear = self.get_parameter('axis_linear').value
            speed_multiplier = self.get_parameter('speed_multiplier').value
            joystick_value = msg.axes[axis_linear]
            temp_target = max(0.0, joystick_value) * speed_multiplier
        
        elif self.control_mode == 'PRESET':
            # Se nenhum botão de preset for pressionado, a velocidade alvo é 0
            if msg.buttons[self.get_parameter('button_preset_A').value] == 1:
                temp_target = self.get_parameter('speed_preset_A').value
            elif msg.buttons[self.get_parameter('button_preset_X').value] == 1:
                temp_target = self.get_parameter('speed_preset_X').value
            elif msg.buttons[self.get_parameter('button_preset_B').value] == 1:
                temp_target = self.get_parameter('speed_preset_B').value
            elif msg.buttons[self.get_parameter('button_preset_Y').value] == 1:
                temp_target = self.get_parameter('speed_preset_Y').value
        
        self.target_speed = temp_target

    def control_loop_callback(self):
        """Este loop roda a 20Hz e ajusta a velocidade gradativamente."""
        
        # Pega o valor da aceleração do parâmetro
        acceleration = self.get_parameter('acceleration').value
        # Calcula o quanto a velocidade pode mudar neste ciclo (neste "tick" de 1/20 de segundo)
        speed_step = acceleration * self.publish_period
        if self.control_mode == 'ANALOG':
            self.current_speed = self.target_speed
        # Compara a velocidade atual com a alvo e ajusta
        elif self.current_speed < self.target_speed:
            self.current_speed += speed_step
            # Garante que não ultrapassemos o alvo
            if self.current_speed > self.target_speed:
                self.current_speed = self.target_speed
        
        elif self.current_speed > self.target_speed:
            self.current_speed -= speed_step
            # Garante que não fiquemos abaixo do alvo
            if self.current_speed < self.target_speed:
                self.current_speed = self.target_speed

        # Publica a velocidade ATUAL, que muda gradativamente
        twist_msg = Twist()
        twist_msg.angular.z = self.current_speed
        self.publisher_.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickMotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()