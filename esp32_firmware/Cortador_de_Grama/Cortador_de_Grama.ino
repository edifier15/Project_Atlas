#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// --- Mensagens ROS ---
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/laser_scan.h>

// --- Drivers de Hardware ---
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <Wire.h>

// ============================================================
// 1. PAINEL DE CONTROLE (CONFIGURE AQUI O SEU TESTE)
// ============================================================

// [CONEXÃO]
// Comente para usar CABO USB. Descomente para usar WI-FI.
// #define USAR_WIFI  

// [MODULOS]
// Comente esta linha se o LIDAR não estiver conectado.
// Isso evita que o ESP32 trave procurando sensores I2C que não existem.
// #define USAR_LIDAR 

#ifdef USAR_WIFI
    char ssid[] = "NOME_DA_SUA_REDE";
    char psk[] = "SENHA_DA_SUA_REDE";
    char agent_ip[] = "192.168.1.100"; 
    size_t agent_port = 8888;
#endif

// ============================================================
// 2. INCLUSÕES CONDICIONAIS
// ============================================================
#ifdef USAR_LIDAR
    #include <SparkFun_VL53L1X.h> 
    #include "AS5600.h"           
#endif

// ============================================================
// 3. DEFINIÇÃO DE PINOS
// ============================================================

// Rodas
#define FL_STEP 26
#define FL_DIR  27
#define RL_STEP 14
#define RL_DIR  12
#define FR_STEP 33
#define FR_DIR  25
#define RR_STEP 32
#define RR_DIR  35

// Lâmina
#define BLADE_PIN 13 

// LiDAR (Só define pinos se estiver ativo)
#ifdef USAR_LIDAR
    #define LIDAR_STEP 19 
    #define LIDAR_DIR  18 
#endif

// ============================================================
// 4. OBJETOS E VARIÁVEIS
// ============================================================

// Cinemática
const float STEPS_PER_REV_WHEEL = 2400.0; 
const float WHEEL_DIAMETER = 0.10; 
const float TRACK_WIDTH = 0.30; 

// Motores de Tração
AccelStepper motorFL(AccelStepper::DRIVER, FL_STEP, FL_DIR);
AccelStepper motorRL(AccelStepper::DRIVER, RL_STEP, RL_DIR);
AccelStepper motorFR(AccelStepper::DRIVER, FR_STEP, FR_DIR);
AccelStepper motorRR(AccelStepper::DRIVER, RR_STEP, RR_DIR);

// Lâmina
Servo bladeESC;

// micro-ROS Global
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t sub_cmd_vel;
geometry_msgs__msg__Twist msg_twist;
rcl_subscription_t sub_blade;
std_msgs__msg__Int32 msg_blade;
rcl_publisher_t debug_publisher;
geometry_msgs__msg__Twist debug_msg; // Vamos usar a mesma mensagem Twist para devolver
// --- BLOCO EXCLUSIVO LIDAR ---
#ifdef USAR_LIDAR
    AccelStepper motorLidar(AccelStepper::DRIVER, LIDAR_STEP, LIDAR_DIR);
    SFEVL53L1X distanceSensor;
    AS5600 as5600; 
    rcl_publisher_t pub_lidar;
    sensor_msgs__msg__LaserScan msg_scan;

    #define NUM_READINGS 60 
    float ranges_buffer[NUM_READINGS]; 
    int current_reading_index = 0;
#endif

// ============================================================
// 5. FUNÇÕES AUXILIARES
// ============================================================

float m_s_to_steps(float velocity_m_s) {
    float circumference = PI * WHEEL_DIAMETER;
    return (velocity_m_s / circumference) * STEPS_PER_REV_WHEEL;
}

// Inicializa mensagem Scan apenas se o LiDAR estiver ativo
void init_lidar_message() {
    #ifdef USAR_LIDAR
        static float memory_ranges[NUM_READINGS];
        msg_scan.ranges.data = memory_ranges;
        msg_scan.ranges.capacity = NUM_READINGS;
        msg_scan.ranges.size = 0;
        msg_scan.header.frame_id.data = (char*)"laser_frame";
        msg_scan.header.frame_id.size = strlen(msg_scan.header.frame_id.data);
        msg_scan.angle_min = 0.0;
        msg_scan.angle_max = 2.0 * PI;
        msg_scan.angle_increment = (2.0 * PI) / NUM_READINGS;
        msg_scan.range_min = 0.05; 
        msg_scan.range_max = 4.0;  
    #endif
}

// Callback Movimento
void cmd_vel_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    float linear_x = msg->linear.x;
    float angular_z = msg->angular.z;

    // --- PARTE NOVA: DEBUG ---
    // Copia os valores recebidos para a mensagem de saída
    debug_msg.linear.x = linear_x;
    debug_msg.angular.z = angular_z;
    
    // Publica de volta para o PC ver
    rcl_publish(&debug_publisher, &debug_msg, NULL);

    float vel_l = linear_x - (angular_z * (TRACK_WIDTH / 2.0));
    float vel_r = linear_x + (angular_z * (TRACK_WIDTH / 2.0));

    motorFL.setSpeed(m_s_to_steps(vel_l));
    motorRL.setSpeed(m_s_to_steps(vel_l));
    motorFR.setSpeed(m_s_to_steps(vel_r)); 
    motorRR.setSpeed(m_s_to_steps(vel_r));
}

// Callback Lâmina
void blade_callback(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    int val = constrain(msg->data, 0, 100);
    bladeESC.writeMicroseconds(map(val, 0, 100, 1000, 2000));
}

// ============================================================
// 6. SETUP
// ============================================================
void setup() {
    Serial.begin(115200);

    // --- Configuração de Transporte ---
    #ifdef USAR_WIFI
        set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
    #else
        set_microros_transports(); 
    #endif

    // --- Configuração de Hardware Base (Tração + Lâmina) ---
    motorFL.setMaxSpeed(4000); 
    motorRL.setMaxSpeed(4000);
    motorFR.setMaxSpeed(4000);
    motorRR.setMaxSpeed(4000);

    bladeESC.attach(BLADE_PIN, 1000, 2000);
    bladeESC.writeMicroseconds(1000); // Arming

    // --- Configuração Condicional do LiDAR ---
    #ifdef USAR_LIDAR
        Wire.begin(); 
        Wire.setClock(400000); 

        if (distanceSensor.begin() != 0) Serial.println("Erro VL53L1X");
        distanceSensor.setDistanceModeShort(); 
        distanceSensor.startRanging(); 

        if (!as5600.isConnected()) Serial.println("Erro AS5600");

        motorLidar.setMaxSpeed(1000); 
        motorLidar.setSpeed(200); 
        
        init_lidar_message();
    #endif

    delay(2000); 

    // --- micro-ROS Setup ---
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "lawnmower_v4", "", &support);

    // Subscribers sempre existem
    rclc_subscription_init_default(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
    rclc_subscription_init_default(&sub_blade, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "cmd_blade");

    // Publisher só existe se LiDAR estiver ativo
    #ifdef USAR_LIDAR
        rclc_publisher_init_default(&pub_lidar, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan");
    #endif
    rclc_publisher_init_default(
            &debug_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "debug_vel" // Tópico de retorno
        );
    // Executor init
    rclc_executor_init(&executor, &support.context, 2, &allocator); // 2 Handles fixos (Cmd + Blade)
    rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_twist, &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_blade, &msg_blade, &blade_callback, ON_NEW_DATA);
}

// ============================================================
// 7. LOOP
// ============================================================
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

    // Tração roda sempre
    motorFL.runSpeed();
    motorRL.runSpeed();
    motorFR.runSpeed();
    motorRR.runSpeed();

    // LiDAR roda condicionalmente
    #ifdef USAR_LIDAR
        motorLidar.runSpeed();

        if (distanceSensor.checkForDataReady()) {
            float dist_meters = distanceSensor.getDistance() / 1000.0;
            distanceSensor.clearInterrupt(); 

            if (current_reading_index < NUM_READINGS) {
                msg_scan.ranges.data[current_reading_index] = dist_meters;
                current_reading_index++;
            } else {
                int64_t time = rmw_uros_epoch_synchronized();
                msg_scan.header.stamp.sec = time / 1000000000;
                msg_scan.header.stamp.nanosec = time % 1000000000;
                msg_scan.ranges.size = NUM_READINGS;
                
                rcl_publish(&pub_lidar, &msg_scan, NULL);
                current_reading_index = 0;
            }
        }
    #endif
}