#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <esp_task_wdt.h> 
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/laser_scan.h>

#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <Wire.h>

// --- SELETORES ---
// #define USAR_WIFI  
// #define USAR_LIDAR 

#ifdef USAR_WIFI
    char ssid[] = "SUA_REDE";
    char psk[] = "SUA_SENHA";
    char agent_ip[] = "192.168.1.100";
    size_t agent_port = 8888;
#endif

#ifdef USAR_LIDAR
    #include <SparkFun_VL53L1X.h> 
    #include "AS5600.h"           
#endif

// --- PINOS ---
#define FL_STEP 26
#define FL_DIR  27
#define RL_STEP 14
#define RL_DIR  12
#define FR_STEP 33
#define FR_DIR  25
#define RR_STEP 32
#define RR_DIR  35

#define LIDAR_STEP 19 
#define LIDAR_DIR  18 
#define BLADE_PIN 13 

// --- OBJETOS ---
AccelStepper motorFL(AccelStepper::DRIVER, FL_STEP, FL_DIR);
AccelStepper motorRL(AccelStepper::DRIVER, RL_STEP, RL_DIR);
AccelStepper motorFR(AccelStepper::DRIVER, FR_STEP, FR_DIR);
AccelStepper motorRR(AccelStepper::DRIVER, RR_STEP, RR_DIR);
AccelStepper motorLidar(AccelStepper::DRIVER, LIDAR_STEP, LIDAR_DIR);

Servo bladeESC;

// --- VARIÁVEIS GLOBAIS DE CONTROLE (COMPARTILHADAS ENTRE CORES) ---
// Usamos 'volatile' para garantir que os núcleos leiam o valor atualizado
volatile float shared_speed_FL = 0;
volatile float shared_speed_RL = 0;
volatile float shared_speed_FR = 0;
volatile float shared_speed_RR = 0;
volatile float shared_speed_Lidar = 0;
volatile unsigned long last_cmd_time = 0;
// Configuração Física
const float STEPS_PER_REV_WHEEL = 3200.0; // Microstepping 1/16 ativado
const float WHEEL_DIAMETER = 0.10; 
const float TRACK_WIDTH = 0.30; 

// ROS Objects
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t sub_cmd_vel;
geometry_msgs__msg__Twist msg_twist;
rcl_subscription_t sub_blade;
std_msgs__msg__Int32 msg_blade;

#ifdef USAR_LIDAR
    SFEVL53L1X distanceSensor;
    AS5600 as5600; 
    rcl_publisher_t pub_lidar;
    sensor_msgs__msg__LaserScan msg_scan;
    #define NUM_READINGS 60 
    float ranges_buffer[NUM_READINGS]; 
    int current_reading_index = 0;
#endif

// ==========================================
// TAREFA DEDICADA AOS MOTORES (CORE 0)
// ==========================================
void motorTask(void * parameter) {
    // Configura velocidade máxima AQUI dentro também, por segurança
    motorFL.setMaxSpeed(4000); 
    motorRL.setMaxSpeed(4000);
    motorFR.setMaxSpeed(4000);
    motorRR.setMaxSpeed(4000);
    motorLidar.setMaxSpeed(1000);

    for(;;) { // Loop infinito da tarefa
        
        // 1. Atualiza as velocidades com base nas variáveis globais
        // Isso é super rápido
        motorFL.setSpeed(shared_speed_FL);
        motorRL.setSpeed(shared_speed_RL);
        motorFR.setSpeed(shared_speed_FR);
        motorRR.setSpeed(shared_speed_RR);
        motorLidar.setSpeed(shared_speed_Lidar);

        // 2. Gera os pulsos
        // Como nada mais roda neste núcleo, isso acontece perfeitamente no tempo
        motorFL.runSpeed();
        motorRL.runSpeed();
        motorFR.runSpeed();
        motorRR.runSpeed();
        motorLidar.runSpeed();

    }
}

// ==========================================
// CALLBACKS (RODAM NO CORE 1 - ROS)
// ==========================================
float m_s_to_steps(float velocity_m_s) {
    float circumference = PI * WHEEL_DIAMETER;
    return (velocity_m_s / circumference) * STEPS_PER_REV_WHEEL;
}

void cmd_vel_callback(const void * msgin) {
    last_cmd_time = millis();
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    
    // Cálculos de Cinemática
    float linear_x = msg->linear.x;
    float angular_z = msg->angular.z;
    float vel_l = linear_x - (angular_z * (TRACK_WIDTH / 2.0));
    float vel_r = linear_x + (angular_z * (TRACK_WIDTH / 2.0));

    // EM VEZ DE setSpeed(), ATUALIZAMOS AS VARIÁVEIS GLOBAIS
    shared_speed_FL = m_s_to_steps(vel_l);
    shared_speed_RL = m_s_to_steps(vel_l);
    // Invertendo o lado direito se necessário (teste prático)
    shared_speed_FR = m_s_to_steps(vel_r); 
    shared_speed_RR = m_s_to_steps(vel_r);
}

void blade_callback(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    int val = constrain(msg->data, 0, 100);
    bladeESC.writeMicroseconds(map(val, 0, 100, 1000, 2000));
}

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

// ==========================================
// SETUP
// ==========================================
void setup() {
    disableCore0WDT();
    Serial.begin(115200);

    // 1. Configura Hardware (Pinos, Drivers)
    // Se tiver pinos de ENABLE, ative-os aqui como LOW
    
    #ifdef USAR_WIFI
        set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
    #else
        set_microros_transports(); 
    #endif

    Wire.begin(); 
    Wire.setClock(400000); 

    bladeESC.attach(BLADE_PIN, 1000, 2000);
    bladeESC.writeMicroseconds(1000); 

    #ifdef USAR_LIDAR
        if (distanceSensor.begin() != 0) Serial.println("Erro VL53L1X");
        distanceSensor.setDistanceModeShort(); 
        distanceSensor.startRanging(); 
        if (!as5600.isConnected()) Serial.println("Erro AS5600");
        
        shared_speed_Lidar = 200; // Velocidade fixa para o LiDAR
        init_lidar_message();
    #endif

    delay(2000); 

    // 2. INICIA A TAREFA DOS MOTORES NO CORE 0 (A MÁGICA)
    xTaskCreatePinnedToCore(
        motorTask,   // Função da tarefa
        "MotorTask", // Nome
        10000,       // Tamanho da pilha (Stack size)
        NULL,        // Parâmetros
        1,           // Prioridade (1 = Alta)
        NULL,        // Handle
        0            // Core ID (0 ou 1) -> Vamos usar o 0!
    );

    // 3. Configura micro-ROS (Core 1)
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "lawnmower_dualcore", "", &support);

    // Use BEST EFFORT para joystick!
    rclc_subscription_init_best_effort(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
    rclc_subscription_init_default(&sub_blade, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "cmd_blade");

    #ifdef USAR_LIDAR
        rclc_publisher_init_default(&pub_lidar, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan");
    #endif

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_twist, &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_blade, &msg_blade, &blade_callback, ON_NEW_DATA);
}

// ==========================================
// LOOP PRINCIPAL (CORE 1 - ROS & SENSORES)
// ==========================================
void loop() {
    // Agora o loop pode demorar o quanto quiser que NÃO trava o motor!
    
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

    #ifdef USAR_LIDAR
        // Leitura do I2C (que antes travava o motor)
        if (distanceSensor.checkForDataReady()) {
            float dist_meters = distanceSensor.getDistance() / 1000.0;
            distanceSensor.clearInterrupt(); 

            if (current_reading_index < NUM_READINGS) {
                msg_scan.ranges.data[current_reading_index] = dist_meters;
                current_reading_index++;
            } else {
                // Publicar scan...
                int64_t time = rmw_uros_epoch_synchronized();
                msg_scan.header.stamp.sec = time / 1000000000;
                msg_scan.header.stamp.nanosec = time % 1000000000;
                msg_scan.ranges.size = NUM_READINGS;
                rcl_publish(&pub_lidar, &msg_scan, NULL);
                current_reading_index = 0;
            }
        }
    #endif
    delay(10);

}