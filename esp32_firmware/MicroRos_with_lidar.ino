// --- 1. INCLUSÕES DE BIBLIOTECAS ---
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

// --- Drivers de Hardware e Sensores ---
#include "driver/mcpwm.h"         // Para o motor de passo
#include <Wire.h>                 // Para I2C
#include <SparkFun_VL53L1X.h>     // <<< BIBLIOTECA CORRETA para o VL53L1X

// --- 2. DEFINIÇÕES DE HARDWARE ---
// Pinos do motor (movidos para 19 e 18 para liberar I2C)
#define PUL_PIN 19 
#define DIR_PIN 18 
#define LED_PIN 2

// --- Parâmetros do Motor de Passo ---
const int STEPS_PER_REV = 200;
const int MICROSTEPS = 16;
const int PULSES_PER_REV = STEPS_PER_REV * MICROSTEPS;

// --- 3. LÓGICA DE ERRO (COM DEBUG) ---
void error_loop(){ 
    Serial.println("!!! ERRO CRÍTICO - Entrando no loop de erro. Verifique a etapa anterior no log. !!!");
    while(1){ 
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
        delay(100); 
    }
}

// Macros de Checagem
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.printf("Falha na linha %d: %s\n", __LINE__, #fn); error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.printf("Falha suave na linha %d: %s\n", __LINE__, #fn);}}

// --- 4. VARIÁVEIS GLOBAIS ---
// micro-ROS
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Motor Subscriber
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg; // Variável para guardar a mensagem recebida

// Status Publisher
rcl_publisher_t publisher;
std_msgs__msg__Int32 pub_msg;
rcl_timer_t status_timer; 

// Sensor (LiDAR) Publisher
rcl_publisher_t lidar_publisher;
rcl_timer_t lidar_timer;
sensor_msgs__msg__LaserScan scan_msg;
SFEVL53L1X lox(Wire); // <<< Objeto do sensor CORRETO (SparkFun)

// Debug
unsigned long last_ping_time = 0;

// --- 5. LÓGICA DO MOTOR ---
void motor_setup() {
    pinMode(DIR_PIN, OUTPUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PUL_PIN); 
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;
    pwm_config.cmpr_a = 50.0;
    pwm_config.cmpr_b = 50.0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
}

// --- 6. CALLBACKS DO ROS ---

// Callback: Recebe comando do motor
void subscription_callback(const void *msgin) {
    Serial.println("[Callback] Comando de motor recebido!");
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    float target_speed_rad_s = msg->angular.z;

    if (target_speed_rad_s >= 0) {
        digitalWrite(DIR_PIN, HIGH);
    } else {
        digitalWrite(DIR_PIN, LOW);
    }

    float speed_rev_s = abs(target_speed_rad_s) / (2 * PI);
    uint32_t frequency_hz = (uint32_t)(speed_rev_s * PULSES_PER_REV);
    
    if (frequency_hz > 0) {
        mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, frequency_hz);
        mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    } else {
        mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    }
}

// Callback: Publica status do nó
void status_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
        pub_msg.data++;
    }
}

// Callback: Lê o sensor VL53L1X e publica
void lidar_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        
        // --- Lógica de leitura do VL53L1X (SparkFun) ---
        lox.startRanging();
        while (!lox.checkForDataReady()) {
            delay(1);
        }
        float range_mm = (float)lox.getDistance();
        lox.clearInterrupt();
        
        float range_meters = range_mm / 1000.0;
        
        if (range_meters == 0.0 || range_meters > 4.0) { // L1X tem alcance de 4m
             range_meters = std::numeric_limits<float>::infinity();
        }
        // -----------------------------------------------

        // Preenche o timestamp
        int64_t RMW_UROS_EPOCH_NANOS = rmw_uros_epoch_synchronized();
        scan_msg.header.stamp.sec = RMW_UROS_EPOCH_NANOS / 1000000000;
        scan_msg.header.stamp.nanosec = RMW_UROS_EPOCH_NANOS % 1000000000;
        
        // Preenche o frame_id (com cast para (char*))
        scan_msg.header.frame_id.data = (char*)"laser_link";
        scan_msg.header.frame_id.size = strlen(scan_msg.header.frame_id.data);
        
        // Define os parâmetros do "scan de 1 ponto"
        scan_msg.angle_min = 0.0;
        scan_msg.angle_max = 0.0;
        scan_msg.angle_increment = 1.0;
        scan_msg.scan_time = 0.1;
        scan_msg.range_min = 0.03;
        scan_msg.range_max = 4.0; // Alcance do L1X

        // Aloca memória para o array de leituras (se for a primeira vez)
        if (scan_msg.ranges.data == NULL) {
             scan_msg.ranges.data = (float*)malloc(sizeof(float) * 1);
             scan_msg.ranges.capacity = 1;
        }
        scan_msg.ranges.size = 1;
        scan_msg.ranges.data[0] = range_meters; // Coloca a leitura no array

        RCSOFTCHECK(rcl_publish(&lidar_publisher, &scan_msg, NULL));
    }
}

// --- 7. SETUP PRINCIPAL ---
void setup() {
    Serial.begin(115200);
    Serial.println("--- INICIANDO SETUP ---");

    pinMode(LED_PIN, OUTPUT);
    
    Serial.println("Iniciando I2C (Wire)...");
    Wire.begin(); 
    
    Serial.println("Iniciando sensor VL53L1X...");
    // Tenta inicializar o sensor VL53L1X
    if (lox.begin() != 0) { // 0 = sucesso para a biblioteca SparkFun
        Serial.println(">>> FALHA AO INICIAR SENSOR VL53L1X <<<");
        error_loop(); 
    }
    Serial.println("Sensor VL53L1X iniciado com sucesso.");
    lox.setDistanceModeShort(); // Modo curto é mais rápido

    Serial.println("Configurando transporte Wi-Fi...");
    //set_microros_wifi_transports((char*)"Quarto", (char*)"151400fbms@A", (char*)"192.168.1.116", 8888);
     set_microros_transports(); // Descomente esta linha se for usar cabo USB
    Serial.println("Transporte Wi-Fi configurado. Aguardando conexão...");
    
    delay(2000);

    Serial.println("Iniciando motor_setup()...");
    motor_setup(); 
    Serial.println("Motor setup concluído.");

    Serial.println("Iniciando alocador micro-ROS...");
    allocator = rcl_get_default_allocator();
    Serial.println("Iniciando suporte rclc...");
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    Serial.println("Iniciando nó micro-ROS...");
    RCCHECK(rclc_node_init_default(&node, "esp32_stepper_and_lidar_node", "", &support));
    Serial.println("Nó criado com sucesso.");

    Serial.println("Criando publisher de status...");
    RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "esp32_status"));
    Serial.println("Criando publisher do LiDAR...");
    RCCHECK(rclc_publisher_init_default(&lidar_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan"));
    
    Serial.println("Criando timer de status...");
    const unsigned int status_timer_period = 1000; 
    RCCHECK(rclc_timer_init_default(&status_timer, &support, RCL_MS_TO_NS(status_timer_period), status_timer_callback));
    
    Serial.println("Criando timer do LiDAR...");
    const unsigned int lidar_timer_period = 100;
    RCCHECK(rclc_timer_init_default(&lidar_timer, &support, RCL_MS_TO_NS(lidar_timer_period), lidar_timer_callback));
    
    Serial.println("Criando subscriber do motor...");
    RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    Serial.println("Entidades ROS criadas.");

    Serial.println("Iniciando o executor (3 tarefas)...");
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &status_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &lidar_timer)); 
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA));
    Serial.println("Executor configurado.");
    
    pub_msg.data = 0;
    scan_msg.ranges.data = NULL; // Importante
    
    Serial.println("--- SETUP CONCLUÍDO ---");
}

// --- 8. LOOP PRINCIPAL ---
void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

    if (millis() - last_ping_time > 2000) { 
        int ping_result = rmw_uros_ping_agent(100, 1); 
        
        if (ping_result != RMW_RET_OK) {
            Serial.println("Ping no Agente: FALHA.");
        } else {
            Serial.println("Ping no Agente: SUCESSO. Conexão ativa.");
        }
        last_ping_time = millis();
    }
}