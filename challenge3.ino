#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

// Pines 
const int pin_in1 = 18;
const int pin_in2 = 15;
const int pin_pwm = 4;

// Configuración PWM ESP32
const int pwmFreq = 980;        // Hz
const int pwmResolution = 8;    // 0–255

// Estructuras micro-ROS
rcl_subscription_t subscriber;
std_msgs_msg_Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Macros de verificación de errores
#define RCCHECK(fn) { rcl_ret_t rc = fn; if(rc != RCL_RET_OK){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; if(rc != RCL_RET_OK){}}

void error_loop(){
  while(1){ delay(100); }
}

// Callback: recibe referencia de velocidad en rango [-1, 1]
void subscription_callback(const void * msgin) {
  const std_msgs_msg_Float32 * incoming_msg =
    (const std_msgs_msg_Float32 *)msgin;

  float cmd = incoming_msg->data;

  // Saturación de seguridad
  if (cmd > 1.0) cmd = 1.0;
  if (cmd < -1.0) cmd = -1.0;

  // Control de dirección
  if (cmd > 0) {
    digitalWrite(pin_in1, HIGH);
    digitalWrite(pin_in2, LOW);
  } 
  else if (cmd < 0) {
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, HIGH);
  } 
  else {
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, LOW);
  }

  // Conversión a duty cycle (0–255)
  int dutyCycle = (int)(abs(cmd) * 255.0);

  // Aplicar PWM al motor
  ledcWrite(pin_pwm, dutyCycle);
}

void setup() {

  // Configuración de pines de dirección
  pinMode(pin_in1, OUTPUT);
  pinMode(pin_in2, OUTPUT);

  // Inicialización PWM en el pin seleccionado
  ledcAttach(pin_pwm, pwmFreq, pwmResolution);

  // Inicialización transporte micro-ROS
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Inicialización del entorno micro-ROS
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor", "", &support));

  // Suscripción al tópico de referencia
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "cmd_pwm"));

  // Configuración del ejecutor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &msg,
    &subscription_callback, ON_NEW_DATA));
}

void loop() {
  // Procesa eventos de micro-ROS
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(15);
}