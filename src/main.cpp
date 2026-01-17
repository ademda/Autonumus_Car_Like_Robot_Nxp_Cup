//THIS CODE IS FOR THE TUNING OF THE ROBOT: THE COMMANDS WILL BE SENT FROM THE PYTHON INTERFACE 
//THE OTHER BRANCH: ADEM_BRANCH HAS THE CODE THAT IS THE CODE FOR THE COMPETITION 
//THIS CODE CONTAINS : ASSERVISSEMENT PAR ROUE
#include <Arduino.h>
#include <Servo.h>
#include <TimerOne.h>
#include <QuadEncoder.h>
#include <math.h> 

#define UART_TX 1
#define UART_RX 0

#define LEFT_ENC_CH1 3
#define LEFT_ENC_CH2 2

#define RIGHT_ENC_CH1 5
#define RIGHT_ENC_CH2 4

#define RIGHTMOTOR_FWD_PWM 23 //Forward PWM
#define RIGHTMOTOR_BWD_PWM 22 //Backward PWM

#define LEFTMOTOR_FWD_PWM 6 //Forward PWM
#define LEFTMOTOR_BWD_PWM 7 //Backward PWM

#define TIM1_PIN 8
#define TIM3_PIN 9

#define SERVO_PIN 17  

#define SPI_CS_PIN 10   //ya turki badel pinet spi ll camera teensy 4.0
#define SPI_MOSI_PIN 11  
#define SPI_MISO_PIN 12
#define SPI_SCLK_PIN 13

#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19

/***************** CONTROLLER DEFINES ****************** */
//PID DEFINES
#define RIGHT_VEL_KP 0.25 //0.1
#define RIGHT_VEL_KI 0.007 //0.001
#define RIGHT_VEL_KD 0.0 //0.0

#define LEFT_VEL_KP 0.025
#define LEFT_VEL_KI 0.007
#define LEFT_VEL_KD 0.0

#define STEERING_KP 1.0
#define STEERING_KI 0.0
#define STEERING_KD 0.0

#define MAX_SERVO_ANGLE 150
#define MIN_SERVO_ANGLE 30
#define MAX_MOTOR_CMD 255
#define MIN_MOTOR_CMD 0
#define MAX_STEERING_ERROR_SUM 120  // Prevent integral windup
#define MAX_VEL_ERROR_SUM 35000

#define WHEEL_GAIN  1.000
#define CONTROL_LOOP_DT_MS 5  // 5ms = 0.005 seconds (200Hz control loop from Timer1)
#define VELOCITY_CALC_DT_MS 5
/************ SAMSON DEFINES ************* */
#define PATH_SIZE 1000
#define SAMSON_K1 0.5
#define SAMSON_K2 0.3
#define SAMSON_K3 0.4
/****************  ODOMETRY DEFINES *************** */
#define LEFT_ENCODER_CPR 408
#define RIGHT_ENCODER_CPR 408
#define LEFT_WHEEL_DIAMETER_MM 67.58 //arbitrary number //65 //91.77 //72.19
#define RIGHT_WHEEL_DIAMETER_MM 67.58 //65 //99.32
#define WHEEL_BASE_MM 150 //distance between wheels
#define ROBOT_LENGTH_MM 200
#define SERVO_INIT_ANGLE 5
/**************** ROBOT INIT POSITION  ************* */

#define ROBOT_INIT_X_MM 0.0
#define ROBOT_INIT_Y_MM 0.0
#define ROBOT_INIT_ANGLE_RAD 90* (M_PI / 180.0)

/********************** ODOMETRY VARIABLES *********************** */
volatile double left_wheel_curr_vel_mm_s, right_wheel_curr_vel_mm_s, robot_curr_vel_mm_s;
volatile double prev_left_wheel_distance_mm, prev_right_wheel_distance_mm, prev_robot_dist_mm;
volatile double left_wheel_distance_mm, right_wheel_distance_mm, robot_distance_mm;
volatile double left_vel_filtered, right_vel_filtered;

volatile double curr_orientation_deg, prev_orientation_deg;
volatile double curr_orientation_rad, prev_orientation_rad;
volatile double robot_angular_velocity_deg, prev_robot_angular_velocity_deg;
volatile double robot_angular_velocity_rad_s, prev_robot_angular_velocity_rad_s;
volatile double robot_x_mm, robot_y_mm, robot_global_orientation_deg, robot_global_orientation_rad;
volatile double prev_robot_x_mm, prev_robot_y_mm, prev_robot_global_orientation_deg, prev_robot_global_orientation_rad;
volatile double robot_curve_coef ;

volatile double left_wheel_dist_prev_vel_calc = 0;
volatile double right_wheel_dist_prev_vel_calc = 0;
volatile uint32_t last_vel_calc_ms = 0;
/************************ PID VARIABLES ***************  */
volatile float right_vel_kp = RIGHT_VEL_KP, right_vel_ki = RIGHT_VEL_KI, right_vel_kd = RIGHT_VEL_KD;
volatile float left_vel_kp = LEFT_VEL_KP, left_vel_ki = LEFT_VEL_KI, left_vel_kd = LEFT_VEL_KD;
volatile float steering_kp = STEERING_KP, steering_ki = STEERING_KI, steering_kd = STEERING_KD;

/**************** ACTUATORS VARIABLES ************ */
volatile int32_t right_motor_cmd, left_motor_cmd;
volatile int16_t servo_angle_cmd_deg; //in deg

/**************** SENSORS VARIABLES *************** */
volatile int32_t left_ticks_i32, right_ticks_i32;

/**************** CONTROL VARIABLES ********* */
//VELOCITY CONTROL
volatile double left_motor_vel_setpoint_mm_s, right_motor_vel_setpoint_mm_s, robot_vel_setpoint_mm_s; //desired velocity to reach
volatile double prev_left_motor_vel_setpoint_mm_s, prev_right_motor_vel_setpoint_mm_s; // Track previous setpoints
volatile double left_motor_vel_error_mm_s, right_motor_vel_error_mm_s, robot_vel_error_mm_s;

volatile double left_motor_vel_pid_output, right_motor_vel_pid_output;
volatile double left_motor_vel_error_sum_mm_s, right_motor_vel_error_sum_mm_s;  
volatile double left_motor_vel_last_error_mm_s, right_motor_vel_last_error_mm_s;  
  
//STEERING CONTROL
volatile float orientation_setpoint_deg, orientation_error_deg, orientation_error_sum_deg;
volatile float orientation_last_error_deg;
volatile float servo_angle_pid_output;
//DISTANCE  CONTROL
volatile float distance_setpoint_mm, prev_robot_distance_mm;
volatile float distance_error_mm;
volatile bool distance_control_enable = false;
volatile bool distance_reached = false;
//EMERGENCY STOP
volatile bool emergency_stop_enable = false;

/********************* SAMSON SPECIFIC VARIABLES ***************************** */
typedef struct{
  float path_x_mm;
  float path_y_mm;
  float path_theta_rad;
} path_point_cartesien;

typedef struct{
  float path_distance_mm;
  float path_orientation_rad;
  float path_theta_rad;
} path_point_polar;

path_point_cartesien path_cart[PATH_SIZE];
path_point_polar path_polar[PATH_SIZE];

volatile float path_dx_mm, path_dy_mm, path_dtheta_rad;
volatile float path_linear_vel_mm_s, path_angular_vel_rad_s;
volatile float robot_angular_vel_setpoint_rad_s;
volatile float robot_x_error_mm, robot_y_error_mm, robot_theta_error_rad;
volatile float samson_x_error_mm, samson_y_error_mm, samson_theta_error_rad;
volatile float robot_distance_error_mm, robot_orientation_error_rad;
volatile uint32_t path_index;
volatile float samson_k1 = SAMSON_K1, samson_k2 = SAMSON_K2, samson_k3 = SAMSON_K3;
/********* INSTANCES ********** */
QuadEncoder left_encoder(1, LEFT_ENC_CH1, LEFT_ENC_CH2);
QuadEncoder right_encoder(2, RIGHT_ENC_CH1, RIGHT_ENC_CH2);

Servo  steer_servo;

/*********** DEBUG VARIABLES ******** */
uint32_t last_debug = 0;

/************** FUNCTIONS DECLARATIONS  ********* */
void ReadEncoders();
void RotateMotors();
void SetServoAngle();
void ConvertTicksToDistance();
void ConvertDistanceToVel();
void CalcutateRobotCoordinates();
void CalculateVelError();
void CalculateVelPID();
void CalculateOrientationError();
void CalculateSteeringPID();

void VelControllerRoutine();
void GetOrientation(); // using encoders for now until camera code comes
void VelOdomRoutine();
void CalculateRobotCoordinates();

void parseTuningValues(String data);
void checkUARTForPID();
void SendStatusToESP32();

void CalculateDistanceError();//used for tuning only
void StopMotors();

void CalculateSamson();
void CalculatePathVel();
void CalcuateTrajectoryError();
void CalculateSteeringVelCommands();
float wrapDeg180(float angle_deg);
float wrapRadPI(float angle_rad);
void EmptyFunction(){

}

void NavRoutine(){
  VelOdomRoutine();
  GetOrientation();
  CalculateRobotCoordinates();
  // if (emergency_stop_enable==false && distance_reached == false){
  //   CalculateSamson();
  //   VelControllerRoutine();
  //   if (distance_control_enable){
  //     CalculateDistanceError();
  //     if (distance_error_mm <= 3.0 || distance_reached == true){
  //       StopMotors();
  //       distance_reached = true;
  //     }
  //     else if (distance_reached == false) {
  //       RotateMotors();
  //     }
  //   }
  //   else {
  //     RotateMotors();
  //   }
  //   /*CalculateOrientationError();
  //   CalculateSteeringPID();*/
  //   SetServoAngle();
  // }
  // else {
  //   StopMotors();
  // }
}

void VelOdomRoutine(){
  ReadEncoders();
  ConvertTicksToDistance();
  ConvertDistanceToVel();
}

void VelControllerRoutine(){
  CalculateVelError();
  CalculateVelPID();
}

void setup() {
  /****************  ENCODERS INIT *************** */
  // Initialize left encoder
  left_encoder.setInitConfig();
  left_encoder.init();

  // Initialize right encoder
  right_encoder.setInitConfig();
  right_encoder.init();

  left_encoder.write(0);
  right_encoder.write(0);

  /**************** TIMERS INIT *************** */
  Timer1.initialize(CONTROL_LOOP_DT_MS*1000);          // set period in µs //5000
  Timer1.attachInterrupt(NavRoutine);  // attach the interrupt function

  /****************  SERVO INIT *************** */
  steer_servo.attach(SERVO_PIN);
  steer_servo.write(SERVO_INIT_ANGLE);
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(4000);
  left_motor_vel_setpoint_mm_s = 0;
  right_motor_vel_setpoint_mm_s = 0;
  
  robot_global_orientation_rad = ROBOT_INIT_ANGLE_RAD;
  robot_x_mm = ROBOT_INIT_X_MM;
  robot_y_mm = ROBOT_INIT_Y_MM;
}

void loop() {
  // Check for commands from ESP32 via Serial1
  checkUARTForPID();
  
  // Send status to ESP32 every 100ms
  static uint32_t last_status_send = 0;
  if (millis() - last_status_send > 10) {
    SendStatusToESP32();
    last_status_send = millis();
  }

  // Manual setpoint input from Serial Monitor for testing

  if (millis() - last_debug > 10) {
    // Teleplot format: >variable_name:value
    // Serial.print(">enc right:");
    // Serial.println(right_ticks_i32);
    // Serial.print(">enc left:");
    // Serial.println(left_ticks_i32);

    // Serial.print(">left distance:");
    // Serial.println(left_wheel_distance_mm);
    // Serial.print(">right distance:");
    // Serial.println(right_wheel_distance_mm);

    Serial.print(">curr_orientation_deg:");
    Serial.println(wrapDeg180(curr_orientation_deg));
    
    Serial.print(">robot_x_mm:");
    Serial.println(robot_x_mm);

    Serial.print(">robot_y_mm:");
    Serial.println(robot_y_mm);

    Serial.print(">robot_global_orientation_deg:");
    Serial.println(wrapDeg180(robot_global_orientation_deg));
    
    Serial.print(">left_cmd:");
    Serial.println(left_motor_cmd);


    //Serial.print("distance error");Serial.println(distance_error_mm);
    last_debug = millis();
    //delay(100);
  }
}

/****************  BASIC FUNCTIONS *************** */
void ReadEncoders(){
  left_ticks_i32 = left_encoder.read();
  right_ticks_i32 = right_encoder.read();
} 

void RotateMotors(){
  uint8_t right_cmd =(uint8_t)(constrain(abs(right_motor_cmd), MIN_MOTOR_CMD, MAX_MOTOR_CMD));
  uint8_t left_cmd = (uint8_t)(constrain(abs(left_motor_cmd), MIN_MOTOR_CMD, MAX_MOTOR_CMD));
  if (right_motor_cmd>=0){
    analogWrite(RIGHTMOTOR_FWD_PWM, right_cmd);
    analogWrite(RIGHTMOTOR_BWD_PWM, 0);
  } 
  else {
    analogWrite(RIGHTMOTOR_FWD_PWM, 0);
    analogWrite(RIGHTMOTOR_BWD_PWM, right_cmd);
  }
  if (left_motor_cmd>=0){
    analogWrite(LEFTMOTOR_FWD_PWM, left_cmd);
    analogWrite(LEFTMOTOR_BWD_PWM, 0);
  }
  else {
    analogWrite(LEFTMOTOR_FWD_PWM, 0);
    analogWrite(LEFTMOTOR_BWD_PWM, left_cmd);
  }
}

void SetServoAngle(){
  steer_servo.write(servo_angle_cmd_deg);
}

void StopMotors(){
  analogWrite(RIGHTMOTOR_FWD_PWM, 0);
  analogWrite(RIGHTMOTOR_BWD_PWM, 0);
  analogWrite(LEFTMOTOR_FWD_PWM, 0);
  analogWrite(LEFTMOTOR_BWD_PWM, 0);
}

/****************  ODOMETRY FUNCTIONS *************** */

void ConvertTicksToDistance(){
  //update values
  left_wheel_distance_mm = (left_ticks_i32 * M_PI * LEFT_WHEEL_DIAMETER_MM) / LEFT_ENCODER_CPR;
  right_wheel_distance_mm = (right_ticks_i32 * M_PI * RIGHT_WHEEL_DIAMETER_MM) / RIGHT_ENCODER_CPR ;
  robot_distance_mm = (left_wheel_distance_mm + right_wheel_distance_mm)/2.0;
}

void ConvertDistanceToVel(){
  double raw_left_vel = 1000*(left_wheel_distance_mm - left_wheel_dist_prev_vel_calc)/(VELOCITY_CALC_DT_MS);
  double raw_right_vel = 1000*(right_wheel_distance_mm - right_wheel_dist_prev_vel_calc)/(VELOCITY_CALC_DT_MS);
  
  left_vel_filtered = (left_vel_filtered * 0.8) + (raw_left_vel * 0.2);
  right_vel_filtered = (right_vel_filtered * 0.8) + (raw_right_vel * 0.2);

  left_wheel_curr_vel_mm_s = left_vel_filtered;
  right_wheel_curr_vel_mm_s = right_vel_filtered;
  
  left_wheel_dist_prev_vel_calc = left_wheel_distance_mm;
  right_wheel_dist_prev_vel_calc = right_wheel_distance_mm;
  
  robot_curr_vel_mm_s = (right_wheel_curr_vel_mm_s + left_wheel_curr_vel_mm_s) / 2.0;
}

void GetOrientation(){
  //get orientation from camera 
  prev_orientation_rad = curr_orientation_rad;
  curr_orientation_rad = ((right_wheel_distance_mm - left_wheel_distance_mm) / WHEEL_BASE_MM);
  robot_angular_velocity_rad_s = 1000*(curr_orientation_rad - prev_orientation_rad) /(VELOCITY_CALC_DT_MS);
  curr_orientation_deg = curr_orientation_rad*180/M_PI;
}

void CalculateRobotCoordinates(){
  double dL = left_wheel_distance_mm  - prev_left_wheel_distance_mm;
  double dR = right_wheel_distance_mm - prev_right_wheel_distance_mm;

  prev_left_wheel_distance_mm = left_wheel_distance_mm;
  prev_right_wheel_distance_mm = right_wheel_distance_mm;

  double ds = (dR + dL) * 0.5;
  double dtheta = (dR - dL) / WHEEL_BASE_MM;
  double theta_mid = robot_global_orientation_rad + dtheta * 0.5;
  
  double curve_coef;
  if (fabs(dtheta) < 1e-6){
      curve_coef = 1.0;
  }  
  else{
    curve_coef = sin(dtheta * 0.5) / (dtheta * 0.5);
  }
  robot_x_mm += ds * curve_coef * cos(theta_mid);
  robot_y_mm += ds * curve_coef * sin(theta_mid);

  robot_global_orientation_rad += dtheta;
  robot_global_orientation_rad = robot_global_orientation_rad;

  robot_global_orientation_deg = robot_global_orientation_rad * 180.0 / M_PI;
}


/****************  CONTROLLER FUNCTIONS *************** */

void CalculateDistanceError(){//used for tuning only
  distance_error_mm = distance_setpoint_mm - robot_distance_mm; 
}

void CalculateVelError(){
  //store previous values
  left_motor_vel_last_error_mm_s = left_motor_vel_error_mm_s;
  right_motor_vel_last_error_mm_s  = right_motor_vel_error_mm_s;
  //update error
  left_motor_vel_error_mm_s = left_motor_vel_setpoint_mm_s - left_wheel_curr_vel_mm_s;
  right_motor_vel_error_mm_s = right_motor_vel_setpoint_mm_s - right_wheel_curr_vel_mm_s;
  robot_vel_error_mm_s = robot_vel_setpoint_mm_s - robot_curr_vel_mm_s;
}

void CalculateVelPID(){
  
  // Accumulate integral
  left_motor_vel_error_sum_mm_s += left_motor_vel_error_mm_s;
  right_motor_vel_error_sum_mm_s += right_motor_vel_error_mm_s;
  
  // Enable integral windup protection to prevent unbounded growth
  left_motor_vel_error_sum_mm_s = constrain(left_motor_vel_error_sum_mm_s, -MAX_VEL_ERROR_SUM, MAX_VEL_ERROR_SUM);
  right_motor_vel_error_sum_mm_s = constrain(right_motor_vel_error_sum_mm_s, -MAX_VEL_ERROR_SUM, MAX_VEL_ERROR_SUM);
  
  // FIXED: Divide derivative by dt
  float left_motor_vel_error_sub_mm_s = (left_motor_vel_error_mm_s - left_motor_vel_last_error_mm_s) ;
  float right_motor_vel_error_sub_mm_s = (right_motor_vel_error_mm_s - right_motor_vel_last_error_mm_s) ;
  
  // Calculate PID output
  left_motor_vel_pid_output = (left_motor_vel_error_mm_s * left_vel_kp) +
                              (left_motor_vel_error_sum_mm_s * left_vel_ki) +
                              (left_motor_vel_error_sub_mm_s * left_vel_kd);

  right_motor_vel_pid_output = (right_motor_vel_error_mm_s * right_vel_kp) +
                               (right_motor_vel_error_sum_mm_s * right_vel_ki) +
                               (right_motor_vel_error_sub_mm_s * right_vel_kd);    
                              
  // FIXED: Add output saturation to prevent PWM overflow
  right_motor_cmd = constrain(right_motor_vel_pid_output, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
  left_motor_cmd = constrain(left_motor_vel_pid_output, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);                          
} 

void CalculateOrientationError(){
  //store prev error
  orientation_last_error_deg = orientation_error_deg;
  //update error
  orientation_error_deg = orientation_setpoint_deg - curr_orientation_deg;
}

void CalculateSteeringPID(){
  orientation_error_sum_deg = constrain(orientation_error_sum_deg + orientation_error_deg, 
                                       -MAX_STEERING_ERROR_SUM, MAX_STEERING_ERROR_SUM);
  float orientation_error_sub_deg = orientation_error_deg - orientation_last_error_deg;
  servo_angle_pid_output = (orientation_error_deg*steering_kp)+
                           (orientation_error_sum_deg*steering_ki)+
                           (orientation_error_sub_deg*steering_kd);
  
  //servo cmd = pid_output (+ constraint)
  servo_angle_cmd_deg = constrain(servo_angle_pid_output, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
}

/******************************* Pathfollowing Controller Functions ***************************** */

void CalculateSamson(){
  CalculatePathVel();
  CalcuateTrajectoryError();
  CalculateSteeringVelCommands();
}

void CalculatePathVel(){
  path_dx_mm = (path_cart[path_index+1].path_x_mm) - (path_cart[path_index -1].path_x_mm);
  path_dy_mm = (path_cart[path_index+1].path_y_mm) - (path_cart[path_index -1].path_y_mm);
  path_linear_vel_mm_s = 1000*sqrt(path_dx_mm * path_dx_mm + path_dy_mm * path_dy_mm)/VELOCITY_CALC_DT_MS;
  path_angular_vel_rad_s = atan2(path_dy_mm , path_dx_mm);
}
void CalcuateTrajectoryError(){
  robot_x_error_mm = path_cart[path_index].path_x_mm - robot_x_mm;
  robot_y_error_mm = path_cart[path_index].path_y_mm - robot_y_mm;
  robot_theta_error_rad = path_cart[path_index].path_theta_rad - curr_orientation_rad;
  samson_x_error_mm = cos(curr_orientation_rad)*robot_x_error_mm + sin(curr_orientation_rad)*robot_y_error_mm;
  samson_y_error_mm = -sin(curr_orientation_rad)*robot_x_error_mm + cos(curr_orientation_rad)*robot_y_error_mm;
  samson_theta_error_rad = wrapRadPI(robot_theta_error_rad);
}
void CalculateSteeringVelCommands(){
  left_motor_vel_setpoint_mm_s = path_linear_vel_mm_s*cos(samson_theta_error_rad) + samson_k1*samson_x_error_mm;
  right_motor_vel_setpoint_mm_s = left_motor_vel_setpoint_mm_s;
  robot_angular_vel_setpoint_rad_s = path_angular_vel_rad_s + samson_k2*path_linear_vel_mm_s*samson_y_error_mm + samson_k3*sin(samson_theta_error_rad); 
  servo_angle_cmd_deg = atan2(robot_angular_vel_setpoint_rad_s*ROBOT_LENGTH_MM,left_motor_vel_setpoint_mm_s) * 180 / M_PI;
}


float wrapDeg180(float angle_deg)
{
    angle_deg = fmod(angle_deg + 180.0f, 360.0f);
    if (angle_deg < 0)
        angle_deg += 360.0f;
    return angle_deg - 180.0f;
}

float wrapRadPI(float angle_rad)
{
    angle_rad = fmod(angle_rad + M_PI, 2*M_PI);
    if (angle_rad < 0)
        angle_rad += 2*M_PI;
    return angle_rad - M_PI;
}
/********************** Remote Tuning functions ***************************** */

void parseTuningValues(String data) {
  // Parse new format: "right_kp,right_ki,right_kd,left_kp,left_ki,left_kd,steer_kp,steer_ki,steer_kd,right_velocity,left_velocity,distance,emergency_stop,distance_mode"
  int commas[13];
  int index = 0;
  
  // Find all 13 comma positions (14 values)
  for (unsigned int i = 0; i < data.length() && index < 13; i++) {
    if (data[i] == ',') {
      commas[index] = i;
      index++;
    }
  }
  
  if (index >= 13) {
    // Parse RIGHT motor PID
    prev_robot_distance_mm = robot_distance_mm;
    right_vel_kp = data.substring(0, commas[0]).toFloat();
    right_vel_ki = data.substring(commas[0]+1, commas[1]).toFloat();
    right_vel_kd = data.substring(commas[1]+1, commas[2]).toFloat();
    
    // Parse LEFT motor PID
    left_vel_kp = data.substring(commas[2]+1, commas[3]).toFloat();
    left_vel_ki = data.substring(commas[3]+1, commas[4]).toFloat();
    left_vel_kd = data.substring(commas[4]+1, commas[5]).toFloat();
    
    // Parse Steering PID
    steering_kp = data.substring(commas[5]+1, commas[6]).toFloat();
    steering_ki = data.substring(commas[6]+1, commas[7]).toFloat();
    steering_kd = data.substring(commas[7]+1, commas[8]).toFloat();
    
    // Parse velocity setpoints and distance
    right_motor_vel_setpoint_mm_s = data.substring(commas[8]+1, commas[9]).toFloat();
    left_motor_vel_setpoint_mm_s = data.substring(commas[9]+1, commas[10]).toFloat();
    distance_setpoint_mm = (data.substring(commas[10]+1, commas[11]).toFloat()) + prev_robot_distance_mm;
    
    // Parse control flags
    int emergency = data.substring(commas[11]+1, commas[12]).toInt();
    emergency_stop_enable = (emergency == 1);
    
    int dist_mode = data.substring(commas[12]+1).toInt();
    distance_control_enable = (dist_mode == 1);
    
    Serial.println("PID Updated:");
    Serial.print("Right: Kp="); Serial.print(right_vel_kp,5);
    Serial.print(" Ki="); Serial.print(right_vel_ki,5);
    Serial.print(" Kd="); Serial.println(right_vel_kd,5);
    Serial.print("Left: Kp="); Serial.print(left_vel_kp,5);
    Serial.print(" Ki="); Serial.print(left_vel_ki,5);
    Serial.print(" Kd="); Serial.println(left_vel_kd,5);
    Serial.print("Distance mode: ");
    Serial.println(distance_control_enable ? "ENABLED" : "DISABLED");
    Serial.print("Distance: "); Serial.println(data.substring(commas[10]+1, commas[11]).toFloat(),3);
    distance_reached = false;
  }
}

void checkUARTForPID() {
  if (Serial1.available()) {
    String pidString = Serial1.readStringUntil('\n');
    parseTuningValues(pidString);
  }
}

void SendStatusToESP32() {
  // Send current robot status: "robot_distance,left_velocity,right_velocity,left_distance,right_distance"
  String statusData = String(right_motor_vel_error_sum_mm_s) + "," +
                     String(right_motor_cmd) + "," +
                     String(left_wheel_curr_vel_mm_s, 2) + "," +
                     String(right_wheel_curr_vel_mm_s, 2) + "," +
                     String(right_motor_vel_setpoint_mm_s, 2) + "\n";
                    
  Serial1.print(statusData);
   
}