// Flywheel
#include <Servo.h>
#include <PID_v2.h>
// IMU
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
// Gimbal
#include <Dynamixel2Arduino.h>
#include <SoftwareSerial.h>
//Calculation
#include <BasicLinearAlgebra.h>
using namespace BLA;
//Filter
#include <movingAvgFloat.h>

// =====================================================================================================
// GIMBAL MOTOR SETUP
// =====================================================================================================
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial
const float DXL_PROTOCOL_VERSION = 2.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// Gimbal 1
const uint8_t DXL_ID_1 = 0;
float Gim_vel_1;
float Gim_pos_1;
float Gim_pos_1_conv1;
float Gim_pos_1_conv;
float Gim_com_1 = 0;
float Gim_offset_1 = 45;// Degrees
float cmd_gim_1 = 270; // Degrees (Homing)
//float cmd_gim_1 = 180;// Degrees (Homing)
float Gim_vel_thresh = 70;//RPM
float Kp_gimbal_1 = 0.01;
float Kd_gimbal_1 = 0.001;
float Ki_gimbal_1 = 0;
PID_v2 PID_gim_1(Kp_gimbal_1, Ki_gimbal_1, Kd_gimbal_1, PID::Direct);

// Gimbal 2
const uint8_t DXL_ID_2 = 1;
float Gim_vel_2;
float Gim_pos_2;
float Gim_pos_2_conv1;
float Gim_pos_2_conv;
float Gim_com_2 = 0;
float Gim_offset_2 = Gim_offset_1;// Degrees
float cmd_gim_2 = 0;//Degrees (Homing)
//float cmd_gim_2 = 0;//Degrees (Homing)
float Kp_gimbal_2 = 0.01;
float Kd_gimbal_2 = 0.001;
float Ki_gimbal_2 = 0;
PID_v2 PID_gim_2(Kp_gimbal_2, Ki_gimbal_2, Kd_gimbal_2, PID::Direct);

// =====================================================================================================
// FLYWHEEL MOTOR SETUP
// =====================================================================================================
#define ESC_PIN_1 50
#define INTERRUPT_PIN_1 18
#define ESC_PIN_2 48
#define INTERRUPT_PIN_2 19
float numpoles = 14;              // Change value to the number of magnets in the motor.
int rpm_calc_delay = 75;          // ms

// Flywheel 1
Servo ESC1;                        // create servo object to control the ESC
float cmd_spd_1 = 3000.0;
float cur_spd_1;
float my_motor_rpm_1 = 0;
volatile int int_cnt_1;
double Kp_1 = 0.03, Ki_1 = 0.003, Kd_1 = 0.003;
//double Kp_1 = 0.009, Ki_1 = 0.001, Kd_1 = 0.001;
PID_v2 PID_CMG_1(Kp_1, Ki_1, Kd_1, PID::Direct);

// Flywheel 2
Servo ESC2;                        // create servo object to control the ESC
float cmd_spd_2 = cmd_spd_1;
float cur_spd_2;
float my_motor_rpm_2 = 0;
volatile int int_cnt_2;
double Kp_2 = 0.03, Ki_2 = 0.003, Kd_2 = 0.003;
//double Kp_2 = 0.009, Ki_2 = 0.001, Kd_2 = 0.001;
PID_v2 PID_CMG_2(Kp_2, Ki_2, Kd_2, PID::Direct);

//Calculation
float Ixx = 0.039304;//kgm2
float Iyy = 0.012433;//kgm2
float fac = Iyy/Ixx;
Matrix<2, 2> K_p = {0.0020, 0.00,   0.00, fac * K_p(0)};
Matrix<2, 2> K_d = {0.0040, 0.00,   0.00, fac * K_d(0)};
Matrix<2, 2> K_i = {0.0000, 0.00,   0.00, fac * K_i(0)};
Matrix<2, 1> EA_err = {0, 0};
Matrix<2, 1> W_err = {0, 0};
Matrix<2, 1> Add_err = {0, 0};
Matrix<2, 1> EA_act = {0, 0};
float x_offset = 16.25-0.5-1.5;
float y_offset = 01.31-0.2;
float z_offset = 0;
float x_act = 0;
float y_act = 0;
float z_act = 0;
Matrix<2, 1> W_act = {0, 0};
Matrix<2, 1> W_fil = {0, 0};
Matrix<2, 1> EA_com = {0, 0};
Matrix<2, 1> W_com = {0, 0};
Matrix<2, 1> Tau = {0, 0};
float I = 160329 * pow(10,-9);//kgm^2
float fly_speed = cmd_spd_2 * 2 * PI / 60; //rad/s
float h0 = I * fly_speed; //Nms
Matrix<2, 2> A = {0, 0, 0, 0};
double A_11, A_12, A_21, A_22;
Matrix<2, 2> A_inv = {0, 0, 0, 0};
Matrix<2, 2> A_term = {0, 0, 0, 0};
Matrix<2, 1> Gim_com = {0, 0};
Matrix<2, 1> N = {0, 0};
Matrix<2, 2> Idnt = {1, 0, 0, 1};
Matrix<2, 1> Gim_Rec = {0, 0};

double loop_start = 0, loop_end = 0, dt = 0 , tim = 0;
int loop_no = 0;
String input;
int var = 0;
float deg2rad = 3.141592 / 180.0; // deg to rad conversion
float rpm2rads = 3.141592 / 30.0;  // RPM to rad/s conversion
float rads2rpm = 1/rpm2rads;  // rad/s to RPM conversion
float rev_val = 1 * 3.141592, meas_ang_conv_res;
int meas_ang_temp, meas_ang_conv, divend_val, div_val = rev_val * 10000;

// Filter
movingAvgFloat ang_x(6);
movingAvgFloat ang_y(6);

// =====================================================================================================
// VOID SETUP
// =====================================================================================================

void setup() {
  //Serial begin
  Serial.begin(115200);

  //moving average begin
  ang_x.begin();
  ang_y.begin();
  
  //IMU
  Serial.println("Initializing IMU");
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  //DYNAMIXEL
  Serial.println("Initializing Dynamixel");
  dxl.begin(115200);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID_1);
  dxl.ping(DXL_ID_2);
  dxl.torqueOff(DXL_ID_1);
  dxl.torqueOff(DXL_ID_2);
  dxl.setOperatingMode(DXL_ID_1, OP_VELOCITY);
  dxl.setOperatingMode(DXL_ID_2, OP_VELOCITY);
  dxl.torqueOn(DXL_ID_1);
  dxl.torqueOn(DXL_ID_2);
  PID_gim_1.SetOutputLimits(-50, 50);
  PID_gim_2.SetOutputLimits(-50, 50);
  PID_gim_1.Start(0, 0, cmd_gim_1); // input current_output setpoint
  PID_gim_2.Start(0, 0, cmd_gim_2); // input current_output setpoint

  //ESC
  Serial.println("Initializing ESC");
  // Hall sensor configuration
  pinMode(INTERRUPT_PIN_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_1), interruptFired_1, CHANGE);
  pinMode(INTERRUPT_PIN_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_2), interruptFired_2, CHANGE);

  // ESC configuration
  ESC1.attach(ESC_PIN_1, 1000, 2000);
  ESC2.attach(ESC_PIN_2, 1000, 2000);

  // Initializing PID
  PID_CMG_1.Start(0, 0, cmd_spd_1); // input current output setpoint
  PID_CMG_2.Start(0, 0, cmd_spd_2); // input current output setpoint
  delay(10);

  // Initializing ESC
  ESC1.writeMicroseconds(1000);
  ESC2.writeMicroseconds(1000);
  delay(1000);
  ESC1.writeMicroseconds(1050);
  ESC2.writeMicroseconds(1050);
  delay(200);
}

// =====================================================================================================
// VOID LOOP
// =====================================================================================================

void loop()  {
  loop_start = millis();
  //.....................Flywheel command and feedback...................//
  // Flywheel speed feedback
  cur_spd_1 = meas_speed_1();
  cur_spd_2 = meas_speed_2();
  
  // Flywheel speed command
  const double ESC_Command_1 = PID_CMG_1.Run(cur_spd_1);
  const double ESC_Command_us_1 = map(ESC_Command_1, 0, 180, 1000, 2000);
  const double ESC_Command_ms1_constrain_1 = constrain(ESC_Command_us_1, 1000, 2000);
  ESC1.writeMicroseconds(ESC_Command_us_1);

  const double ESC_Command_2 = PID_CMG_2.Run(cur_spd_2);
  const double ESC_Command_us_2 = map(ESC_Command_2, 0, 180, 1000, 2000);
  const double ESC_Command_ms1_constrain_2 = constrain(ESC_Command_us_2, 1000, 2000);
  ESC2.writeMicroseconds(ESC_Command_us_2);
    
  //........................... Feedback...........................//

  // IMU feedback
  imu::Vector<3> Euler_ang = bno.getVector(Adafruit_BNO055::VECTOR_EULER);// Euler angles
  imu::Vector<3> Ang_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);// Angular rates
  x_act = -(Euler_ang.z()-x_offset);
  y_act = -(Euler_ang.y()-y_offset);
  EA_act = {x_act, y_act};
  W_act = {Ang_vel.x(), Ang_vel.y()};

  //Angular speed filter
  float avg_x = ang_x.reading(Ang_vel.x());
  float avg_y = ang_y.reading(Ang_vel.y());
  W_fil = {avg_x, avg_y};
  
  // Gimbal position feedback
  Gim_pos_1 = dxl.getPresentPosition(DXL_ID_1, UNIT_DEGREE);
  Gim_pos_1_conv1 = -1*(conv_meas_ang(Gim_pos_1)/deg2rad) ;//Degrees // -1 to reverse the gimbal direction
  Gim_pos_1_conv = Gim_pos_1_conv1 + Gim_offset_1;
  if (Gim_pos_1_conv1 >= 135){
    Gim_pos_1_conv = Gim_pos_1_conv1 - 315;
  }

  Gim_pos_2 = dxl.getPresentPosition(DXL_ID_2, UNIT_DEGREE);
  Gim_pos_2_conv1 = -1*(conv_meas_ang(Gim_pos_2)/deg2rad) ;//Degrees // -1 to reverse the gimbal direction
  Gim_pos_2_conv = Gim_pos_2_conv1 + Gim_offset_2;
  if (Gim_pos_2_conv1 >= 135){
    Gim_pos_2_conv = Gim_pos_2_conv1 - 315;
  }
  
  // Gimbal speed feedback
  Gim_vel_1 = dxl.getPresentVelocity(DXL_ID_1, UNIT_RPM);
  Gim_vel_2 = dxl.getPresentVelocity(DXL_ID_2, UNIT_RPM);

  //......................... Calculation.........................//
  // Error
  EA_err = EA_act - EA_com;//degrees
  W_err = W_fil - W_com;//rad/s
//  W_err = W_act - W_com;//rad/s
  Add_err = Add_err + EA_err;//degrees
  
  // PID Controller
  Tau   = - K_p * EA_err - K_d * W_err - K_i * Add_err;//Nm
  Tau(1) = 0;
  
  // Allocation Matrix
  A_11 = -h0 * cos(Gim_pos_1_conv*PI/180);
  A_12 = -h0 * cos(Gim_pos_2_conv*PI/180);
  A_21 = -h0 * sin(Gim_pos_1_conv*PI/180);
  A_22 = -h0 * sin(Gim_pos_2_conv*PI/180);
  A = {A_11, A_12, A_21, A_22};//
  
  // Inverse of Allocation Matrix
  A_inv = A * ~A;
  bool is_nonsingular = Invert(A_inv);
  A_term = ~A * A_inv;

  // Singularity Avoidance / Gimbal Recovery
  Gim_Rec = { Gim_pos_1_conv - 135 , Gim_pos_2_conv - 45 };
  float alpha = 0;
  float beta = 50;
  float zeta = beta/((alpha + beta) * dt);
  N = ( A_term * A - Idnt ) * zeta * Gim_Rec;
  
  // Gimbal vel commands
  Gim_com = A_term * Tau + N;//rad/s
  Gim_com_1 = Gim_com(0)*60/(2*PI);//RPM
  Gim_com_2 = Gim_com(1)*60/(2*PI);//RPM
  if (Gim_com_1 >= Gim_vel_thresh){
    Gim_com_1 = Gim_vel_thresh;
  }
  else if (Gim_com_1 < -Gim_vel_thresh){
    Gim_com_1 = -Gim_vel_thresh;
  }
  if (Gim_com_2 >= Gim_vel_thresh){
    Gim_com_2 = Gim_vel_thresh;
  }
  else if (Gim_com_2 < -Gim_vel_thresh){
    Gim_com_2 = -Gim_vel_thresh;
  }
  
  //.......................Gimbal Command...........................//
  // Gimbal speed or position command
  if (Serial.available()) {
    input = Serial.readStringUntil('\n');
    var = input.toInt();
  }
  if (var == 0) {
    PID_gim_1.Start(0, 0, cmd_gim_1); // input current_output setpoint
    float gim_hom_com_1 = PID_gim_1.Run(Gim_pos_1);
    float gimbal_hom_vel_com_1 = constrain(gim_hom_com_1,-2.0,2.0);
    dxl.setGoalVelocity(DXL_ID_1, gimbal_hom_vel_com_1 * rads2rpm, UNIT_RPM);

    PID_gim_2.Start(0, 0, cmd_gim_2); // input current_output setpoint
    float gim_hom_com_2 = PID_gim_2.Run(Gim_pos_2);
    float gimbal_hom_vel_com_2 = constrain(gim_hom_com_2,-2.0,2.0);
    dxl.setGoalVelocity(DXL_ID_2, gimbal_hom_vel_com_2 * rads2rpm, UNIT_RPM);
  }
  else if (var == 1) {
    dxl.setGoalVelocity(DXL_ID_1, -1*Gim_com_1, UNIT_RPM);//-1 to reverse the gimbal direction
    dxl.setGoalVelocity(DXL_ID_2, -1*Gim_com_2, UNIT_RPM);//-1 to reverse the gimbal direction
  }
  else if (var == 2) {
    PID_gim_1.Start(0, 0, 225); // input current_output setpoint
    float gim_hom_com_1 = PID_gim_1.Run(Gim_pos_1);
    float gimbal_hom_vel_com_1 = constrain(gim_hom_com_1,-2.0,2.0);
    dxl.setGoalVelocity(DXL_ID_1, gimbal_hom_vel_com_1 * rads2rpm, UNIT_RPM);
    
    PID_gim_2.Start(0, 0, 45); // input current_output setpoint
    float gim_hom_com_2 = PID_gim_2.Run(Gim_pos_2);
    float gimbal_hom_vel_com_2 = constrain(gim_hom_com_2,-2.0,2.0);
    dxl.setGoalVelocity(DXL_ID_2, gimbal_hom_vel_com_2 * rads2rpm, UNIT_RPM);
  }

  //............................ Print...........................//
  //Loop
//  Serial.print(loop_no);          Serial.print(" , ");
//  Serial.print(dt);               Serial.print(" , ");
//  Serial.print(tim);              Serial.print("   ,   ");
  
  //IMU
  Serial.print(x_act);            Serial.print(" , ");//Degrees
  Serial.print(y_act);            Serial.print(" , ");
  Serial.print(avg_x);            Serial.print(" , ");//Deg/s
  Serial.print(avg_y);            Serial.print(" , ");
  Serial.print(W_act(0));         Serial.print(" , ");//Deg/s
  Serial.print(W_act(1));         Serial.print(" , ");
//Serial.println(W_act(2));

  //Flywheel
//  Serial.print(0);                Serial.print(" , ");
//  Serial.print(cmd_spd_1);        Serial.print(" , ");
//  Serial.print(cur_spd_1);        Serial.print(" , ");
//  Serial.print(cur_spd_2);        Serial.print("   ,   ");

  //Torque
  //  Serial.print(Tau(0));           Serial.print(" , ");
//  Serial.print(Tau(1));           Serial.print("   ,   ");
  
  //Gimbal
  Serial.print(Gim_pos_1_conv);   Serial.print(" , ");
//  Serial.print(135);   Serial.print(" , ");
//  Serial.print(45);   Serial.print(" , ");
  Serial.print(Gim_pos_2_conv);   Serial.print(" , ");
//  Serial.print(Gim_com_1);        Serial.print(" , ");
//  Serial.print(Gim_com_2);        Serial.print(" , ");
  Serial.print(-Gim_vel_1);       Serial.print(" , ");
  Serial.println(-Gim_vel_2);

  //............................ Time...........................//
  loop_no = loop_no + 1;
  tim = tim + dt;
//  delay(20);//0.15 sec
  loop_end = millis();
  dt = (loop_end - loop_start) / 1000;
}

// =====================================================================================================
// VOID FUNCTIONS
// =====================================================================================================

void interruptFired_1() { //Common function for both CMGs
  int_cnt_1++;
}

void interruptFired_2() { //Common function for both CMGs
  int_cnt_2++;
}

float meas_speed_1 () {
  float speed_meas_1 = 0.0;
  noInterrupts() ;
  int_cnt_1 = 0;  // set variable in critical section
  interrupts() ;
  delay (rpm_calc_delay);
  noInterrupts() ;
  int critical_rpm_1 = int_cnt_1 ;  // read variable in critical section
  interrupts() ;
  my_motor_rpm_1 = ((critical_rpm_1) * (60)) / (numpoles) * (1000 / rpm_calc_delay);
  speed_meas_1 = my_motor_rpm_1;
  return speed_meas_1;
}

float meas_speed_2 () {
  float speed_meas_2 = 0.0;
  noInterrupts() ;
  int_cnt_2 = 0;  // set variable in critical section
  interrupts() ;
  delay (rpm_calc_delay);
  noInterrupts() ;
  int critical_rpm_2 = int_cnt_2 ;  // read variable in critical section
  interrupts() ;
  my_motor_rpm_2 = ((critical_rpm_2) * (60)) / (numpoles) * (1000 / rpm_calc_delay);
  speed_meas_2 = my_motor_rpm_2;
  return speed_meas_2;
}

float conv_meas_ang(float meas_ang) {
  divend_val = (meas_ang * deg2rad) / (2.0 * rev_val);

  // measurement range conversion (0~360 deg)
  meas_ang = (meas_ang * deg2rad) - float(divend_val) * (2.0 * rev_val);

  // Shift angle measurement range: -180.0 ~ 180 deg
  if ((meas_ang >= 0.0) && (meas_ang <= 180.0 * deg2rad)) {
    meas_ang_conv_res = meas_ang;
  }
  else if ((meas_ang > 180.0 * deg2rad) && (meas_ang < 360.0 * deg2rad)) {
    meas_ang_conv_res = meas_ang - 360.0 * deg2rad;
  }
  else if ((meas_ang <= 0.0) && (meas_ang >= (-1.0) * 180.0 * deg2rad)) {
    meas_ang_conv_res = meas_ang;
  }
  else if ((meas_ang < (-1.0) * 180.0 * deg2rad) && (meas_ang > (-1.0) * 360.0 * deg2rad)) {
    meas_ang_conv_res = meas_ang + 360.0 * deg2rad;
  }

  return meas_ang_conv_res;
}
