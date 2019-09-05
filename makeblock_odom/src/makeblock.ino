/**
 * Stub for Lab1 code, Sensors and Sensing at Orebro University
 * 2018
 * 
 * The purpose of this lab is to implement a velocity PID controller
 * for a DC motor with an onboard encoder, connected to the MakeBlock 
 * MegaPi board. Please note: the MegaPi libraries already implement 
 * many of the functionalities needed for this lab. You are only 
 * allowed to use the following library calls:
 *    1. uint8_t MeEncoderOnBoard::getPortB(void);      //get second channel port number
 *    2. uint8_t MeEncoderOnBoard::getIntNum(void);     //get interrupt port number
 *    3. void MeEncoderOnBoard::setMotorPwm(int pwm);   //set a desired control to the motor
 * 
 * 
 * This sketch is based on the example under 
 * Me_On_Board_encoder>Me_MegaPi_encoder_pwm.
 * 
 * This code is licensed under the BSD license.
 * Author list:
 *  - Todor Stoyanov
 */

#include <MeMegaPi.h>

#include <ros.h>
#include <makeblock_odom/MotorStates.h>
#include <makeblock_odom/SetDDParams.h>
#include <makeblock_odom/SetPID.h>
#include <makeblock_odom/ResetOdom.h>

//////////////// struct definitions ///////////////////
/** structure to hold PID parameters */
struct PIDParameters {
  float Kp_;
  float Kd_;
  float Ki_;
  float u_max_; //Maximum controller output (<= max PWM)
  float u_min_; //Minimum controller output [>= -(max PWM)]
  float I_;     //Serves as memory for the integral term [i.e., I=dT*(Ki*e_0, ... , Ki*e_t)]

  PIDParameters(float Kp, float Ki, float Kd, float u_max, float u_min, float I) : Kp_(Kp), Kd_(Kd), Ki_(Ki), u_max_(u_max), u_min_(u_min), I_(I) {};
};

/** structure to hold values associated with a controlled motor */
struct MotorState {
  float r_;  //setpoint value at current iteration (in radians/sec)
  float e_;  //error at current iteration
  float de_; //error derivative at current iteration
  float u_;  //computed control

  MotorState(float r, float e, float de, float u): r_(r),e_(e),de_(de),u_(u) {};
};

/** structure to hold values for an encoder */
struct EncoderState {
  long ticks_;  //raw number of ticks
  float ratio_; //number of ticks per radian
  float pos_;   //position in radians
  float vel_;   //velocity in radians/sec
  EncoderState(long ticks, float ratio, float pos, float vel): ticks_(ticks),ratio_(ratio),pos_(pos),vel_(vel) {};
};

////////////// global variables ////////////////////
//the maximum value we can command the motor
const short MAX_PWM_MEGAPI = 255;
//values bellow this do not make the motor move, so we can clamp them
const short MIN_ACTUATION_PWM = 10;
//TODO: set correct value for ticks per radian 
const float ENC_TICKS = 1; 

//TODO: here setup encoders to the appropriate ports
MeEncoderOnBoard Encoder_1(SLOT2);
MeEncoderOnBoard Encoder_2(SLOT4);

PIDParameters* pid_left_vc = new PIDParameters(1.0, 0.0, 0.0, MAX_PWM_MEGAPI, -MAX_PWM_MEGAPI, 0.0); //Velocity controller PID parameters for Motor 0
PIDParameters* pid_right_vc = new PIDParameters(1.0, 0.0, 0.0, MAX_PWM_MEGAPI, -MAX_PWM_MEGAPI, 0.0); //Velocity controller PID parameters for Motor 1
MotorState* left_motor_state = new MotorState(0.0,0,0,0); //accumulator for left motor
MotorState* right_motor_state = new MotorState(0,0,0,0);//accumulator for right motor
EncoderState* left_encoder = new EncoderState(0,ENC_TICKS,0,0);
EncoderState* right_encoder = new EncoderState(0,ENC_TICKS,0,0);

//ids of the two motors
const short LEFT_MOTOR=0;
const short RIGHT_MOTOR=1;

//timers
int t_new, t_old, t_old_comm, t_old_serial;
int dT = 5000; //sampling time in microseconds
int dT_serial = 10000; //sampling time for output in microseconds

//velocity filtering parameter
float alpha = 0.05;

///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////ROS Callbacks//////////////////////////
//callback function for setting new PID parameters
void setPIDCallback(const makeblock_odom::SetPID::Request &req, makeblock_odom::SetPID::Response &res);
//TODO: Add callbacks for the remaining services
//TODO: Add callback for commanded velocities
///////////////////////////////////////////////////////////////////////////////////

///////////////////ROS global variables/////////////////////////
ros::NodeHandle nh;
makeblock_odom::MotorStates state;
ros::Publisher state_publisher("/motor_states", &state);
ros::ServiceServer<makeblock_odom::SetPID::Request, makeblock_odom::SetPID::Response> pid_server("set_pid", &setPIDCallback);
//TODO: add servers for the remaining two services
//TODO: add publisher for odometry messages nav_msgs::Odometry
//TODO: add subscriber for commands geometry_msgs::Twist
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
//////////////////////////// function declarations /////////////////////////////
////////////////////////////////////////////////////////////////////////////////


//setup interupts and serial communication
void setup()
{
 
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  //setup timers
  t_old = micros();
  t_old_serial = micros();

  ////// ROS initializations////
  nh.initNode();
  nh.advertise(state_publisher);
  nh.advertiseService(pid_server);
  //TODO: advertise additional services and publishers, subscribe to command topic
  ///////////////////////////////
}


/////////////////////////////////////////////////////////////////
/////////////////// Main function ///////////////////////////////
/////////////////////////////////////////////////////////////////

void loop()
{
  
  //spin and check if we should publish
  t_new = micros();
  nh.spinOnce();
  if (abs(t_new - t_old_serial) > dT_serial) {
    //TODO: here update the state varriable with the most recent measurements
    state_publisher.publish(&state);
    t_old_serial = t_new;
  }
  
  //Do nothing if the sampling period didn't pass yet
  if (abs(t_new - t_old) < dT)
    return;
  t_old = t_new;

  ////// the logic bellow can be copied from the previous lab ////
  //calculate new encoder velocity and position
  float enc_pos, enc_vel;
  enc_pos = left_encoder->ticks_/left_encoder->ratio_;
  enc_vel = (float)(enc_pos - left_encoder->pos_)*1e6 / (float)dT;
  left_encoder->pos_ = enc_pos;
  //apply low-pass filter
  left_encoder->vel_ = (1-alpha)*left_encoder->vel_ + alpha*enc_vel;
  
  //TODO: same for right motor  
  //TODO: calculate error and derivative
  //TODO: calculate control
  //TODO: if control is bellow minmum or above maximum command, clamp
  //TODO: actuate motors using setMotorPwm

}

//TODO: add and implement callbacks here.
void setPIDCallback(const makeblock_odom::SetPID::Request &req, makeblock_odom::SetPID::Response &res) {

}

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////// NOTE: REMAINING FUNCTIONS CAN BE COPPIED FROM THE PREVIOUS LAB ////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

//TODO here increment encoder1 if PortB is also high
void isr_process_encoder1(void)
{

}

//TODO here increment encoder2 if PortB is also high
void isr_process_encoder2(void)
{

}

//--------------------------------------------------------------------------//
//   Here compute the PID control for a given error, de, and PID params     //
//--------------------------------------------------------------------------//
float pid(float e, float de, PIDParameters* p)
{
  //TODO: update the integral term
  //TODO: compute the control value

  //TODO: clamp the control value and back-calculate the integral term (the latter to avoid windup)
  
  //TODO: return u;
}


