#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>

#define motor1        0 
#define motor2        1

#define amount_of_hold_angle_changed  3

///////// Proxy Pin Declaration

#define pIR_P1 8
#define pIR_P2 9
#define pIR_P3 10
#define pIR_P4 11
#define pIR_P5 12

///////// bot direction

#define forward   0
#define backward  1
#define stop      2

#define forward_bot_angle   -1.2
#define backward_bot_angle  -3.5
#define stop_bot_angle      -2.9

#define kp_balance  1.2
#define kd_balance  15
#define ki_balance  0.065

#define kp_axis 1.2
#define kd_axis 0
#define ki_axis 0

#define sample_time 9

/////////////////////////// Software Serial Pin

const byte rxPin = 2;
const byte txPin = 3;

// Set up a new SoftwareSerial object
SoftwareSerial mySerial (rxPin, txPin);

/////////////////////////// for gradient descent ///////////////////////////////////

#define learning_rate 0.001

int DIR_PIN = 5;
int PWM_PIN = 6;
int DIR_PIN2 = 4;
int PWM_PIN2 = 3;
bool motor1_direction=0,motor2_direction=0,prev_direction=0;
unsigned long lastMillis = 0;

////////////////// PID Variables 

double actual_axis_angle,actual_balance_angle,diff_angle_axis,diff_angle_balance,prev_angle_balance=0,
       prev_angle_axis=0,prev_diff_balance=0,prev_diff_axis=0,derrivative_balance,derrivative_axis,
       integral_balance=0,integral_axis=0,integral_ki_balance=0,integral_ki_axis=0,circle_angle,hold_angle;

///////////////// Speed Variables of Motor

int speed_motor1=0,speed_motor2=0,speed_axis_motor1,speed_axis_motor2;

///////////////// Variable for value of Proxy

bool IR_P1,IR_P2_L,IR_P3_M,IR_P4_R,IR_P5;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {

   
   Serial.begin(9600);
   mySerial.begin(9600);
   while (!Serial) delay(10);  // wait for serial port to open!
   Serial.println("Orientation Sensor Test"); Serial.println("");

    if(!bno.begin())
    {
       Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }

  pinMode(pIR_P1,INPUT);
  pinMode(pIR_P2,INPUT);
  pinMode(pIR_P3,INPUT);
  pinMode(pIR_P4,INPUT);
  pinMode(pIR_P5,INPUT_PULLUP);

}


void control_motor_speed(bool dir,int speed)
{
  switch(dir)
  {
    case 0:
        digitalWrite(DIR_PIN, LOW); 
        analogWrite(PWM_PIN, speed); 
        digitalWrite(DIR_PIN2, LOW); 
        analogWrite(PWM_PIN2, speed); 
        break;
        
    case 1:
        digitalWrite(DIR_PIN, HIGH); 
        analogWrite(PWM_PIN, speed); 
        digitalWrite(DIR_PIN2, HIGH);
        analogWrite(PWM_PIN2, speed); 
        break;                    
  }    
}

void bot_move(int fwd_or_bwd_or_stop,int duration,double hold_angle_calc)
{
  int i;
  float given_angle;
  switch(fwd_or_bwd_or_stop)
  {
    case 0:
      given_angle=forward_bot_angle;
      break;

    case 1:
      given_angle=backward_bot_angle;
      break;

    case 2:
      given_angle=stop_bot_angle;
      break;

    default:
      break;
  }
  for(i=0;i<duration;i++)
  {
      sensors_event_t event;
      bno.getEvent(&event);
      actual_balance_angle=event.orientation.y;
      actual_axis_angle=event.orientation.x;
      if(actual_axis_angle>180)
      {
        actual_axis_angle=actual_axis_angle-360;
      }
      unsigned long currentMillis = millis();
      if (currentMillis - lastMillis >= sample_time) 
      {
        lastMillis = currentMillis;
        control_motor_pid_with_speed(given_angle,20,hold_angle_calc);
      }
  }

}
int control_motor_pid_with_speed(double desired_balance_ang,int max_speed,double desired_axis_ang)
{
 ///////////////////////////////////////////////////////////////////
 ///////////////////// Axis PID Started ////////////////////////////
 ///////////////////////////////////////////////////////////////////

  diff_angle_axis=(desired_axis_ang-actual_axis_angle)*kp_axis;

  derrivative_axis=(diff_angle_axis-prev_diff_axis)*kd_axis;

  speed_axis_motor1 = diff_angle_axis + derrivative_axis;

  speed_axis_motor2 = (-1)*(speed_axis_motor1);

 ///////////////////////////////////////////////////////////////////
 ///////////////////// Balance PID Started /////////////////////////
 ///////////////////////////////////////////////////////////////////

 ////////////////////// proportional ///////////////////////////////

  diff_angle_balance=(desired_balance_ang-actual_balance_angle)*kp_balance;
  
 ///////////////////// derivative /////////////////////////////////

  derrivative_balance=(diff_angle_balance-prev_diff_balance)*kd_balance;//sample_time;
  
  ////////////////// integral ////////////////////////////////////
 
  integral_balance += diff_angle_balance;
  
  integral_ki_balance = integral_balance * ki_balance;

  // limit the value of integral_ki 
  // if both variable same then the actual value of integral will change
 
  if(integral_ki_balance>20)
  {
    integral_ki_balance=20;
  }
  else if(integral_ki_balance < -20)
  {
    integral_ki_balance=-20;
  }


////////////////// for motor 1 /////////////////////////////

  // final speed calculation
  speed_motor1 = diff_angle_balance + derrivative_balance + integral_ki_balance + speed_axis_motor1;
  speed_motor2 = diff_angle_balance + derrivative_balance + integral_ki_balance + speed_axis_motor2;

  if(speed_motor1>max_speed)
  {
    speed_motor1=max_speed;
  }     
  else if (speed_motor1 < 0-max_speed)
  {
    speed_motor1=-max_speed;          
  }

  if(speed_motor1>0)
  {
    motor1_direction=0;  
  }
  else
  { 
    motor1_direction=1;
    speed_motor1=-1*speed_motor1;
  }

////////////////// for motor 2 /////////////////////////////

  if(speed_motor2>max_speed)
  {
    speed_motor2=max_speed;
  }     
  else if (speed_motor2 < 0-max_speed)
  {
    speed_motor2=-max_speed;          
  }

  if(speed_motor2>0)
  {
    motor2_direction=0;  
  }
  else
  { 
    motor2_direction=1;
    speed_motor2=-1*speed_motor2;
  }

  /////////// if we want to check prev_direction and take feedback
  //   prev_direction=direction;
  prev_diff_balance=diff_angle_balance;
  prev_diff_axis   =diff_angle_axis;

  //////////// applying speed motor 1
  control_motor_speed_Simplified_Serial(motor1,motor1_direction,speed_motor1);  

  //////////// applying speed motor 2
  control_motor_speed_Simplified_Serial(motor2,motor2_direction,speed_motor2);
}

void control_motor_speed_Simplified_Serial(int channel,int direction,int speed_channel)
{
 int data_packet=0;
 if(speed_channel>63)
 {
   speed_channel=63;
 }
 ///////////////////////// speed bit clear ////////////////////////////
 data_packet &= ~(0x3F<<0);
 //////////////////////// speed bit set (desired) /////////////////////
 data_packet |= (speed_channel<<0);

 //////////////////////// direction bit clear /////////////////////////
 data_packet &= ~(0x1<<6);
 /////////////////////// direction bit set (desired) //////////////////
 data_packet |= (direction<<6);
 
 ////////////////////// channel bit clear /////////////////////////////
 data_packet &= ~(0x1<<7);
 ///////////////////// channel bit set (desired) //////////////////////
 data_packet |= (channel<<7);
 
 mySerial.write(data_packet);

}

void loop() {
  
  //  bot_move(stop,1000);
  //  bot_move(forward,1000);
 hold_angle=0;
 while(1)
 {
  sensors_event_t event;
  bno.getEvent(&event);
  actual_balance_angle=event.orientation.y;
  actual_axis_angle=event.orientation.x;
  if(actual_axis_angle>180)
  {
    actual_axis_angle=actual_axis_angle-360;
  }

  // IR_P1=digitalRead(pIR_P1);
  IR_P2_L=digitalRead(pIR_P2);
  IR_P3_M=digitalRead(pIR_P3);
  IR_P4_R=digitalRead(pIR_P4);
  // IR_P5=digitalRead(pIR_P5);
  
  bot_move(stop,1000,hold_angle);

  if(IR_P3_M)
  {
    bot_move(forward,300,hold_angle);
  }
  else
  {
    if(IR_P2_L)
    {
        hold_angle-=amount_of_hold_angle_changed;
        bot_move(stop,400,hold_angle);
    }
    else if(IR_P4_R)
    {
        hold_angle-=amount_of_hold_angle_changed;
        bot_move(stop,400,hold_angle);
    }
  }
  // Serial.print(actual_axis_angle);
  // Serial.print(" ");
  // // // Serial.print("   ");
  // // // Serial.print(IR_P1);
  // // // Serial.print(" ");
  // Serial.print(IR_P2_L);
  // Serial.print(" ");
  // Serial.print(IR_P3_M);
  // Serial.print(" ");
  // Serial.println(IR_P4_R);

  // Serial.println(IR_P5);
  
  // circle_angle=event.orientation.x;
  //control_motor_pid_with_speed(0,20,0);
  // Serial.print("Motor1 speed :");
  // Serial.print(speed_motor1);
  // Serial.print("    Motor1 Direction :");
  // Serial.print(motor1_direction);
  // Serial.print("    Motor2 speed :");
  // Serial.print(speed_motor2);
  // Serial.print("    Motor1 Direction :");
  // Serial.println(motor2_direction);
  // Serial.println(actual_axis_angle);
  // Serial.println(actual_balance_angle);
  // Serial.print("   kp_balance_balance=");
  // Serial.print(diff_angle);
  // Serial.print("   kd_balance=");
  // Serial.print(derrivative);
  // Serial.print("   ki_balance=");
  // Serial.print(integral_ki);
  // Serial.print("   speed=");
  // Serial.print(speed);
  // Serial.print("   direction=");
  // Serial.println(direction);
      // unsigned long currentMillis = millis();
      // if (currentMillis - lastMillis >= sample_time) 
      // {
      //   lastMillis = currentMillis;
      //   control_motor_pid_with_speed(stop_bot_angle,20,0);
      // }
 
 }
 
  
}


