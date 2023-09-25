#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>

#define left        0 
#define right       1

#define kp_balance  1.2//6
#define kd_balance  10//30
#define ki_balance  0.08//0.3

#define kp_axis 0
#define kd_axis 0
#define ki_axis 0

#define sample_time 7

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
bool direction=0,prev_direction=0;
unsigned long lastMillis = 0;

double axis_angle,balance_angle,diff_angle,prev_angle=0,prev_diff=0,derrivative,integral=0,integral_ki=0,circle_angle;
int speed=0;

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
   pinMode(DIR_PIN, OUTPUT);
   pinMode(PWM_PIN, OUTPUT);
   pinMode(DIR_PIN2, OUTPUT);
   pinMode(PWM_PIN2, OUTPUT);
   digitalWrite(DIR_PIN,LOW);
   digitalWrite(DIR_PIN2,LOW);
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

int control_motor_pid_with_speed(double ang,int max_speed)
{
 ////////////////////// proportional ///////////////////////////////

  diff_angle=(ang-balance_angle)*kp_balance;
  
 ///////////////////// derivative /////////////////////////////////

  derrivative=(diff_angle-prev_diff)*kd_balance;//sample_time;
  
  ////////////////// integral ////////////////////////////////////
 
  integral += diff_angle;
  
  integral_ki = integral * ki_balance;

  // limit the value of integral_ki 
  // if both variable same then the actual value of integral will change
 
  if(integral_ki>20)
  {
    integral_ki=20;
  }
  else if(integral_ki < -20)
  {
    integral_ki=-20;
  }

  // final speed calculation
  speed = diff_angle + derrivative + integral_ki;

  if(speed>max_speed)
  {
    speed=max_speed;
  }     
  else if (speed < 0-max_speed)
  {
    speed=-max_speed;          
  }

  if(speed>0)
  {
    direction=0;  
  }
  else
  { 
    direction=1;
    speed=-1*speed;
  }

  /////////// if we want to check prev_direction and take feedback
  //   prev_direction=direction;
  prev_diff=diff_angle;
  control_motor_speed_Simplified_Serial(left,direction,speed);
  control_motor_speed_Simplified_Serial(right,direction,speed);  
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
 
  for(int i=0;i<1000;i++)
  {
  sensors_event_t event;
  bno.getEvent(&event);
  balance_angle=event.orientation.y;
  // circle_angle=event.orientation.x;
   Serial.println(balance_angle);
  // Serial.print(balance_angle);
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
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= sample_time) 
  {
    lastMillis = currentMillis;
    control_motor_pid_with_speed(2.5,20);
  }
 
 }

 while(1)
 {
  
  sensors_event_t event;
  bno.getEvent(&event);
  balance_angle=event.orientation.y;
  // circle_angle=event.orientation.x;
  // Serial.println(balance_angle);
  // Serial.print(balance_angle);
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
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= sample_time) 
  {
    lastMillis = currentMillis;
    control_motor_pid_with_speed(0,20);
  }
 
 }
 
  
}
