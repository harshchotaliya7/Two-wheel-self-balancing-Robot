#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>

float kp=6;
float kd=30;
float ki=0.3;

/////////////////////////// for gradient descent ///////////////////////////////////

#define learning_rate 0.001

int DIR_PIN = 5;
int PWM_PIN = 6;
int DIR_PIN2 = 4;
int PWM_PIN2 = 3;
bool direction=0;
unsigned long lastMillis = 0;

double axis_angle,diff_angle,prev_angle=0,prev_diff=0,derrivative,integral=0;
int speed=0;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {

   
   Serial.begin(115200);

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

  diff_angle=axis_angle-ang;
  
 ///////////////////// derivative /////////////////////////////////

  derrivative=diff_angle-prev_diff;
  
  ////////////////// integral ////////////////////////////////////

  integral += diff_angle;

  speed = (diff_angle*kp) + (derrivative*kd) + (integral*ki);//+ ((diff_angle+10000)/fabs(diff_angle+10000)*60);

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
    speed=-1*speed;
    direction=1;
  }
  prev_diff=diff_angle;
  control_motor_speed(direction,speed); 
}
void loop() {
 
//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// forward ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

 for(int i=0;i<2000;i++)
 {
  sensors_event_t event;
  bno.getEvent(&event);
  axis_angle=event.orientation.z;

  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= 6) 
  {
    lastMillis = currentMillis;
    control_motor_pid_with_speed(3.1,150);
  }
 }

//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Brake ////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

 for(int i=0;i<500;i++)
 {
  sensors_event_t event;
  bno.getEvent(&event);
  axis_angle=event.orientation.z;

  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= 3) 
  {
    lastMillis = currentMillis;
    control_motor_pid_with_speed(-2,150);
  }
 }

////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////// Standing with self balancing ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

 while(1)
 {
  sensors_event_t event;
  bno.getEvent(&event);
  axis_angle=event.orientation.z;

  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= 4) 
  {
    lastMillis = currentMillis;
    control_motor_pid_with_speed(0.1,150);
  }
 }


/////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////// printing values //////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

  //control_motor_speed(0,30); 
  //Serial.print("kp_value =");
  //Serial.print("                          ");
  //Serial.print("angle =");
  //Serial.println(axis_angle);

  
}
