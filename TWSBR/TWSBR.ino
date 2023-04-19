#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define kp 1.5
#define kd 150


int DIR_PIN = 6;
int PWM_PIN = 7;
int DIR_PIN2 = 5;
int PWM_PIN2 = 3;
double axis_angle,diff_angle,prev_angle=0,prev_diff=0,derrivative;
int speed=0;


/*
void loop() {


  digitalWrite(DIR_PIN2, HIGH);

  analogWrite(PWM_PIN2, 70);


}
*/


/* 
   Connections
   ===========
   Connect SCL to SCL pin (analog 5 on Arduino UNO)
   Connect SDA to SDA pin (analog 4 on Arduino UNO)
   Connect VDD to 3-5V DC (depending on your board's logic level)
   Connect GROUND to common ground
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(PWM_PIN2, OUTPUT);
  digitalWrite(DIR_PIN,HIGH);
  digitalWrite(DIR_PIN2,HIGH);
  /* Display some basic information on this sensor */
  displaySensorDetails();
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/

void control_motor_speed(bool dir,int speed)
{
  switch(dir)
  {
    case 0:
        digitalWrite(DIR_PIN, HIGH); 
        analogWrite(PWM_PIN, speed); 
        digitalWrite(DIR_PIN2, LOW); 
        analogWrite(PWM_PIN2, speed); 
        break;
        
    case 1:
        
        digitalWrite(DIR_PIN, LOW); 
        analogWrite(PWM_PIN, speed); 
        digitalWrite(DIR_PIN2, HIGH);
        analogWrite(PWM_PIN2, speed);                     
  }    
}

void control_motor_pid_with_speed(double ang,int max_speed)
{
  bool direction=0;
 
  diff_angle=axis_angle-ang;
  
  derrivative=diff_angle-prev_diff;  
  
  speed = diff_angle*kp + derrivative*kd ;//+ ((diff_angle+10000)/fabs(diff_angle+10000)*60);
  
  prev_angle=axis_angle;
  
  prev_diff=diff_angle; 
  
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
    direction=1;
    speed=speed+60;
  }
  else
  {
    direction=0;
    speed=speed-60;
  }
  //direction=~direction;
  control_motor_speed(direction,fabs(speed));
  
}

/**************************************************************************/
void loop(void)
{
 
  sensors_event_t event;
  bno.getEvent(&event);
  axis_angle=event.orientation.z;
  control_motor_pid_with_speed(-1,100);


  
/**************************************************************************
******************* PRINTING SENSOR DATA **********************************
**************************************************************************/

 // Serial.print("\tZ: ");
 //Serial.println(speed);
  
  //control_motor_speed(1,30);
  //analogWrite(R_PWM, 0);
  //analogWrite(L_PWM, 50);  
  //delay(BNO055_SAMPLERATE_DELAY_MS);
  
  

}
