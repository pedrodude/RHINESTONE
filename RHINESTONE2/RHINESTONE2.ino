#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_NeoPixel.h>

uint8_t neopixel_PIN = 9;
uint8_t NUMPIXELS = 1;
uint8_t spi_clock = 15; // aka SCL
uint8_t spi_miso = 14; // aka SDO
uint8_t spi_mosi = 16; // aka SDA
uint8_t spi_cs = 10;
uint8_t button_PIN = 7;

float CONSTANT_UPWARD_DT = 0.65;
float CONSTANT_DOWNWARD_DT = 0.3;
long CONSTANT_DEBOUNCETIME = 3000; // in milliseconds
float CONSTANT_STRAIGHTANDLEVELRANGE = 0.2; //was 0.05
float CONSTANT_GRAVITY = 9.81;  //flat, not moving = 1.59, straight down = 1.29
//float CONSTANT_ZERO_READING = 1.59;
const float pi = 3.14159;
const uint8_t VARIABLE_QUEUE_DECELERATION_SIZE = 5;

uint8_t FIFO_STATUS;
uint8_t VARIABLE_UPDOWN;
float VARIABLE_COMPENSATED_DECELERATION;
String VARIABLE_CURRENT_MODE;
float VARIABLE_QUEUE_DECELERATION[VARIABLE_QUEUE_DECELERATION_SIZE];
float VARIABLE_QUEUE_RANGE;
float VARIABLE_COMPUTED_PITCHANGLE; // in radians.  Arduino trig functions default to radians.
float VARIABLE_SAMPLE_RATE; // in Hz
float VARIABLE_STRAIGHTANDLEVEL_DECISION_RATE; // in Hz

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, neopixel_PIN, NEO_GRB + NEO_KHZ800);
/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(spi_clock, spi_miso, spi_mosi, spi_cs, 12345);
sensors_event_t event;
volatile uint8_t state = 0;
unsigned long sample_time; //initial sample time
unsigned long decision_time; //initial decision time //does this need to be started now?  What about sample time?
float min_val = 0; //used to calculate data range
float max_val = 0; //used to calculate data range
float mean = 0;
float range;

void setup()
{
  pixels.begin();

  Serial.begin(9600);
  //while (!Serial);

  sensor_t sensor;
  accel.getSensor(&sensor);
//  Serial.println("------------------------------------");
//  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
//  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
//  Serial.print  ("Read Device ID:  "); Serial.println(accel.getDeviceID());
//  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
//  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
//  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
//  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
//  Serial.println("------------------------------------");
//  Serial.println("");
//  delay(500);
  
  if(!accel.begin())
  {
    Serial.println("Problem detecting the ADXL345 - check connections");
    while(1); //All this does is fails forever, maybe get it to retry?
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_4_G);

  pixels.setPixelColor(0, pixels.Color(150,150,0)); // Moderately bright green color.
  pixels.show();

  pinMode(button_PIN, INPUT);
  attachInterrupt(4, ISR_function, FALLING); // Interrupt on Pin 7

  BOOT();
  TRANSITION_TO_SLEEP();
}

void loop()
{  
//  accel.getEvent(&event);

//  Serial.println(event.acceleration.x);
//  Serial.println(event.acceleration.y);
//  Serial.println(event.acceleration.z);
//
//  delay(2000);
//  //pixels.setPixelColor(0, pixels.Color(150,150,0)); // Moderately bright green color.
//  pixels.setPixelColor(0, pixels.Color(int(event.acceleration.x),int(event.acceleration.y),int(event.acceleration.z)));
//  pixels.show();
}

void ISR_function()
{
  detachInterrupt(4);
  delay(CONSTANT_DEBOUNCETIME);
  if(digitalRead(button_PIN) == 0) //(SENSOR_UCS) == 0)
  {
      Serial.println("UCS has been pressed for the debounce period");
      if(VARIABLE_CURRENT_MODE == "mode_normal")
      {
        TRANSITION_TO_SLEEP();
      }
      else if(VARIABLE_CURRENT_MODE == "mode_sleep")
      {
        TRANSITION_TO_NORMAL();
      }
   }
   attachInterrupt(4, ISR_function, FALLING); // Interrupt on Pin 7
}

void BOOT()
{
  Serial.println("BOOT ROUTINE");
  VARIABLE_UPDOWN = 0;
  VARIABLE_COMPENSATED_DECELERATION = 0;
  VARIABLE_CURRENT_MODE = "boot";
  VARIABLE_QUEUE_RANGE = 0;
  VARIABLE_COMPUTED_PITCHANGLE = 0;
  VARIABLE_SAMPLE_RATE = 90; //was 60
  VARIABLE_STRAIGHTANDLEVEL_DECISION_RATE = 30; //was 1
  FIFO_STATUS = 0;

  for(uint8_t i = 0; i<VARIABLE_QUEUE_DECELERATION_SIZE; i++)
  {
    VARIABLE_QUEUE_DECELERATION[i] = 0;
  }
}

void TRANSITION_TO_SLEEP()
{
  Serial.println("TRANSITION_TO_SLEEP ROUTINE");
  VARIABLE_CURRENT_MODE = "transition_to_sleep";
  SLEEP_SIGNAL();
  MODE_SLEEP();
}

void TRANSITION_TO_NORMAL()
{
  Serial.println("TRANSITION_TO_NORMAL ROUTINE");
  VARIABLE_CURRENT_MODE = "transition_to_normal";
  NORMAL_SIGNAL();
  MODE_NORMAL();
}

void MODE_NORMAL()
{
  Serial.println("MODE_NORMAL ROUTINE");
  VARIABLE_CURRENT_MODE = "mode_normal";

  //read sensor data
  accel.getEvent(&event);  //20150714 can this go into APPEND routine?  And sensors_event_t?  Get rid of a global?

  sample_time  = millis();
  decision_time  = millis(); //20150714 Can I move these out of the subroutines and back into mode_normal to get rid of the globals?

  Serial.print("FIFO_STATUS: ");
  Serial.println(FIFO_STATUS);
  APPEND_SAMPLE_TO_QUEUE(); //subroutine to append reading to queue
  if(FIFO_STATUS == VARIABLE_QUEUE_DECELERATION_SIZE) //only do arithmetic and accel test when queue is full
  {
    CALCULATE_QUEUE_STATISTICS();
    EVALUATE_DECELERATION();
  }
}

void MODE_SLEEP()
{
  Serial.println("MODE_SLEEP ROUTINE");
  VARIABLE_CURRENT_MODE = "mode_sleep";
}

void SLEEP_SIGNAL()
{
  Serial.println("SLEEP_SIGNAL ROUTINE");
  unsigned long time = millis();


  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(200,0,0)); // Moderately bright green color.
  pixels.show();
  while(millis() - time < 200);
  //digitalWrite(ACTUATOR_SL,HIGH);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
  pixels.show();
  delay(50);
  
  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(200,0,0)); // Moderately bright green color.
  pixels.show();
  while(millis() - time < 100);
  //digitalWrite(ACTUATOR_SL,HIGH);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
  pixels.show();
  delay(50);
  
  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(200,0,0)); // Moderately bright green color.
  pixels.show();
  while(millis() - time < 50);
  //digitalWrite(ACTUATOR_SL,HIGH);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
  pixels.show();
  while(millis() - time < 1000);

}

void NORMAL_SIGNAL()
{
  Serial.println("NORMAL_SIGNAL ROUTINE");
  unsigned long time = millis();

  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
  pixels.show();
  while(millis() - time < 200);
  //digitalWrite(ACTUATOR_SL,HIGH);
  pixels.setPixelColor(0, pixels.Color(200,0,0)); // Moderately bright green color.
  pixels.show();
  delay(50);
  
  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
  pixels.show();
  while(millis() - time < 100);
  //digitalWrite(ACTUATOR_SL,HIGH);
  pixels.setPixelColor(0, pixels.Color(200,0,0)); // Moderately bright green color.
  pixels.show();
  delay(50);
  
  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
  pixels.show();
  while(millis() - time < 50);
  //digitalWrite(ACTUATOR_SL,HIGH);
  pixels.setPixelColor(0, pixels.Color(200,0,0)); // Moderately bright green color.
  pixels.show();
  while(millis() - time < 1000);
  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
  pixels.show();
}

void BRAKING_SIGNAL_ON()
{
  Serial.println("BRAKING_SIGNAL_ON ROUTINE");
  //digitalWrite(ACTUATOR_SL,HIGH);
  pixels.setPixelColor(0, pixels.Color(200,0,0)); // Moderately bright green color.
  pixels.show();
}

void BRAKING_SIGNAL_OFF()
{
  Serial.println("BRAKING_SIGNAL_OFF ROUTINE");
  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
  pixels.show();
}

void APPEND_SAMPLE_TO_QUEUE()
{
  Serial.println("APPEND_SAMPLE_TO_QUEUE ROUTINE");

  //Shift FIFO values down one space
  Serial.println("Queue being updated");
  for(uint8_t i=VARIABLE_QUEUE_DECELERATION_SIZE-1; i>0; i--) //for(uint8_t i=0; i<VARIABLE_QUEUE_DECELERATION_SIZE-1; i++)//was-2
  {
    VARIABLE_QUEUE_DECELERATION[i] = VARIABLE_QUEUE_DECELERATION[i-1];
  }

  //Add new value to beginning of FIFO
  VARIABLE_QUEUE_DECELERATION[0] = event.acceleration.x;

  //print value just added
  Serial.print("Uncompensated acceleration reading: ");
  Serial.print(VARIABLE_QUEUE_DECELERATION[0]);
  Serial.println(" inserted to queue");

  //printing queue to screen
  for(uint8_t i=0; i<VARIABLE_QUEUE_DECELERATION_SIZE; i++)//was-1
  {
    Serial.print("|");
    Serial.print("i");
    Serial.print(i);
    Serial.print(":");
    Serial.print(VARIABLE_QUEUE_DECELERATION[i]);
  }
  Serial.println("");

  //Update FIFO_STATUS
  if(FIFO_STATUS < VARIABLE_QUEUE_DECELERATION_SIZE)
  {
    FIFO_STATUS++;
  }

  //Maintain fixed sample rate
  while((millis() - sample_time) < (1000/VARIABLE_SAMPLE_RATE));
}

void CALCULATE_QUEUE_STATISTICS()
{
  Serial.println("CALCULATE_QUEUE_STATISTICS ROUTINE");

  //Calculate range and mean of sampled data
  min_val = VARIABLE_QUEUE_DECELERATION[0];
  max_val = VARIABLE_QUEUE_DECELERATION[0];
  mean = VARIABLE_QUEUE_DECELERATION[0];

  for(uint8_t i=1; i<VARIABLE_QUEUE_DECELERATION_SIZE; i++) //was: for(uint8_t i=1; i<FIFO_STATUS; i++)
  {
    if(VARIABLE_QUEUE_DECELERATION[i] < min_val)
    {
      min_val = VARIABLE_QUEUE_DECELERATION[i];
    }
    if(VARIABLE_QUEUE_DECELERATION[i] > max_val)
    {
      max_val = VARIABLE_QUEUE_DECELERATION[i];
    }
    mean = mean + VARIABLE_QUEUE_DECELERATION[i];
  }

  mean = mean/VARIABLE_QUEUE_DECELERATION_SIZE; //was: mean = mean/FIFO_STATUS;
  Serial.print("Queue mean: ");
  Serial.println(mean);
  range = max_val - min_val;
  Serial.print("Queue range: ");
  Serial.println(range);
}

void EVALUATE_DECELERATION()
{
  Serial.println("EVALUATE_DECELERATION ROUTINE");

  //Determine if event occurred based on mean and range
  if(range <= CONSTANT_STRAIGHTANDLEVELRANGE) //20150715 This if could largely be superseded by an if in an earlier routine, maybe mode normal or calculate statistics, and only the relevant pitch angle/calculate decel code called.
  {
    Serial.println("Queue range indicates insignificant deceleration, updating pitch angle");
    VARIABLE_COMPUTED_PITCHANGLE = asin(mean/CONSTANT_GRAVITY);//*(180/pi);
    Serial.print("Pitch Angle: ");
    Serial.println(VARIABLE_COMPUTED_PITCHANGLE);
    //20150714 IS THERE ANY REASON THIS ALWAYS SEEMS TO EVALUATE TO 0.03 ON LAST TEST?
  }
  else
  {
    //20150714 ONLY IF THE DECELERATION IS "SIGNIFICANT" (i.e. range is wide enough) ARE THE DECISIONS MADE TO TURN LIGHT ON OR OFF.  DOESN'T ACCOUNT FOR THE LIKELY POSSIBILITY THAT LIGHT GETS TURNED ON THEN THE RANGE DROPS AND IT NEVER GETS TURNED OFF...
    Serial.println("Queue range indicates significant deceleration, calculating compensated deceleration");
    VARIABLE_COMPENSATED_DECELERATION = abs(mean - (CONSTANT_GRAVITY*sin(VARIABLE_COMPUTED_PITCHANGLE)))*cos(VARIABLE_COMPUTED_PITCHANGLE);
    Serial.print("Compensated deceleration: ");
    Serial.println(VARIABLE_COMPENSATED_DECELERATION);

    if((VARIABLE_UPDOWN == 0)&&(VARIABLE_COMPENSATED_DECELERATION >= CONSTANT_UPWARD_DT))
    {
      Serial.println("Upward");
      TRANSITION_ACTIVATE_ACTUATOR_SL();
    }
    else if((VARIABLE_UPDOWN == 1)&&(VARIABLE_COMPENSATED_DECELERATION < CONSTANT_DOWNWARD_DT)) //20150714 Any reason this isn't encapsulated in curly braces in the same way as the previous else? 
    {
      Serial.println("Downward");
      TRANSITION_DEACTIVATE_ACTUATOR_SL();
    }
  }

  while((millis() - decision_time) < (1000/VARIABLE_STRAIGHTANDLEVEL_DECISION_RATE));
}

void TRANSITION_ACTIVATE_ACTUATOR_SL()
{
  Serial.println("TRANSITION_ACTIVATE_ACTUATOR_SL ROUTINE");
  //VARIABLE_CURRENT_MODE = "transition_activate_actuator_sl";
  BRAKING_SIGNAL_ON();
  VARIABLE_UPDOWN = 1;
}

void TRANSITION_DEACTIVATE_ACTUATOR_SL()
{
  Serial.println("TRANSITION_DEACTIVATE_ACTUATOR_SL ROUTINE");
  //VARIABLE_CURRENT_MODE = "transition_deactivate_actuator_sl";
  BRAKING_SIGNAL_OFF();
  VARIABLE_UPDOWN = 0;
}

