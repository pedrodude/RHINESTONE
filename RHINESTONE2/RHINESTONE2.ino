//TBR present

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SPI.h>
//#include <ADXL345.h>
#include <Adafruit_NeoPixel.h>

uint8_t neopixel_PIN = 9;
uint8_t NUMPIXELS = 1;
uint8_t spi_clock = 15; // aka SCL
uint8_t spi_miso = 14; // aka SDO
uint8_t spi_mosi = 16; // aka SDA
uint8_t spi_cs = 10;
uint8_t button_PIN = 7; //TBR How does this relate to the next comment?  Is there a table of interrupts mapped to pins?

uint8_t interrupt1 = 4; //pin 7 is interrupt 4, pin 2 is interrupt 1, pin 0 is interrupt 2, and pin 1 is interrupt 3
uint8_t interrupt2 = 1; //pin 3 maps to interrupt 0

float CONSTANT_UPWARD_DT = 0.65;
float CONSTANT_DOWNWARD_DT = 0.3;
long CONSTANT_DEBOUNCETIME = 3000; // in milliseconds
float CONSTANT_STRAIGHTANDLEVELRANGE = 0.2; //was 0.05
float CONSTANT_GRAVITY = 9.81;  //flat, not moving = 1.59, straight down = 1.29
//float CONSTANT_ZERO_READING = 1.59;
const float pi = 3.14159;
const uint8_t VARIABLE_QUEUE_DECELERATION_SIZE = 32; //must be less than 32 to use FIFO

int16_t temp_FIFO_int[VARIABLE_QUEUE_DECELERATION_SIZE];
uint8_t FIFO_STATUS;
uint8_t VARIABLE_UPDOWN;
float VARIABLE_COMPENSATED_DECELERATION;
String VARIABLE_CURRENT_MODE;
float VARIABLE_QUEUE_DECELERATION[VARIABLE_QUEUE_DECELERATION_SIZE];
float VARIABLE_QUEUE_RANGE;
float VARIABLE_COMPUTED_PITCHANGLE; // in radians.  Arduino trig functions default to radians.  //Does the trig require degrees or will radians produce the same result?
//float VARIABLE_SAMPLE_RATE; // in Hz
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
  while (!Serial);

  sensor_t sensor;
  accel.getSensor(&sensor);
  delay(1000);
  
  if(!accel.begin())
  {
    Serial.println("Problem detecting the ADXL345 - check connections");
    while(1); //All this does is fails forever, maybe get it to retry?
  }

  accel.setRange(ADXL345_RANGE_4_G);
  accel.setDataRate(ADXL345_DATARATE_25_HZ); //TBR Out of interest, why this data rate?  Any tradeoffs with higher or lower data rates (energy usage, etc)?
  accel.setFIFOMode(ADXL345_FIFO_MODE_FIFO); //TBR Should this be ADXL345_FIFO_MODE_STREAM?  We want oldest values to be overwritten.
  accel.setFIFOSamples(VARIABLE_QUEUE_DECELERATION_SIZE-1);
  accel.writeRegister(ADXL345_REG_INT_ENABLE, 0);
  accel.writeRegister(ADXL345_REG_INT_MAP, 0);
  accel.writeBits(ADXL345_REG_INT_ENABLE, 1, ADXL345_INT_WATERMARK_BIT, 1); 

  pixels.setPixelColor(0, pixels.Color(150,150,0)); // Moderately bright green color.
  pixels.show();

  pinMode(button_PIN, INPUT);
  attachInterrupt(interrupt1, pushbutton_ISR, FALLING); // Interrupt on Pin 7

  BOOT();
  TRANSITION_TO_SLEEP();
}

void loop()
{  
}

void pushbutton_ISR()
{
  detachInterrupt(interrupt1);
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
   attachInterrupt(interrupt1, pushbutton_ISR, FALLING); // Interrupt on Pin 7
}

void BOOT()
{
  Serial.println("BOOT ROUTINE");
  VARIABLE_UPDOWN = 0;
  VARIABLE_COMPENSATED_DECELERATION = 0;
  VARIABLE_CURRENT_MODE = "boot";
  VARIABLE_QUEUE_RANGE = 0;
  VARIABLE_COMPUTED_PITCHANGLE = 0;
  //VARIABLE_SAMPLE_RATE = 90; //was 60
  VARIABLE_STRAIGHTANDLEVEL_DECISION_RATE = 30; //was 1
  FIFO_STATUS = 0;
}

void TRANSITION_TO_SLEEP()
{
  Serial.println("TRANSITION_TO_SLEEP ROUTINE");
  detachInterrupt(interrupt2);
  accel.writeBits(ADXL345_REG_POWER_CTL, 0, ADXL345_PCTL_MEASURE_BIT, 1);  //TBR Could these hardcoded bits be given variable/constant names to be a bit more intuitive?
  VARIABLE_CURRENT_MODE = "transition_to_sleep";  
  SLEEP_SIGNAL();
  MODE_SLEEP();
}

void TRANSITION_TO_NORMAL()
{
  Serial.println("TRANSITION_TO_NORMAL ROUTINE");
  attachInterrupt(interrupt2, Read_FIFO_ISR, RISING); // Interrupt on Pin 7 //TBR Isn't this interrupt on pin 2?
  accel.writeBits(ADXL345_REG_POWER_CTL, 1, ADXL345_PCTL_MEASURE_BIT, 1); //TBR These also should be named as variables
  VARIABLE_CURRENT_MODE = "transition_to_normal"; 
  NORMAL_SIGNAL();
  MODE_NORMAL();
}

void MODE_NORMAL()
{
  Serial.println("MODE_NORMAL ROUTINE");
  VARIABLE_CURRENT_MODE = "mode_normal";  
}

void Read_FIFO_ISR() //TBR unsure of the nomenclature here.  In what situations is this ISR called?
{
  detachInterrupt(interrupt2);

  Serial.println("FIFO filled");

  for(uint8_t i = 0; i<32; i++) //TBR Should this 32 be replaced with VARIABLE_QUEUE_DECELERATION_SIZE?
  {
    temp_FIFO_int[i] = accel.getX(); //TBR What is the purpose of this forloop?  It looks like the entire queue is being overwritten entirely with what would effectively be almost identical acceleration values?
  }

  CALCULATE_QUEUE_STATISTICS();
  attachInterrupt(interrupt2, Read_FIFO_ISR, RISING); // Interrupt on Pin 7 //TBR Isn't this interrupt on pin 2?
}

void MODE_SLEEP()
{
  Serial.println("MODE_SLEEP ROUTINE");
  VARIABLE_CURRENT_MODE = "mode_sleep";
}

void SLEEP_SIGNAL()
{
  Serial.println("SLEEP_SIGNAL ROUTINE");  //TBR This is 3 progressively shorter flashes?
  unsigned long time = millis();

  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(200,0,0)); // Moderately bright green color.
  pixels.show();
  while(millis() - time < 200); //TBR Perhaps convert to variables?
  //digitalWrite(ACTUATOR_SL,HIGH);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color. //TBR Is this actually off?
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
  Serial.println("NORMAL_SIGNAL ROUTINE");  //TBR This is 3 flashes of equal length with shorter delay in between?
  unsigned long time = millis();

  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.  //TBR Isn't this off?
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

/*
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
*/

void CALCULATE_QUEUE_STATISTICS()
{
  Serial.println("CALCULATE_QUEUE_STATISTICS ROUTINE");
  float temp_val = temp_FIFO_int[0] * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD; //TBR check arithmetic here

  //Calculate range and mean of sampled data
  min_val = temp_val;	//TBR this is the only one that shouldn't be initialised to zero, but surely there must be a better way to initialise 
  max_val = temp_val;	//TBR this one doesn't need to be assigned, already initialised to zero
  mean = temp_val;	//TBR this one doesn't need to be assigned, already initialised to zero

  for(uint8_t i=1; i<VARIABLE_QUEUE_DECELERATION_SIZE; i++) //was: for(uint8_t i=1; i<FIFO_STATUS; i++)
  {
    //Serial.println(temp_val);
    //delay(1000);
    temp_val = temp_FIFO_int[i] * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    if(temp_val < min_val)
    {
      min_val = temp_val;
    }
    if(temp_val > max_val)
    {
      max_val = temp_val;
    }
    mean = mean + temp_val;  //TBR New value for mean to differentiate?
  }

  mean = mean/VARIABLE_QUEUE_DECELERATION_SIZE; //was: mean = mean/FIFO_STATUS;
  Serial.print("Queue mean: ");
  Serial.println(mean);
  range = max_val - min_val;
  Serial.print("Queue range: ");
  Serial.println(range);
}

void EVALUATE_DECELERATION()  //TBR Never actually called that I can see?
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

