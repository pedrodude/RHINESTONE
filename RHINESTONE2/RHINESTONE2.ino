#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_NeoPixel.h>

uint8_t PIN = 4;
uint8_t NUMPIXELS = 1;
uint8_t spi_clock = 13; // aka SCL
uint8_t spi_miso = 12; // aka SDO
uint8_t spi_mosi = 11; // aka SDA
uint8_t spi_cs = 10;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(spi_clock, spi_miso, spi_mosi, spi_cs, 12345);
sensors_event_t event;

void setup() {

  pixels.begin();

  Serial.begin(9600);

  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Read Device ID:  "); Serial.println(accel.getDeviceID());
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
  
  if(!accel.begin())
  {
    Serial.println("Problem detecting the ADXL345 - check connections");
    while(1); //All this does is fails forever, maybe get it to retry?
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_4_G);

  pixels.setPixelColor(0, pixels.Color(0,150,0)); // Moderately bright green color.
  pixels.show();
}

void loop() {
  
  accel.getEvent(&event);

  Serial.println(event.acceleration.x);
  Serial.println(event.acceleration.y);
  Serial.println(event.acceleration.z);

  delay(2000);

}
