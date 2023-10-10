#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <SoftwareSerial.h>
#include <MQUnifiedsensor.h>

#define SEALEVELPRESSURE_HPA (1013.25)
/************************Hardware Related Macros************************************/
#define Board ("ESP32")
#define Pin (A0)  //Analog input
/***********************Software Related Macros************************************/
#define Type ("MQ-2")  //MQ2
#define Voltage_Resolution (3.3)
#define ADC_Bit_Resolution (12)  // For ESP32
#define RatioMQ2CleanAir (9.83)  //RS / R0 = 9.83 ppm

/*****************************Globals***********************************************/
MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);
/*****************************Globals***********************************************/

#define SCREEN_WIDTH 128  // OLED display width,  in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_BME680 bme;  // I2C
SoftwareSerial pmsSerial(D6, D7);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //BME setup
  while (!Serial)
    ;
  Serial.println(F("BME680 test"));

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1)
      ;
  }
  pmsSerial.begin(9600);

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(3263, 150);  // 320*C for 150 ms

  // initialize OLED display with address 0x3C for 128x64
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true)
      ;
  }

  MQ2.setRegressionMethod(1);  //_PPM =  a*ratio^b
  MQ2.setA(574.25);
  MQ2.setB(-2.222);  // Configure the equation to to calculate LPG concentration
  /*
    Exponential regression:
    Gas    | a      | b
    H2     | 987.99 | -2.162
    LPG    | 574.25 | -2.222
    CO     | 36974  | -3.109
    Alcohol| 3616.1 | -2.675
    Propane| 658.71 | -2.168
  */
  MQ2.init();
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ2.update();  // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0 / 10);
  Serial.println("  done!.");

  if (isinf(calcR0)) {
    Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while (1)
      ;
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while (1)
      ;
  }
  /*****************************  MQ CAlibration ********************************************/

  MQ2.serialDebug(true);

  delay(2000);                           // wait for initializing
  oled.clearDisplay();                   // clear display
  oled.setTextSize(1);                   // text size
  oled.setTextColor(WHITE);              // text color
  oled.setCursor(0, 0);                  // position to display
  oled.println("Screen functional :)");  // text to display
  oled.display();                        // show on OLED
  delay(1000);
  oled.clearDisplay();
}

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data data;

void loop() {
  // put your main code here, to run repeatedly:
  if (!bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    //return;
  }
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");
  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");
  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");
  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");
  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  if (readPMSdata(&pmsSerial)) {
    // reading data was successful!
    Serial.println();
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (standard)");
    Serial.print("PM 1.0: ");
    Serial.print(data.pm10_standard);
    Serial.print("\t\tPM 2.5: ");
    Serial.print(data.pm25_standard);
    Serial.print("\t\tPM 10: ");
    Serial.println(data.pm100_standard);
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (environmental)");
    Serial.print("PM 1.0: ");
    Serial.print(data.pm10_env);
    Serial.print("\t\tPM 2.5: ");
    Serial.print(data.pm25_env);
    Serial.print("\t\tPM 10: ");
    Serial.println(data.pm100_env);
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:");
    Serial.println(data.particles_03um);
    Serial.print("Particles > 0.5um / 0.1L air:");
    Serial.println(data.particles_05um);
    Serial.print("Particles > 1.0um / 0.1L air:");
    Serial.println(data.particles_10um);
    Serial.print("Particles > 2.5um / 0.1L air:");
    Serial.println(data.particles_25um);
    Serial.print("Particles > 5.0um / 0.1L air:");
    Serial.println(data.particles_50um);
    Serial.print("Particles > 10.0 um / 0.1L air:");
    Serial.println(data.particles_100um);
    Serial.println("---------------------------------------");
  }

  MQ2.update();  // Update data, the arduino will read the voltage from the analog pin

  MQ2.setA(987.99);
  MQ2.setB(-2.162);  //H2 concentration calculation values
  float H2 = MQ2.readSensor();

  MQ2.setA(574.25);
  MQ2.setB(-2.222);              //LPG concentration calculation values
  float LPG = MQ2.readSensor();  // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ2.setA(36974);
  MQ2.setB(-3.109);             // CO concentration calculation values
  float CO = MQ2.readSensor();  // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ2.setA(3616.1);
  MQ2.setB(-2.675);                  // Alcohol concentration calculation values
  float Alcohol = MQ2.readSensor();  // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ2.setA(658.71);
  MQ2.setB(-2.168);                  // Propane concentration calculation value
  float Propane = MQ2.readSensor();  // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  //MQ2 Display
  Serial.print("|H2: ");
  Serial.print(H2);
  Serial.print(" |LPG: ");
  Serial.print(LPG);
  Serial.print(" |CO: ");
  Serial.print(CO);
  Serial.print(" |Alcohol ");
  Serial.print(Alcohol);
  Serial.print(" |Propane: ");
  Serial.print(Propane);
  Serial.println("   |");

  oled.setTextSize(2);
  oled.setCursor(20, 10);
  oled.println("H2: ");
  oled.print(H2);
  oled.println(" PPM");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(20, 10);
  oled.println("LPG: ");
  oled.print(LPG);
  oled.println(" PPM");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(20, 10);
  oled.println("CO: ");
  oled.print(CO);
  oled.println(" PPM");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);
  oled.println("Alcohol: ");
  oled.print(Alcohol);
  oled.println(" PPM");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);
  oled.println("Propane: ");
  oled.print(Propane);
  oled.println(" PPM");
  oled.display();
  delay(2000);

  //BME688 Display
  oled.clearDisplay();
  oled.setCursor(20, 10);
  oled.println("TEMP:");
  oled.print(bme.temperature);
  oled.println(" *C");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);
  oled.println("PRESSURE: ");
  oled.println(bme.pressure / 100.0);
  oled.println(" hPa");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);
  oled.println("HUMIDITY: ");
  oled.print(bme.humidity);
  oled.println(" %");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(20, 10);
  oled.println("GAS: ");
  oled.println(bme.gas_resistance / 1000.0);
  oled.println(" KOhms");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);
  oled.println("ALTITUDE: ");
  oled.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  oled.println(" m");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);

  //PM5003 Display
  oled.println("Conc STD");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);
  oled.print("PM 1.0:");
  oled.println(data.pm10_standard);
  oled.print("PM 2.5:");
  oled.println(data.pm25_standard);
  oled.print("PM 10:");
  oled.println(data.pm100_standard);
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);

  oled.println("Conc ENV");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);
  oled.print("PM 1.0:");
  oled.println(data.pm10_env);
  oled.print("PM 2.5:");
  oled.println(data.pm25_env);
  oled.print("PM 10:");
  oled.println(data.pm100_env);

  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);

  oled.println("Particle Concentration um/0.1L");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);

  oled.print("P>0.3:");
  oled.println(data.particles_03um);
  oled.print("P>0.5:");
  oled.println(data.particles_05um);
  oled.print("P>1.0:");
  oled.println(data.particles_10um);
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);

  oled.print("P>2.5:");
  oled.println(data.particles_25um);
  oled.print("P>5.0:");
  oled.println(data.particles_50um);
  oled.print("P>10.0:");
  oled.println(data.particles_100um);
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);


  oled.clearDisplay();
}

boolean readPMSdata(Stream *s) {
  if (!s->available()) {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }


  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}
