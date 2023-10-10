#include <Adafruit_SSD1306.h>
#include <splash.h>

#include <Adafruit_GFX.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>

#include <Wire.h>

#include <MQUnifiedsensor.h>

/************************Hardware Related Macros************************************/
#define         Board                   ("ESP32")
#define         Pin                     (A0)  //Analog input 
/***********************Software Related Macros************************************/
#define         Type                    ("MQ-2") //MQ2
#define         Voltage_Resolution      (3.3)
#define         ADC_Bit_Resolution      (12) // For ESP32
#define         RatioMQ2CleanAir        (9.83) //RS / R0 = 9.83 ppm 

/*****************************Globals***********************************************/
MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);
/*****************************Globals***********************************************/

#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // initialize OLED display with address 0x3C for 128x64
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
  {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }

  MQ2.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ2.setA(574.25); MQ2.setB(-2.222); // Configure the equation to to calculate LPG concentration
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
  for(int i = 1; i<=10; i ++)
  {
    MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
  /*****************************  MQ CAlibration ********************************************/ 

  MQ2.serialDebug(true);

  delay(2000);         // wait for initializing
  oled.clearDisplay(); // clear display
  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 0);        // position to display
  oled.println("Screen functional :)"); // text to display
  oled.display();               // show on OLED
  delay(1000);
  oled.clearDisplay();


}

void loop() {
  // put your main code here, to run repeatedly:

  MQ2.update(); // Update data, the arduino will read the voltage from the analog pin

  MQ2.setA(987.99); MQ2.setB(-2.162); //H2 concentration calculation values
  float H2 = MQ2.readSensor(); 

  MQ2.setA(574.25); MQ2.setB(-2.222); //LPG concentration calculation values
  float LPG = MQ2.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ2.setA(36974); MQ2.setB(-3.109); // CO concentration calculation values
  float CO = MQ2.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ2.setA(3616.1); MQ2.setB(-2.675); // Alcohol concentration calculation values
  float Alcohol = MQ2.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  
  MQ2.setA(658.71); MQ2.setB(-2.168); // Propane concentration calculation value
  float Propane = MQ2.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  Serial.print("|H2: "); Serial.print(H2); 
  Serial.print(" |LPG: "); Serial.print(LPG);
  Serial.print(" |CO: "); Serial.print(CO); 
  Serial.print(" |Alcohol "); Serial.print(Alcohol); 
  Serial.print(" |Propane: "); Serial.print(Propane); 
  Serial.println("   |"); 

  oled.setTextSize(2);
  oled.setCursor(0, 10);
  oled.println("H2: "); oled.print(H2); oled.println("PPM");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);
  oled.println("LPG: "); oled.print(LPG); oled.println("PPM");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);
  oled.println("CO: "); oled.print(CO); oled.println("PPM");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);
  oled.println("Alcohol: "); oled.print(Alcohol); oled.println("PPM");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);
  oled.println("Propane: "); oled.print(Propane); oled.println("PPM");
  oled.display();
  delay(2000);
  oled.clearDisplay();

}
