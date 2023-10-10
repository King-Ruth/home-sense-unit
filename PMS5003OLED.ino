#include <Adafruit_SSD1306.h>
#include <splash.h>

#include <Adafruit_GFX.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>

#include <Wire.h>


#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#include <SoftwareSerial.h>
SoftwareSerial pmsSerial(D1, D2);

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pmsSerial.begin(9600);
  // initialize OLED display with address 0x3C for 128x64
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
  {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }

  delay(2000);         // wait for initializing
  oled.clearDisplay(); // clear display

  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 0);        // position to display
  oled.println("Hello World!"); // text to display
  oled.display();               // show on OLED
  delay(2000);


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
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setCursor(0, 0);
  oled.print("lOOPING NOW");
  oled.display();
  delay(1000);
  oled.clearDisplay();

   if (readPMSdata(&pmsSerial)) {
    // reading data was successful!
    Serial.println();
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (standard)");
    Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
    Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
    Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (environmental)");
    Serial.print("PM 1.0: "); Serial.print(data.pm10_env);
    Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);
    Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
    Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
    Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
    Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
    Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
    Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.particles_100um);
    Serial.println("---------------------------------------");
  }

  //PM5003 Display
  oled.setTextSize(2);
  oled.setCursor(0, 10);
  oled.println("Conc STD");
  oled.println("PM 1.0:");
  oled.print(data.pm10_standard);
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);

  oled.println("PM 2.5:");
  oled.print(data.pm25_standard);
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);

  oled.println("PM 10:");
  oled.print(data.pm100_standard);
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);
  oled.println("----");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);

  oled.println("Conc ENV");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);
  oled.println("PM 1.0:");
  oled.print(data.pm10_env);
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);

  oled.println("PM 2.5:");
  oled.print(data.pm25_env);
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);

  oled.println("PM 10:");
  oled.print(data.pm100_env);

  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);

  oled.println("----");
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);

  oled.println("P > 0.3um/0.1L:");
  oled.print(data.particles_03um);
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);

  oled.println("P > 0.5um/0.1L:");
  oled.print(data.particles_05um);
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);

  oled.println("P > 1.0um/0.1L:");
  oled.print(data.particles_10um);
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);

  oled.println("P > 2.5um/0.1L:");
  oled.print(data.particles_25um);
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);

  oled.println("P > 5.0um/0.1L:");
  oled.print(data.particles_50um);
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);

  oled.print("P > 10.0 um/0.1L:");
  oled.println(data.particles_100um);
  oled.display();
  delay(2000);
  oled.clearDisplay();
  oled.setCursor(0, 10);


  oled.clearDisplay();



}

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
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
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }
 
  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
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
