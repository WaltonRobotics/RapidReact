#include <Wire.h>

#define LED_RED 7540.0f
#define LED_GREEN 14470.0f
#define LED_BLUE 7270.0f

#define BAL_RED (LED_GREEN/LED_RED)
#define BAL_GREEN (LED_GREEN/LED_GREEN)
#define BAL_BLUE (LED_GREEN/LED_BLUE)

#define I2C_ADDR 0x52

uint8_t readBuff[9];
uint16_t ir=0;
uint16_t red=0;
uint16_t green=0;
uint16_t blue=0;

void i2cWrite(uint8_t reg, uint8_t val){
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void i2cRead(uint8_t reg,uint8_t *val,uint16_t len){
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(I2C_ADDR, len);
    for(uint8_t i=0;i<len;i++){
      val[i]=Wire.read();
    }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  i2cWrite(0x00,0b0110);
  i2cWrite(0x04,0b01000000);
}

void loop() {
  i2cRead(0x0A,readBuff,12);
 
  ir=(readBuff[1]<<8)|readBuff[0];
  green=(readBuff[4]<<8)|readBuff[3];
  blue=(readBuff[7]<<8)|readBuff[6];
  red=(readBuff[10]<<8)|readBuff[9];
 
  red*=BAL_RED;
  green*=BAL_GREEN;
  blue*=BAL_BLUE;

  float maxV=max(blue,max(red,green));
  red=255*pow(red/maxV,3.3);
  green=255*pow(green/maxV,3.3);
  blue=255*pow(blue/maxV,3.3);
 
  
  // print sensor results to serial output
  Serial.print("Red: ");
  Serial.print(red);

  Serial.print(" Green: ");
  Serial.print(green);

  Serial.print(" Blue: ");
  Serial.println(blue);
  
  delay(25);

}
