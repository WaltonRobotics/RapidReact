
#include <Wire.h>

#define LED_RED 7540.0f
#define LED_GREEN 14470.0f
#define LED_BLUE 7270.0f

#define BAL_RED (LED_GREEN/LED_RED)
#define BAL_GREEN (LED_GREEN/LED_GREEN)
#define BAL_BLUE (LED_GREEN/LED_BLUE)

#define TCAADDR 0x70
#define I2C_ADDR 0x52

uint8_t readBuff[9];
uint16_t ir=0;
uint16_t red=0;
uint16_t green=0;
uint16_t blue=0;
uint16_t red0total=0;
uint16_t red1total=0;
uint16_t red2total=0;
uint16_t green0total=0;
uint16_t green1total=0;
uint16_t green2total=0;
uint16_t blue0total=0;
uint16_t blue1total=0;
uint16_t blue2total=0;

uint16_t red0base=0;
uint16_t green0base=0;
uint16_t red1base=0;
uint16_t green1base=0;
uint16_t red2base=0;
uint16_t green2base=0;

uint16_t reddiff = 30;
uint16_t greendiff = 25;
int count = 1;
const int pin0 = 14;
const int pin1 = 12;
const int pin2 = 13;

void tcaselect (uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();
}

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
  
  tcaselect(0);
  i2cWrite(0x00,0b0110);
  i2cWrite(0x04,0b01000000);
  pinMode(pin0, OUTPUT);
  
  tcaselect(1);
  i2cWrite(0x00,0b0110);
  i2cWrite(0x04,0b01000000);
  pinMode(pin1, OUTPUT);
  
  tcaselect(2);
  i2cWrite(0x00,0b0110);
  i2cWrite(0x04,0b01000000);
  pinMode(pin2, OUTPUT);
}

void loop() {
  
tcaselect(0);
  uint16_t redlocal=0;
  uint16_t greenlocal=0;

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

  red0total = red0total + red;
  green0total = green0total + green;
  blue0total = blue0total + blue;

  if (count == 5) {
    redlocal = red0total/5;
    greenlocal = green0total/5;
    
    if (red0base == 0) {
      red0base = redlocal;
      Serial.println(red0base);
      green0base = greenlocal;
      Serial.println(green0base);
    }

    if (red0base - redlocal >= reddiff && green0base - greenlocal >= greendiff) {
      Serial.println("Sensor 0: Black Tape detected");
      digitalWrite(pin0, HIGH);
    } else {
      Serial.println("Sensor 0: Carpet");
      digitalWrite(pin0, LOW);
    }
    
    /*Serial.print("Red0: ");
    Serial.print(red0total/5);
  
    Serial.print(" Green0: ");
    Serial.print(green0total/5);
  
    Serial.print(" Blue0: ");
    Serial.println(blue0total/5);*/
  }
  
  /*if(red >395 && green > 373 && blue > 420){
   Serial.print("1");
   }
  else{
   Serial.print("0");
   }
  delay(100);*/

tcaselect(1);
  i2cRead(0x0A,readBuff,12);
  ir=(readBuff[1]<<8)|readBuff[0];
  green=(readBuff[4]<<8)|readBuff[3];
  blue=(readBuff[7]<<8)|readBuff[6];
  red=(readBuff[10]<<8)|readBuff[9];
  red*=BAL_RED;
  green*=BAL_GREEN;
  blue*=BAL_BLUE;
  maxV=max(blue,max(red,green));
  red=255*pow(red/maxV,3.3);
  green=255*pow(green/maxV,3.3);
  blue=255*pow(blue/maxV,3.3);

  red1total = red1total + red;
  green1total = green1total + green;
  blue1total = blue1total + blue;

  if (count == 5){
    // set baseline if not set
    redlocal = red1total/5;
    greenlocal = green1total/5;
    
    if (red1base == 0) {
      red1base = redlocal;
    }
    if (green1base == 0){
      green1base = greenlocal;
    }

    if (red1base - redlocal >= reddiff && green1base - greenlocal >= greendiff) {
      Serial.println("Sensor 1: Black Tape detected");
      digitalWrite(pin1, HIGH);
    } else {
      Serial.println("Sensor 1: Carpet");
      digitalWrite(pin1, LOW);
    }
          
    /*Serial.print("Red1: ");
    Serial.print(red1total/5);
    Serial.print(" Green1: ");
    Serial.print(green1total/5);
  
    Serial.print(" Blue1: ");
    Serial.println(blue1total/5);*/
  }


  /*if(red < 600 && green < 600 && blue < 630){
   Serial.print("1");
   }
  else{
   Serial.print("0");
   }
   delay(100);*/
  
tcaselect(2);
  i2cRead(0x0A,readBuff,12);
  ir=(readBuff[1]<<8)|readBuff[0];
  green=(readBuff[4]<<8)|readBuff[3];
  blue=(readBuff[7]<<8)|readBuff[6];
  red=(readBuff[10]<<8)|readBuff[9];
  red*=BAL_RED;
  green*=BAL_GREEN;
  blue*=BAL_BLUE;
  maxV=max(blue,max(red,green));
  red=255*pow(red/maxV,3.3);
  green=255*pow(green/maxV,3.3);
  blue=255*pow(blue/maxV,3.3);

  red2total = red2total + red;
  green2total = green2total + green;
  blue2total = blue2total + blue;

  if (count == 5){
    redlocal = red2total/5;
    greenlocal = green2total/5;
    
    if (red2base == 0) {
      red2base = redlocal;
      green2base = greenlocal;
    }

    if (red2base - redlocal >= reddiff && green2base - greenlocal >= greendiff) {
      Serial.println("Sensor 2: Black Tape detected");
      digitalWrite(pin2, HIGH);
    } else {
      Serial.println("Sensor 2: Carpet");
      digitalWrite(pin2, LOW);
    }
  
  /*Serial.print("Red2: ");
  Serial.print(red2total/5);

  Serial.print(" Green2: ");
  Serial.print(green2total/5);

  Serial.print(" Blue2: ");
  Serial.println(blue2total/5);*/
  }

  if (count == 5) {
    count = 1;
    // set all totals to 0
    red0total=0;
    red1total=0;
    red2total=0;
    green0total=0;
    green1total=0;
    green2total=0;
    blue0total=0;
    blue1total=0;
    blue2total=0;
  } else {
    count++;
  }
  delay(50);
 /*if(red > 510 && green > 490 && blue > 590){
   Serial.print("1");
   }
  else{
   Serial.print("0");
   }
   delay(100);

   Serial.println();*/
}
