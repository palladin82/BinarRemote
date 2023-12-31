#include <U8g2lib.h>
#include <Wire.h>

#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define SCREEN_WIDTH 128 //display width, in pixels
#define SCREEN_HEIGHT 64 //display height, in pixels
U8G2_SSD1306_128X64_NONAME_F_HW_I2C *u8g2 = nullptr;
#define I2C_SDA                     4
#define I2C_SCL                     15
#define OLED_RST                    16

char CurMenu[256];

char *GetStatus(int Status);
extern ESP32Time rtc;
unsigned long prevEpoch;
int prevSNR;

extern float LoRaSNR;
#define bat_width  8
#define bat_height 11


static unsigned char image_ant[] = {
 0x00,0xf8,0x00,0xfe,0x00,0xfe,0xc0,0xfe,0xc0,0xfe,0xd8,0xfe,
 0xd8,0xfe,0xdb,0xfe,0xdb,0xfe,0xdb,0xfe,0x00,0xf8};

 static unsigned char image_ant_inv[] = {
 0xff,0xff,0xff,0xf9,0xff,0xf9,0x3f,0xf9,0x3f,0xf9,0x27,0xf9,
 0x27,0xf9,0x24,0xf9,0x24,0xf9,0x24,0xf9,0xff,0xff};


static unsigned char image_bati[] = {
 0xff,0xe7,0x81,0xbd,0xbd,0xbd,0x81,0x81,0x81,0x81,0x81};

static unsigned char image_batt[] = {
 0x00,0x18,0x7e,0x42,0x42,0x42,0x7e,0x7e,0x7e,0x7e,0x7e};

static unsigned char image_temp[] = {
 0x08,0x3c,0x14,0x34,0x14,0x34,0x14,0x3e,0x2e,0x3e,0x1c};

static unsigned char image_tempi[] = {
 0xf7,0xc3,0xeb,0xcb,0xeb,0xcb,0xeb,0xc1,0xd1,0xc1,0xe3};

static unsigned char logo_bits[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xF8, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x01, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x7E, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x1F, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 
  0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x80, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x0F, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 
  0x0F, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x30, 0x38, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x3F, 0x00, 0x00, 0x00, 
  0x00, 0x30, 0x30, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF4, 
  0x4F, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xE0, 0x8F, 0x00, 0x00, 0x00, 0x00, 0x18, 0x30, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x1F, 0x00, 0x00, 0x00, 
  0x00, 0x18, 0x18, 0x08, 0x78, 0xE0, 0x01, 0x1C, 0x00, 0x00, 0x00, 0xF0, 
  0x1F, 0x00, 0x00, 0x00, 0x00, 0x18, 0x06, 0x86, 0xC3, 0x18, 0xC3, 0x01, 
  0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x03, 0x86, 
  0xC1, 0x08, 0xC3, 0x00, 0x80, 0x3F, 0x00, 0x80, 0x1F, 0x00, 0x00, 0x00, 
  0x00, 0x0C, 0x0E, 0x86, 0xC1, 0x00, 0xC3, 0x00, 0x40, 0xFF, 0x01, 0xF0, 
  0x1F, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x82, 0x40, 0x80, 0x41, 0x00, 
  0x00, 0xB2, 0x0F, 0xFC, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x06, 0x18, 0xC3, 
  0x60, 0x80, 0x61, 0x00, 0x00, 0x00, 0xBC, 0xFF, 0x7F, 0x00, 0x00, 0x00, 
  0x00, 0x06, 0x0C, 0xC3, 0x60, 0x86, 0x61, 0x00, 0x00, 0x18, 0xF0, 0xFF, 
  0xFF, 0x01, 0x00, 0x00, 0x00, 0x06, 0x0C, 0x41, 0x20, 0x83, 0x20, 0x00, 
  0x00, 0x00, 0x00, 0xFC, 0xFF, 0x07, 0x00, 0x00, 0x00, 0x03, 0x86, 0x61, 
  0x30, 0xC1, 0x30, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFF, 0x0F, 0x00, 0x00, 
  0x00, 0x03, 0x83, 0x61, 0x30, 0xE3, 0x30, 0x00, 0x00, 0x00, 0x00, 0xC0, 
  0xFF, 0x0F, 0x00, 0x00, 0x80, 0xFF, 0xC0, 0x71, 0x3C, 0xFF, 0x78, 0x00, 
  0x00, 0x00, 0x00, 0x80, 0xFF, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0x01, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x80, 
  0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0xF8, 0x8F, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7A, 0xF8, 0xFF, 0x03, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3A, 0x80, 
  0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x44, 0x00, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x03, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0xFE, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x60, 0x00, 0x00, 0xFE, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0xFE, 0x0F, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0xFF, 0x1F, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0xF0, 0x1F, 0x00, 0x00, 
  0x00, 0x07, 0x00, 0x80, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 
  0x30, 0x38, 0x00, 0x00, 0x00, 0x0F, 0x06, 0x80, 0xFF, 0x3F, 0x00, 0x00, 
  0x00, 0x30, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0xCF, 
  0xFF, 0x3F, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 
  0xF8, 0xFF, 0xFF, 0xC7, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 
  0x18, 0x38, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0xCF, 0xFF, 0xFF, 0x00, 0x00, 
  0x00, 0x18, 0x00, 0x1E, 0x18, 0x18, 0x78, 0x00, 0xFE, 0xFF, 0xFF, 0xEF, 
  0xFF, 0xFF, 0x00, 0x00, 0x00, 0x18, 0x00, 0x21, 0x18, 0x0E, 0xC6, 0x00, 
  0xFE, 0xFF, 0xFF, 0xEF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x0C, 0x80, 0x60, 
  0xFC, 0x03, 0xC2, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 
  0x00, 0x0C, 0x40, 0x60, 0x0C, 0x01, 0xC0, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0x00, 0x00, 0x00, 0x0E, 0x60, 0x60, 0x0C, 0x03, 0x60, 0x80, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x06, 0x20, 0x60, 
  0x06, 0x03, 0x60, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x01, 0x00, 
  0x00, 0x06, 0x20, 0x20, 0x06, 0x83, 0x61, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0x7F, 0x02, 0x00, 0x00, 0x07, 0x30, 0x30, 0x06, 0xC3, 0x20, 0x80, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0x03, 0x20, 0x18, 
  0x03, 0x43, 0x30, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 
  0x00, 0x03, 0x66, 0x0C, 0x03, 0xC3, 0x38, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0x7F, 0x00, 0x00, 0x80, 0xFF, 0xC3, 0x83, 0x03, 0xC3, 0x3F, 0x80, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0xFF, 
  0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0x0F, 0xFF, 0x3F, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 
  0x03, 0xD8, 0x0F, 0x00 };











String intToString(int num)
{
    String convert = String(num);
    //itoa(num,cstr,10);
    
    return convert;

}

void init_oled()
{
  //reset display via software

  Wire.begin(I2C_SDA, I2C_SCL);

  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, HIGH); delay(20);
  digitalWrite(OLED_RST, LOW);  delay(20);
  digitalWrite(OLED_RST, HIGH); delay(20);


  Wire.beginTransmission(0x3C);
  if (Wire.endTransmission() == 0) {
    //Serial.println("Started OLED");
    u8g2 = new U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R2, U8X8_PIN_NONE);
    u8g2->begin();
    u8g2->enableUTF8Print();
    //enableUTF8Print();

    u8g2->clearBuffer();
    u8g2->setFlipMode(0);
    u8g2->setFontMode(1); // Transparent
    
    u8g2->firstPage();
    
   
    
    u8g2->setFont(u8g2_font_haxrcorp4089_t_cyrillic);

    u8g2->clearBuffer();
    
    
    //delay(1000);
    u8g2->drawBox(0, 0, 128, 64);
    u8g2->setDrawColor(1);
    u8g2->drawXBMP( 0, 0, 128, 64, logo_bits);
    u8g2->sendBuffer();
    delay(3000);
    u8g2->setDisplayRotation(U8G2_R1);
    //u8g2->drawStr(10, 125, fbat);


  }  
  //delay(1000);
  u8g2->clearBuffer();
}



void displayMsg(char* msg)
{
  u8g2->setDrawColor(1);
  u8g2->drawBox(0, 60, 64, 13);
  u8g2->setDrawColor(0);
  //u8g2->drawStr(0, 70, msg);
  u8g2->setCursor(0, 70);
  u8g2->print(msg);
  u8g2->sendBuffer();
  u8g2->setDrawColor(1);
}

void displayMsgS0(String msg)
{
  char chars[msg.length()+1];
  msg.toCharArray(chars,msg.length()+1);
  u8g2->setDrawColor(1);
  u8g2->drawBox(0, 50, 64, 13);
  u8g2->setDrawColor(0);
  //u8g2->drawStr(0, 80, chars);
  u8g2->setCursor(0, 60);
  u8g2->print(chars);
  u8g2->sendBuffer();
  u8g2->setDrawColor(1);
}

void displayMsgS(String msg)
{
  char chars[msg.length()+1];
  msg.toCharArray(chars,msg.length()+1);
  u8g2->setDrawColor(1);
  u8g2->drawBox(0, 60, 64, 13);
  u8g2->setDrawColor(0);
  //u8g2->drawStr(0, 70, chars);
  u8g2->setCursor(0, 70);
  u8g2->print(chars);
  u8g2->sendBuffer();
  u8g2->setDrawColor(1);
}

void displayMsgS1(String msg)
{
  char chars[msg.length()+1];
  msg.toCharArray(chars,msg.length()+1);
  u8g2->setDrawColor(1);
  u8g2->drawBox(0, 70, 64, 13);
  u8g2->setDrawColor(0);
  //u8g2->drawStr(0, 80, chars);
  u8g2->setCursor(0, 80);
  u8g2->print(chars);
  u8g2->sendBuffer();
  u8g2->setDrawColor(1);
}

void displayMsgS2(String msg)
{
  char chars[msg.length()+1];
  msg.toCharArray(chars,msg.length()+1);
  u8g2->setDrawColor(1);
  u8g2->drawBox(0, 80, 64, 13);
  u8g2->setDrawColor(0);
  //u8g2->drawStr(0, 80, chars);
  u8g2->setCursor(0, 90);
  u8g2->print(chars);
  u8g2->sendBuffer();
  u8g2->setDrawColor(1);
}


void displayBat(uint8_t msg)
{
  float battery=(float)msg/10;
  char fbat[6];
  sprintf(fbat, "%.1f", battery);
  
  
  u8g2->setDrawColor(0);
  u8g2->drawBox(0, 115, 32, 13);
  u8g2->setDrawColor(1);
  u8g2->drawXBM( 0, 115, bat_width, bat_height, image_batt);
  //u8g2->drawStr(10, 125, fbat);
  u8g2->setCursor(10, 125);
  u8g2->print(fbat);
  u8g2->sendBuffer();
  u8g2->setDrawColor(1);
}

void displayTemp(char* msg)
{
  u8g2->setDrawColor(0);
  u8g2->drawBox(0, 103, 64, 13);
  u8g2->setDrawColor(1);
  u8g2->drawXBM( 0, 103, bat_width, bat_height, image_temp);
  //u8g2->drawStr(10, 113, msg);
  u8g2->setCursor(10, 113);
  u8g2->print(msg);
  u8g2->sendBuffer();
  u8g2->setDrawColor(1);
}


void displayX()
{  
  /*u8g2->setDrawColor(0);
  u8g2->drawBox(0, 90, 15, 13);
  u8g2->setDrawColor(1);  
  //u8g2->drawStr(0, 100, "X");
  u8g2->setCursor(0, 100);
  u8g2->print("X");

  u8g2->sendBuffer();
  delay(30);
  u8g2->setDrawColor(0);
  u8g2->drawBox(0, 90, 15, 13);
  u8g2->sendBuffer();
  u8g2->setDrawColor(1);*/
}

void displayDateTime()
{
  //u8g2->nextPage();
  if(prevEpoch<rtc.getEpoch())
  {
    u8g2->setDrawColor(1);
    u8g2->drawBox(0, 0, 32, 11);
    u8g2->setDrawColor(0);
    u8g2->setCursor(0, 10);
    char DT[255];  
    sprintf(DT,"%02d:%02d",rtc.getHour(true),rtc.getMinute());
    u8g2->print(DT);
    u8g2->sendBuffer();
    prevEpoch=rtc.getEpoch();
  }

  //Отобразить snr столбиками
  //u8g2->setCursor(53, 10);
  if(prevSNR!=LoRaSNR)
  {
    //u8g2->drawBox(0, 0, 32, 11);
    u8g2->setDrawColor(1);
    u8g2->drawBox(32, 0, 32, 11);
    u8g2->setDrawColor(0);
    u8g2->drawXBM( 52, 0, 11, 11, image_ant);
    //u8g2->sendBuffer(); 

    u8g2->setDrawColor(1);

    if(LoRaSNR<=10)
    {    
      u8g2->drawBox(64, 0, 64, 11);
    }
    if(LoRaSNR<=5)
    {    
      u8g2->drawBox(62, 0, 64, 11);
    }
    if(LoRaSNR<=0)
    {    
      u8g2->drawBox(59, 0, 64, 11);
    }
    if(LoRaSNR<=-5)
    {    
      u8g2->drawBox(58, 0, 64, 11);
    }
    if(LoRaSNR<=-10)
    {    
      u8g2->drawBox(54, 0, 64, 11);
    }
    u8g2->sendBuffer();  
    //u8g2->firstPage();
    prevSNR=LoRaSNR;
  }
}


void displayRSSI()
{  
  u8g2->setDrawColor(0);
  
  u8g2->drawBox(0, 90, 15, 13);
  u8g2->setDrawColor(1);  
  
  u8g2->setCursor(0, 100);
  u8g2->print("X");

  u8g2->sendBuffer();
  delay(30);
  u8g2->setDrawColor(0);
  u8g2->drawBox(0, 90, 15, 13);
  u8g2->sendBuffer();
  u8g2->setDrawColor(1);
}

void displayStatus(char *msg)
{
  u8g2->setDrawColor(0);
  u8g2->drawBox(35, 115, 64, 13);
  u8g2->setDrawColor(1);
  //u8g2->drawStr(35, 125, GetStatus(atoi(msg)));
  u8g2->setCursor(35, 125);
  u8g2->print(GetStatus(atoi(msg)));
  u8g2->sendBuffer();
  u8g2->setDrawColor(1);
}


char *GetStatus(int Status)
    {
          static char msg[9];
          switch(Status)
          {
            case 1: sprintf(msg,"Off");return msg; break;
            case 2: sprintf(msg,"Starting");return msg; break;
            case 3: sprintf(msg,"Warm");return msg; break;
            case 4: sprintf(msg,"Heat");return msg; break;
            case 5: sprintf(msg,"Purge");return msg; break;
            case 6: sprintf(msg,"Shut");return msg; break;            
            default: sprintf(msg,"E-%d",Status);return msg; break;
          }
          return msg;
    }

/*void displayValue(SimpleMenu *_menu)
{

  //u8g2->clearBuffer();
  u8g2->setDrawColor(1);
  u8g2->drawBox(0, 0, 64, 13);

  char buf[256];
  snprintf(buf, sizeof(buf), "%i", _menu->getUValue());
  u8g2->setDrawColor(0);
  
  //u8g2->drawStr(0, 10, buf);
  u8g2->setCursor(0, 10);
  u8g2->print(buf);

  u8g2->sendBuffer();
  u8g2->setDrawColor(1);

}*/




