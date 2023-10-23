/*
  LoRa Binar-5s remote communication

  created 08 October 2023
  by NikMarch
*/
#include <Arduino.h>
#include <SPI.h>              
#include <esp_task_wdt.h>
#include "soc/rtc_io_reg.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include <ESP32Time.h>


#include <OneButton.h>
#include <SimpleMenu.h>
#include <EEPROM.h>

#include "display.h"
#include "LoRaDriver.h"

#define BUTTON_PIN_BITMASK 0x000000001
#define PIN_INPUT 0
#define PIN_LED 2 // GIOP25 pin connected to led

int temp = 55;
unsigned int wakeflag=0;
int tick=0;

bool crcok=false;
bool pult=false;
RTC_DATA_ATTR int bootCount = 0;
ESP32Time rtc(10800); //gmt+3
tm TS;
bool timechanged=false;

const int lcdBrightness = 10; // (0-255)
float LoRaSNR=-20;

static const unsigned long REFRESH_INTERVAL = 5000; // ms
int timetosleep=12; // timetosleep * REFRESH_INTERVAL
static unsigned long lastRefreshTime = 0;

int commandQueue[255];
int commandQueueD=0;

OneButton button(PIN_INPUT, true);


// Variables will change:
int lastState = HIGH; // the previous state from the input pin
int currentState;
uint8_t debug=0;


int valueA = 1, valueB = 2, mainValue = 55;

void fStart();
void fStop();
void exitF();
void sendStatus();
void fMessage();
void setBw();
void sendPing();
void ToggleDebug();
void setSprd(int *param);
//void setbaud(int *baud);

SimpleMenu* ShowAllNext(SimpleMenu *menu, char *buf);



void ulp_start(void) 
{
    // Slow memory initialization
    memset(RTC_SLOW_MEM, 0, 8192);
    // if LED is connected to GPIO2 (specify by +14)
    const gpio_num_t ledPWMPin = (gpio_num_t) GPIO_NUM_2;//GPIO_NUM_2;
    
    const int ledPWMBit = RTCIO_GPIO2_CHANNEL+14;//RTCIO_GPIO2_CHANNEL + 14;
    // GPIOx initialization (set to output and initial value is 0)
    rtc_gpio_init(ledPWMPin);
    rtc_gpio_set_direction(ledPWMPin, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(ledPWMPin, 0);
    // Define ULP program
    const ulp_insn_t ulp_prog[] = 
    {
        M_LABEL(1),
        I_WR_REG(RTC_GPIO_OUT_REG, ledPWMBit, ledPWMBit, 1), // on
        I_DELAY(lcdBrightness * 100),
        I_WR_REG(RTC_GPIO_OUT_REG, ledPWMBit, ledPWMBit, 0), // off
        I_DELAY(25500 - lcdBrightness * 100),
        M_BX(1),
    };
    // Run ULP program
    size_t size = sizeof(ulp_prog) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, ulp_prog, &size);
    ulp_run(0);
}

void goto_deepsleep()
{
  
     

      if(debug)
      {
        Serial.println("going to sleep!");
        displayMsgS("going to sleep!");        
      }
      
      //pinMode(14, INPUT_PULLUP);
      //delay(2);


      //gpio_hold_en((gpio_num_t)14);


      //gpio_deep_sleep_hold_en();

      //if(esp_sleep_enable_ext0_wakeup(GPIO_NUM_26, HIGH)==ESP_OK) Serial.println("Lora sleep sleep configured");
      //else Serial.println("GPIO_26 sleep ERR");
      
      if(esp_sleep_enable_ext1_wakeup(GPIO_SEL_0, ESP_EXT1_WAKEUP_ALL_LOW)==ESP_OK) Serial.println("GPIO_button sleep configured"); //BUTTON_PIN_BITMASK
      else Serial.println("GPIO_button sleep ERR");
      
      //char tempSs[255];
      
      
      
      //ulp_start();
      //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
      
      LoRa.sleep();
      esp_deep_sleep_start();
}

int zero=0, one=1, four=4, five=5;
int menuMaxShow=5;
int Hour=-1;
int Minutes=-1;

SimpleMenu MenuSubSprd[] = {
  SimpleMenu("..", exitF),
  SimpleMenu("OFF", setSprd, 0),
  SimpleMenu("Старт", setSprd, 1),
  SimpleMenu("Нагрев", setSprd, 4),
  SimpleMenu("Остановка", setSprd, 5),
  //SimpleMenu("Waiting", CallbackP(setSprd, &zero)),
  //SimpleMenu("Waiting", CallbackP(setSprd, &one)),
  //SimpleMenu("Waiting", CallbackP(setSprd, &four)),
  //SimpleMenu("Waiting", CallbackP(setSprd, &five))
  
};

/*SimpleMenu MenuSubTM[] = {  
  SimpleMenu("00", setBw),
  SimpleMenu("01", setBw),
  SimpleMenu("02", setBw),
  SimpleMenu("03", setBw),
  SimpleMenu("04", setBw),
  SimpleMenu("05", setBw),
  SimpleMenu("06", setBw),
  SimpleMenu("125E3", setBw),
  SimpleMenu("250E3", setBw),
  SimpleMenu("500E3", setBw)
};*/

SimpleMenu MenuSub[2] = {  
  SimpleMenu("SetHour", &TS.tm_hour,0,24),
  SimpleMenu("SetMin", &TS.tm_min,0,60)
  //SimpleMenu("Debug-On", ToggleDebug)
};


SimpleMenu Menu[] = {
  //SimpleMenu("Cur Temp",&mainValue),
  SimpleMenu("Старт", fStart),
  SimpleMenu("Стоп", fStop),
  SimpleMenu("Запрос", fMessage),    
  SimpleMenu("Конфиг", 2, MenuSub)
};

SimpleMenu TopMenu(4, Menu);


void displayMenu(SimpleMenu *_menu)
{
  
  int menucnt = 10;
  SimpleMenu *next;
  SimpleMenu *prev;
  int index=_menu->getIndex();

  //prev = TopMenu.next(-1);
  
  u8g2->setDrawColor(0);
  u8g2->drawBox(0, 12, 64, 60);
  u8g2->setDrawColor(1);
  u8g2->sendBuffer();

  for(int i=-menucnt/2;i<=menucnt/2;i++)
  {
    prev = TopMenu.next(i);
    if(prev!=NULL)
    {
      menucnt += 11;
      if(i==index)
      {
        u8g2->setDrawColor(1);
        u8g2->drawBox(0, menucnt-9, 64, 11);
        u8g2->setDrawColor(0);
        char buf[256];
        snprintf(buf, sizeof(buf), "%s", prev->name);
        u8g2->setCursor(0, menucnt);
        u8g2->print(buf);
      }
      else
      {
        u8g2->setDrawColor(1);
        char buf[256];
        snprintf(buf, sizeof(buf), "%s", prev->name);
        u8g2->setCursor(0, menucnt);
        u8g2->print(buf);
      }
      //u8g2->drawStr(0, menucnt, buf);  //тоже работало
      u8g2->sendBuffer();
    }
  }
  



  /*u8g2->setDrawColor(0);
  u8g2->drawBox(0, 0, 64, 60);
  u8g2->setDrawColor(1);
  u8g2->drawBox(0, 0, 64, 12);  
  char buf[256];
  if(prev!=NULL)snprintf(buf, sizeof(buf), "%s", prev->name);  
  u8g2->setDrawColor(0);  
  u8g2->setCursor(0, menucnt);
  u8g2->print(buf);
  //u8g2->drawStr(0, menucnt, buf); //работало!!! 10:05 10102023
  u8g2->sendBuffer();*/




  /*for(int i=1;i<=menuMaxShow;i++)
  {
    next = TopMenu.next(i);
    if (next != NULL && next != prev)
    {
      menucnt += 10;
      u8g2->setDrawColor(1);
      char buf[256];
      snprintf(buf, sizeof(buf), "%s", next->name);
      u8g2->setCursor(0, menucnt);
      u8g2->print(buf);
      //u8g2->drawStr(0, menucnt, buf);  //тоже работало
      u8g2->sendBuffer();
    }
    prev = next;
  }
  */
}

void displayValue(SimpleMenu *_menu)
{
  TS=rtc.getTimeStruct();
  int menucnt = 10;
  u8g2->setDrawColor(0);
  u8g2->drawBox(0, 0, 64, 60);
  u8g2->setDrawColor(1);
  u8g2->drawBox(0, 0, 64, 12);
  
  char buf[256];
  snprintf(buf, sizeof(buf), "%s %02d:%02d", _menu->name,(TS.tm_hour-(rtc.offset/3600)),TS.tm_min); 
  
  u8g2->setDrawColor(0);  
  u8g2->setCursor(0, menucnt);
  u8g2->print(buf);
  u8g2->sendBuffer();
  rtc.setTimeStruct(TS);
  //timechanged = true;

}


SimpleMenu* ShowAllNext(SimpleMenu *menu, char *buf)
{
    static SimpleMenu *next=NULL;
    next = menu->next();
    


    if (next != NULL)
    {
        snprintf(buf, sizeof(buf), "%s", next->name);
        displayMsg(buf);
    }
    return next;
}


void setSprd(int *param)
{
  int index = * (int *) param;
  //MyHeater.Status = uint8_t(index);
  if(index==5&&debug) Serial.println("OK !!!!!!!!!!!!");
  //return true;
}


void ToggleDebug()
{
    

    if(debug)
    {
      debug=0;
    }
    else
    {
      debug=1;
    }
    
    EEPROM.write(0,debug);
    EEPROM.commit();
}









void setBw()
{
    digitalWrite(2,1);
    usleep(200);
    digitalWrite(2,0);
    
}

void fMessage()
{

    sendHEX(0x08);

}

void fStart()
{

  if(commandQueueD<=254)
  {
    commandQueueD++;
    commandQueue[commandQueueD]=0xF8;
  }
  //sendHEX(0xF8);
    if(debug)
    {
      //displayMsg(print(&MyHeater.start));
      //Serial.println(print(&MyHeater.start));
    }
}

void fStop()
{
  if(commandQueueD<=254)
  {
    commandQueueD++;
    commandQueue[commandQueueD]=0xFC;
  }
    //sendHEX(0xFC);
    if(debug)
    {
      //displayMsg(print(&MyHeater.shutdown));
      //Serial.println(print(&MyHeater.shutdown));
    } 
}

void fGetStatus()
{
  if(commandQueueD<=254)
  {
    commandQueueD++;
    commandQueue[commandQueueD]=0x08;
  }
  //sendHEX(0x08);
  if(debug)
  {
    //displayMsg(print(&MyHeater.shutdown));
    //Serial.println(print(&MyHeater.shutdown));
  } 
}


void fSettings()
{
    if(debug)
    {
      //displayMsg(print(&MyHeater.settingsP));
      //Serial.println(print(&MyHeater.settingsP));
    }
}



void exitF()
{
    TopMenu.home();
}



void doubleClick()
{
  //Serial.println("DBL");
  tick=0;
  TopMenu.up();
  
}

void Click()
{
  //Serial.println("CL");
  tick=0;
  TopMenu.down();
}

void longClick()
{
  //Serial.println("LNG-CL");
  tick=0;
  TopMenu.select();
}



void IRAM_ATTR isr() 
{
	Serial.println("LORA_IO_DONE!");
  digitalWrite(PIN_LED,1);
  usleep(1000);
  digitalWrite(PIN_LED,0);  
}


void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.print("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}


void setup()
{
  if(rtc.getHour()==3) rtc.setTime(0, 54, 9, 23, 10, 2023); //GMT TIME
  

  EEPROM.begin(256);
  
  ++bootCount;

  pinMode(2, OUTPUT);
  
  pinMode(GPIO_NUM_26, INPUT);

  
  debug = EEPROM.read(0);
  


  button.attachDoubleClick(doubleClick);
  button.attachClick(Click);
  button.attachLongPressStart(longClick);

  init_oled();

  Serial.begin(115200);
  while (!Serial);


  Serial.println("Boot number: " + String(bootCount));
  

  print_wakeup_reason();
  


  int count=0;
  int speed=0;
  
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa_init();
  TopMenu.begin(displayMenu, displayValue);

  
  fGetStatus();
  
// end setup
}



void loop()
{
  onReceive(LoRa.parsePacket());
  
	if(millis() - lastRefreshTime >= REFRESH_INTERVAL)
	{
    
    
    if(Serial.available())
    {
        char temp;
        int speed=2400;
        temp=Serial.read();
        if(temp=='1')speed=1200;
        if(temp=='2')speed=2400;
        if(temp=='3')speed=4800;
        if(temp=='4')speed=9600;
        if(temp=='5')speed=57600;
        if(temp=='d')debug=1;
        if(temp=='+')fStart();
        if(temp=='-')fStop();
        if(temp=='s')fSettings();
        //mySerial.end();
        //mySerial.begin(speed);
        
        //Serial.println(speed);
    }
    lastRefreshTime += REFRESH_INTERVAL;
		

    
    if(LoraMessage.length()>0)
    {
      char tempstr[25];
      sprintf(tempstr,"%s", LoraMessage);
      displayMsg(tempstr);
    }

    /*if(LoraCommand==0)
    {
        //sendPing();
        //sendStatus();
        fGetStatus();
        
    } */   
    if(LoraCommand==1)
    {
        //fStart();
        displayMsg("Старт OK");
        LoraCommand=0;
    }
    
    if(LoraCommand==2)
    {
        //fStop();
        displayMsg("Стоп OK");
        LoraCommand=0;
    }
    
    //if(status)fGetStatus();

    if(commandQueueD>0)
    {
        sendHEX(commandQueue[commandQueueD]);
        commandQueueD--;
    }
    else
    {
      fGetStatus();
    }

    displayDateTime();
    displayBat(LoraBatt);
    char tempbuf[20];
    sprintf(tempbuf,"%d",LoraStatus);
    displayStatus(tempbuf);
    char temps[30];
    sprintf(temps,"%d*",LoraTemp);
    displayTemp(temps);
    
    if(tick > 0 && wakeflag > 0)
    {
        tick=0;
    }

    else if(tick > timetosleep)
    {
      //need to go sleep
      goto_deepsleep();
    }
    tick++;

    
    
	}

  button.tick();


   
  
  
    //onReceive(LoRa.parsePacket());
    
  
}



void sendPing()
{
  for(int i=0;i<=6;i++)
  {
    //mySerialTX.write(char_ping[i]);  
  }

  

}

void sendStatus()
{
  /*if(MyHeater.Battery>0||MyHeater.temp1>0||MyHeater.Mode>0||MyHeater.sync==true)
  {
    char status[4];
    status[0]=0xaa;
    status[1]=MyHeater.Battery;
    status[2]=MyHeater.temp1;
    status[3]=MyHeater.Status;    
    sendMessage(status);
  }
*/
}

