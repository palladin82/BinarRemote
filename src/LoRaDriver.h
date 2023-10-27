#include <lora.h>
#include "sx127xRegs.h"
#include "sx1276Regs-LoRa.h"
#include "sx1276Regs-Fsk.h"


#define RADIO_SCLK_PIN              5
//#define RADIO_MISO_PIN              19
//#define RADIO_MOSI_PIN              27
#define RADIO_CS_PIN                18
#define RADIO_DIO0_PIN              26
#define RADIO_RST_PIN               14
#define REG_PA_CONFIG            0x09
#define REG_PA_DAC               0x4d
#define REG_LNA                  0x0c
#define Channel                  9093E5


extern double freqOffset;
extern double maxfreqOffset;
long corrChannel;

String outgoing;              // outgoing message
String LoraMessage;

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends

int LoraCommand=0;
int LoraTemp=0;
int LoraStatus=0;
int LoraBatt=0;
extern float LoRaSNR;

int currBW = SX127x_BW_125_00_KHZ;
int32_t FreqCorrection;
#define FREQ_STEP 61.03515625
#define FreqCorrectionMax ((int32_t)(100000/FREQ_STEP))
#define FreqCorrectionMin (-FreqCorrectionMax)


static void RxChainCalibration( void );
void SX1276SetModem();
uint8_t SX1276Read( uint16_t addr );
void writeRegisterBits(uint8_t reg, uint8_t value, uint8_t mask);
void  SetPPMoffsetReg(double offset);



void setSprd()
{
  int index;




}


void LoRa_init()
{
  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);// set CS, reset, IRQ pin
  
  corrChannel=Channel;
  corrChannel += freqOffset*1E4;

  LoRa.prebegin(corrChannel);
  RxChainCalibration();
  LoRa.begin(corrChannel);

  LoRa.writeRegister(SX127X_REG_OP_MODE, SX127x_OPMODE_SLEEP);
  LoRa.writeRegister(SX127X_REG_OP_MODE, SX127x_OPMODE_SLEEP);
  LoRa.writeRegister(SX127X_REG_OP_MODE, SX127x_OPMODE_SLEEP);
  LoRa.writeRegister(SX127X_REG_OP_MODE, SX127x_OPMODE_LORA); //must be written in sleep mode
  
  LoRa.setFrequency(corrChannel);
  LoRa.setOCP(240);
  LoRa.setTxPower(20);
  //LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.writeRegister(REG_PA_CONFIG, 0xff);
  LoRa.setPreambleLength(16);
  LoRa.setCodingRate4(7);
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.writeRegister(SX1278_REG_MODEM_CONFIG_3,RF_RXCONFIG_AGCAUTO_ON);
  //LoRa.enableCrc();
  /*LoRa.writeRegister(SX1278_REG_MODEM_CONFIG_3,RF_RXCONFIG_AGCAUTO_ON);
  LoRa.writeRegister(SX127X_REG_DETECT_OPTIMIZE, 0x63);
  writeRegisterBits(SX127X_REG_DETECT_OPTIMIZE, SX127X_DETECT_OPTIMIZE_SF_7_12, SX127X_DETECT_OPTIMIZE_SF_MASK );
  LoRa.writeRegister(SX127X_REG_DETECTION_THRESHOLD, SX127X_DETECTION_THRESHOLD_SF_7_12 );
  LoRa.writeRegister(SX127X_REG_LNA, SX127X_LNA_BOOST_ON|SX127X_LNA_GAIN_1);
  */
 LoRa.idle();
}



void sendMessage(String outgoing) 
{
  digitalWrite(2,1);
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                    // finish packet and send it
  msgCount++;
  //delay(200);                           // increment message ID
  digitalWrite(2,0);
}

void sendHEX(char outgoing) 
{
  digitalWrite(2,1);
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(1);                        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
  //delay(200); 
  digitalWrite(2,0);
}




void onReceive(int packetSize) 
{
  if (packetSize == 0) return;          // if there's no packet, return

  
  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  //  digitalWrite(LED_PIN, !btnState);

  while (LoRa.available())
  {

    incoming += (char)LoRa.read();

  }

  if (incomingLength != incoming.length()) 
  {   // check length for error
    //Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    //Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:


  
  //in car mode

  if (incoming[0] == 0xF8)
  {
     LoraCommand=1;
  }
  if (incoming[0] == 0xFC)
  {
     LoraCommand=2;
  }
  if (incoming[0] == 0xAA)
  {
     LoraBatt = incoming[1];
     LoraTemp = incoming[2];
     LoraStatus = incoming[3];
  }
  if (incoming[0] == 0x08)
  {
     LoraCommand=3;
  }



  if(incoming[0] == 0x20)
  {
     LoraMessage = incoming;
     displayMsgS0(incoming);
  }


  Serial.println(incoming);
  char rssi[16];
  char snr[16];
  snprintf(rssi,sizeof(rssi),"RSSI=%i Dbm",LoRa.packetRssi());
  snprintf(snr,sizeof(snr), " SNR=%0.1f Dbm",LoRa.packetSnr());
  LoRaSNR=LoRa.packetSnr();
  displayMsgS1(rssi);
  displayMsgS2(snr);

  //freq correction!!!
  double ferr=0;
  int32_t freqError = ((LoRa.readRegister(0x28) & 0B111)<<16) + (LoRa.readRegister(0x29)<<8) + LoRa.readRegister(0x2a);
  
  int32_t bit=(freqError & (1<<19));

  if (LoRa.readRegister(0x28) & B1000) 
  { // Sign bit is on
    freqError -= 524288; // B1000'0000'0000'0000'0000
  }

  double tf=((double)freqError*16777216/32E6)* (double)125 / 500.0f * 0.0001;

  freqOffset=tf;
  
  SetPPMoffsetReg(tf);
    
}

uint8_t SX1276Read( uint16_t addr )
{
	return LoRa.readRegister(addr);//read(address);
}

void SX1276Write( uint16_t addr, uint8_t data )
{
	LoRa.writeRegister(addr,data);//write(address, value);
}


void SX1276SetChannel( uint32_t freq )
{
    //SX1276.Settings.Channel = freq;
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1276Write( REG_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
    SX1276Write( REG_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
    SX1276Write( REG_FRFLSB, ( uint8_t )( freq & 0xFF ) );
}

static void RxChainCalibration( void )
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = SX1276Read( REG_PACONFIG );
    initialFreq = ( double )( ( ( uint32_t )SX1276Read( REG_FRFMSB ) << 16 ) |
                              ( ( uint32_t )SX1276Read( REG_FRFMID ) << 8 ) |
                              ( ( uint32_t )SX1276Read( REG_FRFLSB ) ) ) * ( double )FREQ_STEP;

    // Cut the PA just in case, RFO output, power = -1 dBm
    SX1276Write( REG_PACONFIG, 0x00 );

    // Launch Rx chain calibration for LF band
    SX1276Write( REG_IMAGECAL, ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Sets a Frequency in HF band
    SX1276SetChannel( 8683E5 ); // FROM DATASHEET

    // Launch Rx chain calibration for HF band
    SX1276Write( REG_IMAGECAL, ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Restore context
    SX1276Write( REG_PACONFIG, regPaConfigInitVal );
    SX1276SetChannel( initialFreq );
}



void SX1276SetModem()
{
        LoRa.sleep();
        SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );

        SX1276Write( REG_DIOMAPPING1, 0x00 );
        SX1276Write( REG_DIOMAPPING2, 0x00 );
}


void writeRegisterBits(uint8_t reg, uint8_t value, uint8_t mask)
{
    
        uint8_t currentValue = LoRa.readRegister(reg);
        uint8_t newValue = (currentValue & ~mask) | (value & mask);
        LoRa.writeRegister(reg, newValue);
    
}










void  SetPPMoffsetReg(double offset)
{

  
  char msg[255]; 
  
  //freqOffset = offset;
  if(abs(maxfreqOffset)<abs(offset))
  {
    maxfreqOffset=offset;    
  }
  else
  {
    maxfreqOffset+=offset;
    
  }
  corrChannel = Channel - maxfreqOffset*1E4;
  LoRa.setFrequency(corrChannel);

  sprintf(msg,"%uk %.2fk", corrChannel/10000, freqOffset);
  displayMsgS(msg);
  Serial.print("new offset=");
  Serial.println(freqOffset*10000);
 
}
