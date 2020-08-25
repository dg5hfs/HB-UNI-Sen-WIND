//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2018-09-13 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------

// Ventus W132 Homematic Windsensor
// Test mit nur einem Arduino
// Conversion of a W132 wind sensor from ELV based on https://gist.github.com/micw/098709efc83a9d9ebf16d14cea4ca38e
// * 
// * - Disconnect the 433 MHz transmitter, which is connected to the logic board with 3 lines (black, red, blue)
// *   433MHz Sender kann auch angeschlossen bleiben falls man die Original Wetterstation hat oder das 433MHz
//     Signal z.B. mit einem RTL Stick empfangen möchte.
// * - Connect the Arduino Pro Mini 
// * - Black -> GND
// * - Red -> 3.3V
// * - Blue -> PIN3
// * 
// * A description of the protocol can be found at http://www.tfd.hu/tfdhu/files/wsprotocol/auriol_protocol_v20.pdf, https://sourceforge.net/p/wmrx00/discussion/855158/thread/ is also helpful b3a47730 /
// * - Bits and sync signals are encoded using the spacing of the falling edge
// * - this implementation uses an interrupt on the falling edge


// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>
#include <Wire.h>;

#include <MultiChannelDevice.h>


#define PIN_ANEMOMETER 3

const byte addrSlaveI2C = 2;
volatile unsigned long lastTrigger;
const int i2cDataLength = 6;
volatile int W132Data[i2cDataLength];

uint8_t windGust; //For store maximum wind gust between I2C reading
uint8_t windDirectionRangeOld; //For store maximum wind direction range between I2C reading
uint8_t windDirectionOld;  
volatile bool w132run = false;

byte bitPos=-1;
byte messageNum=-1;
// there are always sent 6 messages. For temperature/humidity, the message is repeatet 6 times. For wind, 2 messages are repeatet 3 times each
int message1Bits[36];
int message2Bits[36];

// we use a Pro Mini
// Arduino pin for the LED
// D4 == PIN 4 on Pro Mini
#define LED_PIN 4
// Arduino pin for the config button
// B0 == PIN 8 on Pro Mini
#define CONFIG_BUTTON_PIN 8

// number of available peers per channel
#define PEERS_PER_CHANNEL 6

//seconds between sending messages
#define MSG_INTERVAL 30 //120

// all library classes are placed in the namespace 'as'
using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0x00, 0x3f, 0x01},     // Device ID
  "BCWND00003",           // Device Serial
  {0xF1, 0xD2},           // Device Model Indoor {0xF1, 0xD2},  
 // {0x00,0x40},
  0x10,                   // Firmware Version
  as::DeviceType::THSensor, // Device Type
  {0x01, 0x01}            // Info Bytes
};

/**
   Configure the used hardware
*/
typedef AvrSPI<10, 11, 12, 13> SPIType;
typedef Radio<SPIType, 2> RadioType;
typedef StatusLed<LED_PIN> LedType;
typedef AskSin<LedType, BatterySensor, RadioType> Hal;
Hal hal;

class WeatherEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, int16_t temperature, uint8_t humidity, uint16_t windspeed, uint16_t winddir, uint8_t winddirrange, uint16_t gustspeed, bool batlow) {
      uint8_t t1 = (temperature >> 8) & 0x7f;
      uint8_t t2 = temperature & 0xff;
      if ( batlow == true ) {
        t1 |= 0x80; // set bat low bit
      }
      Message::init(0x14, msgcnt, 0x70, BIDI | WKMEUP, t1, t2);
      pload[0] = humidity & 0xff;
			pload[1] = (windspeed >> 8) & 0xff;
      pload[2] = windspeed & 0xff;
      pload[3] = (winddir >> 8) & 0xff;
		  pload[4] = winddir & 0xff;
		  pload[5] = winddirrange & 0xff;
		  pload[6] = (gustspeed >> 8) & 0xff;
      pload[7] = gustspeed & 0xff;
    }
};

class WeatherChannel : public Channel<Hal, List1, EmptyList, List4, PEERS_PER_CHANNEL, List0>, public Alarm {

    WeatherEventMsg msg;
    uint16_t temperature;
    uint8_t humidity;
    uint16_t windspeed;
    uint16_t winddir;
    uint8_t winddirrange;
		uint16_t gustspeed;
    
    uint16_t        millis;

  public:
    WeatherChannel () : Channel(), Alarm(5), millis(0) {}
    virtual ~WeatherChannel () {}


    // here we do the measurement
    void measure () {
      DPRINT("Measure...\n");
			
		//	Wire.requestFrom(2,10);
		//	int a = Wire.available();
		//	Serial.print("Available: ");
		//	Serial.println(a);
    //  byte i2cData[10];
		//	int i;
	//		while (Wire.available()) {
		//		for (int i = 0; i < 10; i++) {
		//			i2cData[i] = Wire.read();
		//		}
			//}
			temperature = W132Data[4];
 			humidity = W132Data[5];
			windspeed = W132Data[0];
			winddir = W132Data[2];
			winddirrange = W132Data[3];
			gustspeed = W132Data[1];

      DPRINT(F("WINDSPEED     : ")); DDECLN(windspeed);
      DPRINT(F("GUSTSPEED     : ")); DDECLN(gustspeed);
      DPRINT(F("WINDDIR       : ")); DDECLN(winddir);
      DPRINT(F("WINDDIRRANGE  : ")); DDECLN(winddirrange);
      DPRINT(F("TEMPERATURE   : ")); DDECLN(temperature);
      DPRINT(F("HUMIDITY      : ")); DDECLN(humidity);
      DPRINT(F("gust org      : ")); DDECLN(W132Data[1]);
      windGust = 0;
		}

    virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
      uint8_t msgcnt = device().nextcount();
      // reactivate for next measure
      tick = delay();
      clock.add(*this);
      measure();

      msg.init(msgcnt, temperature, humidity, windspeed, winddir, winddirrange, gustspeed, device().battery().low());
      if (msgcnt % 20 == 1) device().sendPeerEvent(msg, *this); else device().broadcastEvent(msg, *this);
    }

    uint32_t delay () {
      return seconds2ticks(MSG_INTERVAL);
    }
    void setup(Device<Hal, List0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
      sysclock.add(*this);
    }

    uint8_t status () const {
      return 0;
    }

    uint8_t flags () const {
      return 0;
    }
};

typedef MultiChannelDevice<Hal, WeatherChannel, 1> WeatherType;
WeatherType sdev(devinfo, 0x20);
ConfigButton<WeatherType> cfgBtn(sdev);

bool verifyChecksum(int bits[]) {
  int checksum=0xf;
  for (int i=0;i<8;i++) {
    checksum-=bits[i*4]|bits[i*4+1]<<1|bits[i*4+2]<<2|bits[i*4+3]<<3;
  }
  checksum&=0xf;
  int expectedChecksum=bits[32]|bits[33]<<1|bits[34]<<2|bits[35]<<3;
  return checksum==expectedChecksum;
}

void decodeMessages() {
  if (!verifyChecksum(message1Bits)) {
    Serial.println("Checksum mismatch in message #1");
    return;
  }
  w132run = false;
  Serial.println("---");
  for (int i=0;i<36;i++) {
    Serial.print(message1Bits[i]);
    Serial.print(" ");
  }
  Serial.println();
  for (int i=0;i<36;i++) {
    Serial.print(message2Bits[i]);
    Serial.print(" ");
  }
  Serial.println();
 
  if (message1Bits[9]==1 && message1Bits[10]==1) { // wind data (2 messages)

    if (!verifyChecksum(message2Bits)) {
      Serial.println("Checksum mismatch in message #1");
      return;
    }
    
    uint8_t windSpeed=(message1Bits[24]    | message1Bits[25]<<1 | message1Bits[26]<<2 | message1Bits[27]<<3 |
                     message1Bits[28]<<4 | message1Bits[29]<<5 | message1Bits[30]<<6 | message1Bits[31]<<7);             
    W132Data[0] = (windSpeed*72);
    Serial.print("Average wind speed: ");
    Serial.print(windSpeed*0.72);
    Serial.println(" km/h");
    Serial.print(windSpeed);
    

    uint8_t windGustRaw=(message2Bits[24]    | message2Bits[25]<<1 | message2Bits[26]<<2 | message2Bits[27]<<3 |
                    message2Bits[28]<<4 | message2Bits[29]<<5 | message2Bits[30]<<6 | message2Bits[31]<<7);
    if (windGustRaw > windGust) windGust = windGustRaw;
    W132Data[1] = (windGust*72);
    Serial.print("Max wind speed: ");
    Serial.print(windGust*0.72f);
    Serial.println(" km/h");
    

    int windDirection=(message2Bits[15]    | message2Bits[16]<<1 | message2Bits[17]<<2 | message2Bits[18]<<3 |
                       message2Bits[19]<<4 | message2Bits[20]<<5 | message2Bits[21]<<6 | message2Bits[22]<<7 |
                       message2Bits[23]<<8);
    W132Data[2] = (windDirection);
    Serial.print("Wind direction: ");
    Serial.print(windDirection);
    Serial.println(" deg");
  

    uint8_t windDirectionRange;
    windDirectionRange = abs(windDirectionOld - windDirection) % 360;
    Serial.println(windDirectionOld);
    Serial.println(windDirection);
    Serial.println(windDirectionRange);
    if (windDirectionRange > 180) {windDirectionRange = 360 - windDirectionRange;}
    if (windDirectionRange < windDirectionRangeOld) {windDirectionRange = windDirectionRangeOld;}
    windDirectionOld = windDirection;
    windDirectionRangeOld = windDirectionRange;
    W132Data[3] = windDirectionRange;
    Serial.print("Wind direction range: ");
    Serial.print(windDirectionRange);
    Serial.println(" deg");
    
  } else {  // temperature/humidity in both messages
    uint16_t temperatureRaw=(message1Bits[12]    | message1Bits[13]<<1 | message1Bits[14]<<2 | message1Bits[15]<<3 |
                        message1Bits[16]<<4 | message1Bits[17]<<5 | message1Bits[18]<<6 | message1Bits[19]<<7 | 
                        message1Bits[20]<<8 | message1Bits[21]<<9 | message1Bits[22]<<10| message1Bits[23]<<11);
    if (temperatureRaw& 0x800) temperatureRaw+=0xF000; // negative number, 12 to 16 bit
    W132Data[4] = (temperatureRaw);
    float temperature=temperatureRaw*0.1f;
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
    Serial.print("Temp Raw");
    Serial.println(temperatureRaw);
    
    
    int humidity=(message1Bits[24] | message1Bits[25]<<1 | message1Bits[26]<<2 | message1Bits[27]<<3 )+
                 (message1Bits[28] | message1Bits[29]<<1 | message1Bits[30]<<2 | message1Bits[31]<<3 )*10;
    W132Data[5] = humidity;
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
  
   
  }
}


void dataTrigger() {
  w132run = true;
  unsigned long now=micros();
  unsigned long duration=now-lastTrigger;
  lastTrigger=now;

  if (duration>30000) { // a news block of messages begins
    messageNum=0;
  }

  if (duration>7000) { // ~9 ms = sync signal
    if (bitPos==36) { // we got a full message
      if (messageNum==0) { // 1st message completed
        messageNum=1;
      } else if (messageNum==1) { // 2nd message completed
        decodeMessages();
        messageNum=-1;
      }
    }
    bitPos=0; // Message started
    return;
  }

  if (messageNum<0) return; // ignore repeated messages
  if (bitPos<0) return; // invalid message, ignored

  if (messageNum==0) {
    message1Bits[bitPos]=(duration>3300); // 2.2ms=LOW, 4.4ms = HIGH bits
  } else {
    message2Bits[bitPos]=(duration>3300); // 2.2ms=LOW, 4.4ms = HIGH bits
  }
  bitPos++;
  if (bitPos>36) bitPos=-1; // message too long -> invalid
}

void setup () {
//	Wire.begin();
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);
  hal.initBattery(60UL * 60, 22, 19);
  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  sdev.initDone();
  pinMode(PIN_ANEMOMETER,INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ANEMOMETER), dataTrigger, FALLING);
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false && w132run == false) {
     hal.activity.savePower<Sleep<>>(hal);
  }
}

