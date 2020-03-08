//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2018-09-13 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>
#include <Wire.h>;

#include <MultiChannelDevice.h>

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
#define MSG_INTERVAL 120

// all library classes are placed in the namespace 'as'
using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0x00, 0x3f, 0x00},     // Device ID
  "BCWND00002",           // Device Serial
  {0xF1, 0xD2},           // Device Model Indoor
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
			
			Wire.requestFrom(2,10);
			int a = Wire.available();
			Serial.print("Available: ");
			Serial.println(a);
      byte i2cData[10];
			int i;
			while (Wire.available()) {
				for (int i = 0; i < 10; i++) {
					i2cData[i] = Wire.read();
				}
			}
			temperature = (i2cData[7] << 8) | i2cData[8];
 			humidity = i2cData[9];
			windspeed = (i2cData[0] << 8) | i2cData[1];
			winddir = (i2cData[4] << 8) | i2cData[5];
			winddirrange = i2cData[6];
			gustspeed = (i2cData[2] << 8) | i2cData[3];

      DPRINT(F("WINDSPEED     : ")); DDECLN(windspeed);
      DPRINT(F("GUSTSPEED     : ")); DDECLN(gustspeed);
      DPRINT(F("WINDDIR       : ")); DDECLN(winddir);
      DPRINT(F("WINDDIRRANGE  : ")); DDECLN(winddirrange);
      DPRINT(F("TEMPERATURE   : ")); DDECLN(temperature);
      DPRINT(F("HUMIDITY      : ")); DDECLN(humidity);

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

void setup () {
	Wire.begin();
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);
  hal.initBattery(60UL * 60, 22, 19);
  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  sdev.initDone();
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
    hal.activity.savePower<Sleep<>>(hal);
  }
}

