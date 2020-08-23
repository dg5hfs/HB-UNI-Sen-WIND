/*
 * 
 * Conversion of a W132 wind sensor from ELV based on https://gist.github.com/micw/098709efc83a9d9ebf16d14cea4ca38e
 * 
 * - Disconnect the 433 MHz transmitter, which is connected to the logic board with 3 lines (black, red, blue)
 * - Connect the ESP-8266
 * - Black -> GND
 * - Red -> 3.3V
 * - Blue -> D2
 * 
 * A description of the protocol can be found at http://www.tfd.hu/tfdhu/files/wsprotocol/auriol_protocol_v20.pdf, https://sourceforge.net/p/wmrx00/discussion/855158/thread/ is also helpful b3a47730 /
 * - Bits and sync signals are encoded using the spacing of the falling edge
 * - this implementation uses an interrupt on the falling edge
 */

#include <LowPower.h>
#include <Wire.h>

#define PIN_ANEMOMETER 2
const int FIFO_Anzahl = 10;  // Anzahl der FIFO-Speicherplätze
float FIFOU[FIFO_Anzahl];       // Array für die gemittelten U Vectoren
float FIFOV[FIFO_Anzahl];       // Array für die gemittelten V Vectoren
float summeUWerte;              // Summe U Vectoren
float summeVWerte;              // Summe V Vectoren
int durchlauf = FIFO_Anzahl;    // Durchlauf anzahl vom Mittelwert (nur nach Reset intressant damit nicht erst nach 10 durchläufen ein sinnvoller Wert da ist (das Array ist nach dem Reset komplett auf 0))

int windDirection_avg;                     // Windrichtung durchschnitt


const byte addrSlaveI2C = 2;
volatile unsigned long lastTrigger;
const int i2cDataLength = 10;
byte i2cData[i2cDataLength];

uint8_t windGust; //For store maximum wind gust between I2C reading
uint8_t windDirectionRangeOld; //For store maximum wind direction range between I2C reading
uint8_t windDirectionOld;  

byte bitPos=-1;
byte messageNum=-1;
// there are always sent 6 messages. For temperature/humidity, the message is repeatet 6 times. For wind, 2 messages are repeatet 3 times each
int message1Bits[36];
int message2Bits[36];

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
    i2cData[0] = (windSpeed*72 >> 8);
    i2cData[1] = (windSpeed*72 & 0xff);
    Serial.print("Average wind speed: ");
    Serial.print(windSpeed*0.72);
    Serial.println(" km/h");
    Serial.print("Byte H: ");
    Serial.println(i2cData[0], BIN);
    Serial.print("Byte L: ");
    Serial.println(i2cData[1], BIN);
    

    uint8_t windGustRaw=(message2Bits[24]    | message2Bits[25]<<1 | message2Bits[26]<<2 | message2Bits[27]<<3 |
                    message2Bits[28]<<4 | message2Bits[29]<<5 | message2Bits[30]<<6 | message2Bits[31]<<7);
    if (windGustRaw > windGust) windGust = windGustRaw;
    i2cData[2] = (windGust*72 >> 8);
    i2cData[3] = (windGust*72 & 0xff);
    Serial.print("Max wind speed: ");
    Serial.print(windGust*0.72f);
    Serial.println(" km/h");
    Serial.print("Byte H: ");
    Serial.println(i2cData[2], BIN);
    Serial.print("Byte L: ");
    Serial.println(i2cData[3], BIN);

    int windDirection=(message2Bits[15]    | message2Bits[16]<<1 | message2Bits[17]<<2 | message2Bits[18]<<3 |
                       message2Bits[19]<<4 | message2Bits[20]<<5 | message2Bits[21]<<6 | message2Bits[22]<<7 |
                       message2Bits[23]<<8);
    i2cData[4] = (windDirection >> 8);
    i2cData[5] = (windDirection & 0xff);
    Serial.print("Wind direction: ");
    Serial.print(windDirection);
    Serial.println(" deg");
    Serial.print("Byte H: ");
    Serial.println(i2cData[4], BIN);
    Serial.print("Byte L: ");
    Serial.println(i2cData[5], BIN);

    uint8_t windDirectionRange;
    windDirectionRange = abs(windDirectionOld - windDirection) % 360;
    Serial.println(windDirectionOld);
    Serial.println(windDirection);
    Serial.println(windDirectionRange);
    if (windDirectionRange > 180) {windDirectionRange = 360 - windDirectionRange;}
    if (windDirectionRange < windDirectionRangeOld) {windDirectionRange = windDirectionRangeOld;}
    windDirectionOld = windDirection;
    windDirectionRangeOld = windDirectionRange;
    i2cData[6] = windDirectionRange;
    Serial.print("Wind direction range: ");
    Serial.print(windDirectionRange);
    Serial.println(" deg");
    Serial.print("Byte L: ");
    Serial.println(i2cData[6], BIN);

     if (durchlauf > 0) durchlauf-= 1;
  Serial.print("Druchlauf ");
  Serial.println(durchlauf);
 //windrichtung mitteln
    Serial.println("*********************************");
    //float w = 135 * DEG_TO_RAD;
    float U = sin(windDirection * DEG_TO_RAD);        //U Vector berechen (Sinus funktion arbeitet in Radiant)
    float V = cos(windDirection * DEG_TO_RAD);        //V Vector berechen (Sinus funktion arbeitet in Radiant)
    //Serial.println(w);
    Serial.println(U,5);
    Serial.println(V,5);
    
    
    // FIFO-Puffer Inhalte weiterruecken
    for (int i = FIFO_Anzahl-1; i>0; i--) {
    FIFOU[i] = FIFOU[i - 1];
    summeUWerte+=FIFOU[i];
    Serial.print("Summe U: ");
    Serial.println(summeUWerte);
  }
   
    FIFOU[0] = U;                   //U Vector in den FIFO 0
    summeUWerte+=FIFOU[0];
 
    float Uavg = summeUWerte / (FIFO_Anzahl - durchlauf); // gemittelten Analogwert errechnen
    summeUWerte = 0;
    Serial.print("U durchschnitt: ");
    Serial.println(Uavg,5);
    
   
    // FIFO-Puffer Inhalte weiterruecken
    for (int i = FIFO_Anzahl-1; i>0; i--) {
    FIFOV[i] = FIFOV[i - 1];
    summeVWerte+=FIFOV[i];
    Serial.print("Summe V: ");
    Serial.println(summeVWerte);
  }
    FIFOV[0] = V;                   //V Vector in den FIFO 0
    summeVWerte+=FIFOV[0];
 
    float Vavg = summeVWerte / (FIFO_Anzahl - durchlauf); // gemittelten Analogwert errechnen
    summeVWerte = 0;
    Serial.print("V durchschnitt: ");
    Serial.println(Vavg,5);

    float winkel = atan(fabs(Uavg) / fabs(Vavg)) * RAD_TO_DEG;      // aus den U & V Vectoren den Winkel aus rechnen in Grad 
    Serial.print("zurueck gerechneter Winkel: ");
    Serial.println(winkel);
    if (Uavg >= 0 && Vavg >=0) windDirection_avg=round(winkel);                         // Nun aus unserem Berechneten Winkel aus den Vectoren
    if (Uavg >=0 && Vavg < 0) windDirection_avg=round(180-winkel);                      // wieder die Windrichtung in Grad ermitteln 
    if (Uavg < 0 && Vavg >= 0) windDirection_avg=round(360-winkel);               
    if (Uavg < 0 && Vavg < 0) windDirection_avg=round (180+winkel);
    
    Serial.println("*********************************");
    Serial.print("Windrichtung durchschnitt: ");
    Serial.println(windDirection_avg);
    
  } else {  // temperature/humidity in both messages
    uint16_t temperatureRaw=(message1Bits[12]    | message1Bits[13]<<1 | message1Bits[14]<<2 | message1Bits[15]<<3 |
                        message1Bits[16]<<4 | message1Bits[17]<<5 | message1Bits[18]<<6 | message1Bits[19]<<7 | 
                        message1Bits[20]<<8 | message1Bits[21]<<9 | message1Bits[22]<<10| message1Bits[23]<<11);
    if (temperatureRaw& 0x800) temperatureRaw+=0xF000; // negative number, 12 to 16 bit
    i2cData[7] = (temperatureRaw >> 8);
    i2cData[8] = (temperatureRaw & 0xff);
    float temperature=temperatureRaw*0.1f;
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
    Serial.print("Byte H: ");
    Serial.println(i2cData[7], BIN);
    Serial.print("Byte L: ");
    Serial.println(i2cData[8], BIN);
    
    int humidity=(message1Bits[24] | message1Bits[25]<<1 | message1Bits[26]<<2 | message1Bits[27]<<3 )+
                 (message1Bits[28] | message1Bits[29]<<1 | message1Bits[30]<<2 | message1Bits[31]<<3 )*10;
    i2cData[9] = humidity;
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
    Serial.print("Byte L: ");
    Serial.println(i2cData[9], BIN);
   
  }
}

void dataTrigger() {
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

void wireRequestEvent() {
  Serial.println("I2C Data Request");
  Wire.write(i2cData, i2cDataLength); 
  windGust = 0;
  windDirectionRangeOld = 0;
}

void setup() {
  Serial.begin(57600);
  Wire.begin(addrSlaveI2C);
  pinMode(PIN_ANEMOMETER,INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ANEMOMETER), dataTrigger, FALLING);
  Wire.onRequest(wireRequestEvent);
  Serial.println("Started");
}

void loop() {
  delay(1000);
  Serial.println("Goto sleep...");
  delay(100);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}
