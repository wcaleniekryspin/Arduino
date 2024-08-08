#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

#define DEBUG 0
#define RXPin 6
#define TXPin 7
#define EEPROM_INIT_FLAG_ADDR 0
#define EEPROM_DATA_ADDR 1
#define EEPROM_INIT_FLAG 0x45
#define F_CPU 12000000

#define cycles_per_micros ((F_CPU) / (1000000UL))
#define cycles_per_milis ((F_CPU) / (1000UL))
#define delay_cyc_micros(x) __builtin_avr_delay_cycles((x)*cycles_per_micros)
#define delay_cyc_milis(x) __builtin_avr_delay_cycles((x)*cycles_per_milis)

#if (DEBUG == 1)
  #define debug(x) Serial.print(x)
  #define debugln(x) Serial.println(x)
#else
  #include <SPI.h>
  #include <SD.h> 
  #define debug(x) file.print(x)
  #define debugln(x) file.println(x)
#endif

TinyGPSPlus *gps = new TinyGPSPlus();
SoftwareSerial *SerialGPS = new SoftwareSerial(RXPin, TXPin);

void setup() {
  delay_cyc_milis(500);

  if (EEPROM.read(EEPROM_INIT_FLAG_ADDR) != EEPROM_INIT_FLAG)  // Reset EEPROM
  {
    for (int i = 0; i < EEPROM.length(); i++)
      EEPROM.write(i, 0xFF);
    
    EEPROM.write(EEPROM_INIT_FLAG_ADDR, EEPROM_INIT_FLAG);
    EEPROM.write(EEPROM_DATA_ADDR, 0);  // Ustaw domyślną wartość
  }

  EEPROM.write(EEPROM_DATA_ADDR, int(EEPROM.read(EEPROM_DATA_ADDR)) + 1);  // Zapisywanie wartości na pozycji 0 w EEPROM

  #if (DEBUG == 1)
    Serial.begin(9600);
  #else
    while (!SD.begin(SS)) delay_cyc_micros(1);
  #endif

  SerialGPS->begin(9600);

  while (SerialGPS->available() > 0) delay_cyc_micros(1);
}

void loop()
{
  #if (DEBUG == 0)
    File file = SD.open("plik" + String(int(EEPROM.read(EEPROM_DATA_ADDR))) + ".txt", FILE_WRITE);
    while (!file) delay_cyc_micros(1);
  #endif

  uint_fast8_t hour, minute, second;
  double lat, lon, alti, vel;
  String gpsData = "";

  while (SerialGPS->available() > 0)
  {
      if (gps->encode(SerialGPS->read()))
      {
        obtain_data(&lat, &lon, &alti, &hour, &minute, &second, &vel);
        gpsData = String(lat, 5) + "," + String(lon, 5) + "," + String(alti, 2) +  "," + String(vel) +  "," + 
                  ((hour < 10) ? "0" : "") + String(hour) + ":" + ((minute < 10) ? "0" : "") + String(minute) + ":" + ((second < 10) ? "0" : "") + String(second) + "\n";
      }
  }

  debug(gpsData);

  #if (DEBUG == 0)
    file.close();
  #endif
}

void obtain_data(double *Latitude, double *Longitude, double *Altitude, uint_fast8_t *hour, uint_fast8_t *minute, uint_fast8_t *second, double *velocity)
{
  if (gps->location.isValid())
  {
    *Latitude = gps->location.lat();
    *Longitude = gps->location.lng();
    *Altitude = gps->altitude.meters();
  }
  if (gps->time.isValid())
  {
    *hour = gps->time.hour();
    *minute = gps->time.minute();
    *second = gps->time.second();
  }
  if (gps->speed.isValid())
  {
    *velocity = gps->speed.kmph();
  }
  delay(500);
}
