/**
* Arduino automotive battery voltage logger
*
* @author Robert Klajko
* @url https://github.com/klajkor/auto-battery-logger.git
*
* Board: Arduino Pro Mini (or Nano)
*
* Extension modules and sensors used:
*  - INA219, I2C
*  - DS3231 RTC, I2C
*  - SD card, SPI
*  - DHT22 (AM2302) temperature and humidity sensor
* Libraries used:
*  - Adafruit INA219 by Adafruit - Copyright (c) 2012, Adafruit Industries
*  - SD by Adafruit Industries - Copyright (c) 2012, Adafruit Industries
*  - uRTCLib by Naguissa - Copyright (c) 2019, Naguissa
*
* BSD license, all text here must be included in any redistribution.
*
*/

/** 
* Arduino  pinout connections
*
* I2C bus:
* - SCK - pin A5
* - SDA - pin A4
*
* SD card on SPI bus:
* - MOSI - pin D11
* - MISO - pin D12
* - CLK - pin D13
* - CS - pin D10
*
* DHT22 sensor:
* - DATA - pin D4
*/

/**
 * Toolchain: VSCode + Platform.IO 
 */

#include <Arduino.h>
#include <math.h>
#include <uRTCLib.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <avr/sleep.h> //AVR libraryfor sleep modes
#include "DHT.h"

/* Declarations and initializations */

// Current and voltage sensor class
Adafruit_INA219 ina219_monitor;

// DS3231 RTC modul I2C address
const int RTC_I2C_addr = 0x68;
// RTC class
uRTCLib rtc(RTC_I2C_addr);
#define ALARM_INTERVAL_MINUTES 2

// SD card modul chip select
#define SDCARD_CHIP_SELECT 10

//DHT22 Humidity/Temperature Sensor
#define DHTPIN 4      // DHT sensor pin
#define DHTTYPE DHT22 // DHT DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);

#define WakeUpInterruptPin 2 //INT0 is on pin D2 in Arduino

/* Global variables */

/* RTC global variables */
uint8_t rtc_second = 0;
uint8_t rtc_minute = 0;
uint8_t rtc_hour = 0;
uint8_t rtc_day = 0;
uint8_t rtc_month = 0;
uint8_t rtc_year = 0;
float rtc_temperature = 0;
char rtc_TemperatureString[] = "999.9 ";

// Current sensor variables
float f_BusVoltage_V;    /** Measured bus voltage */
float f_ShuntCurrent_mA; /** Measured shunt current */

// General variables
char DateStampString[] = "2000.99.88"; /** String to store date value */
char TimeStampString[] = "00:00:00";   /** String to store time value */
char logFileName[] = "mmddHHMM.txt";   /** String to store log file name */
char VoltString[] = "99.999 ";         /** String to store measured voltage value */
char CurrentString[] = "9999.999 ";    /** String to store measured current value */
bool SD_log_enabled = false;           /** Enabling SD logging or not */

// Datafile class
File dataFile;

//DHT22 measured values
float temperature = 0; /** in Celsius degree */
float humidity = 0;
char TemperatureString[] = "999.9 ";
char HumidityString[] = "999.99 ";

/* Function definitions */

bool Log_To_SD_card(void);
void setTimeStampString(void);
void ina219_Init(void);
void SD_Card_Init(void);
void GPIO_Init(void);
void dht22_Init(void);
void get_Temp_Humid(void);
void get_Voltage_Current(void);
void rtc_Init(void);
void set_Next_Alarm(uint8_t interval_minute);
void wakeUp(void);

void setup()
{
  Serial.begin(115200);
  GPIO_Init();
  Wire.begin();
  delay(100);
  ina219_Init();
  delay(100);
  rtc_Init();
  SD_Card_Init();
  dht22_Init();
  setTimeStampString();
  get_Voltage_Current();
  get_Temp_Humid();
  Log_To_SD_card();
  
  set_Next_Alarm(ALARM_INTERVAL_MINUTES);
}

void loop()
{
  sleep_enable();
  attachInterrupt(0, wakeUp, LOW); //0 = pin D2
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); //full sleep
  Serial.println(F("sleep starts"));
  delay(200);
  sleep_cpu();
  Serial.println(F("back from sleep")); 
  
  // log all data to file
  /**
  if (digitalRead(WakeUpInterruptPin) == LOW)
  {
    */
    digitalWrite(LED_BUILTIN, LOW);     //turning LED on
    setTimeStampString();
    get_Voltage_Current();
    get_Temp_Humid();
    Log_To_SD_card();
    rtc.alarmClearFlag(URTCLIB_ALARM_1);
    set_Next_Alarm(ALARM_INTERVAL_MINUTES);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);     //turning LED off
  //}
  
}

void wakeUp(void){
  sleep_disable();
  detachInterrupt(0); //Removes the interrupt from pin D2;
  Serial.println(F("IF")); 
}

/**
* @brief Query date & time and set the [Date|Time]StampStrings & logFileName accordingly.
* @param void
*/
void setTimeStampString(void)
{
  // get time stamp, convert to a string
  rtc.refresh();
  rtc_second = rtc.second();
  rtc_minute = rtc.minute();
  rtc_hour = rtc.hour();
  rtc_day = rtc.day();
  rtc_month = rtc.month();
  rtc_year = rtc.year();

  DateStampString[2] = (char)((rtc_year / 10) + 0x30);
  DateStampString[3] = (char)((rtc_year % 10) + 0x30);
  DateStampString[5] = (char)((rtc_month / 10) + 0x30);
  DateStampString[6] = (char)((rtc_month % 10) + 0x30);
  DateStampString[8] = (char)((rtc_day / 10) + 0x30);
  DateStampString[9] = (char)((rtc_day % 10) + 0x30);

  TimeStampString[0] = (char)((rtc_hour / 10) + 0x30);
  TimeStampString[1] = (char)((rtc_hour % 10) + 0x30);
  TimeStampString[3] = (char)((rtc_minute / 10) + 0x30);
  TimeStampString[4] = (char)((rtc_minute % 10) + 0x30);
  TimeStampString[6] = (char)((rtc_second / 10) + 0x30);
  TimeStampString[7] = (char)((rtc_second % 10) + 0x30);

  logFileName[0] = DateStampString[0];
  logFileName[1] = DateStampString[1];
  logFileName[2] = DateStampString[2];
  logFileName[3] = DateStampString[3];
  logFileName[4] = DateStampString[5];
  logFileName[5] = DateStampString[6];
  logFileName[6] = DateStampString[8];
  logFileName[7] = DateStampString[9];

  //display time stamp
  Serial.print(DateStampString);
  Serial.print(F(" "));
  Serial.print(TimeStampString);
  Serial.print(F(" "));
  
}

/**
* @brief Log the measurements with timestamp to SD card in CSV format.
* @param void
* @return bool FileOpenSuccess
*/
bool Log_To_SD_card(void)
{
  bool FileOpenSuccess = false;

  if (SD_log_enabled)
  {
    Serial.print(F("SD log: "));
    Serial.print(logFileName);

    dataFile = SD.open(logFileName, FILE_WRITE);
    //dataFile = SD.open(_logfile, FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile)
    {
      FileOpenSuccess = true;
    }
    else
    {
      FileOpenSuccess = false;
      Serial.println(F(" failed!"));
    }

    if (FileOpenSuccess)
    {
      dataFile.print(DateStampString);
      dataFile.print(F(","));
      dataFile.print(TimeStampString);
      dataFile.print(F(","));
      dataFile.print(VoltString);
      dataFile.print(F(",V,"));
      dataFile.print(CurrentString);
      dataFile.print(F(",mA,"));
      dataFile.print(TemperatureString);
      dataFile.print(F(",C,"));
      dataFile.print(HumidityString);
      dataFile.print(F(",%,"));
      dataFile.print(rtc_TemperatureString);
      dataFile.println(F(",C"));
      dataFile.close();
      Serial.println(F(" OK"));
    }
  }
  return FileOpenSuccess;
}

void ina219_Init(void)
{
  ina219_monitor.begin();
  //Serial.println(F("INA219 begin done"));
  // begin calls:
  // configure() with default values RANGE_32V, GAIN_8_320MV, ADC_12BIT, ADC_12BIT, CONT_SH_BUS
  // calibrate() with default values D_SHUNT=0.1, D_V_BUS_MAX=32, D_V_SHUNT_MAX=0.2, D_I_MAX_EXPECTED=2
  // in order to work directly with ADAFruit's INA219B breakout
}

void SD_Card_Init(void)
{
  Serial.print(F("SD card "));

  // see if the card is present and can be initialized:
  if (!SD.begin(SDCARD_CHIP_SELECT))
  {
    Serial.println(F(" failed"));
    SD_log_enabled = false;
  }
  else
  {
    SD_log_enabled = true;
    Serial.println(F(" OK"));
  }
  delay(200);
}

void GPIO_Init(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(WakeUpInterruptPin, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN, LOW); //turning LED on
}

void dht22_Init(void)
{
  dht.begin(); //Starts the DHT sensor
}

void get_Temp_Humid(void)
{
  temperature = dht.readTemperature(); // Read temperature as Celsius (default setting)
  humidity = dht.readHumidity();
  rtc_temperature = ((float) rtc.temp() / 100);
  dtostrf((temperature), 5, 1, TemperatureString);
  dtostrf((rtc_temperature), 5, 1, rtc_TemperatureString);
  dtostrf(humidity, 6, 2, HumidityString);
  Serial.print(F("Temp:"));
  Serial.print(TemperatureString);
  Serial.print(F(" *C "));
  Serial.print(F("Humid:"));
  Serial.print(HumidityString);
  Serial.print(F(" % "));
  Serial.print(F("RTC Temp:"));
  Serial.print(rtc_TemperatureString);
  Serial.println(F(" *C "));
  
}

void get_Voltage_Current(void)
{
  //measure voltage and current
  f_ShuntCurrent_mA = ina219_monitor.getCurrent_mA();
  f_BusVoltage_V = ina219_monitor.getBusVoltage_V();
  //convert to text
  dtostrf((f_ShuntCurrent_mA), 7, 2, CurrentString);
  dtostrf(f_BusVoltage_V, 6, 3, VoltString);
  //display volt
  Serial.print(VoltString);
  Serial.print(F(" V"));
  //display current
  Serial.print(CurrentString);
  Serial.println(F(" mA "));
}

void rtc_Init(void)
{
  rtc.alarmClearFlag(URTCLIB_ALARM_1);
  rtc.alarmClearFlag(URTCLIB_ALARM_2);
}

void set_Next_Alarm(uint8_t interval_minute)
{
  uint8_t next_alarm_minute;
  if (interval_minute > 60)
  {
    interval_minute = 60;
  }
  next_alarm_minute = (rtc.minute() + interval_minute) % 60;

  rtc.alarmSet(URTCLIB_ALARM_TYPE_1_FIXED_MS, 0, next_alarm_minute, 0, 0);
 }

