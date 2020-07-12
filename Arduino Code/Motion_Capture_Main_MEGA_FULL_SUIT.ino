#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// include the SD library:
#include <SPI.h>
#include <SD.h>
#include "RTClib.h"
#include <LiquidCrystal_I2C.h>


#define NAMECHEST "chest"
#define NAMEBICEP_R "upper_arm.R"
#define NAMEFOREARM_R "forearm.R"
#define NAMEHAND_R "hand.R"
#define NAMEBICEP_L "upper_arm.L"
#define NAMEFOREARM_L "forearm.L"
#define NAMEHAND_L "hand.L"
#define NAMEHEAD "head"
#define NAMEWAIST "spine"
#define NAMETHIGH_R "thigh.R"
#define NAMETHIGH_L "thigh.L"
#define NAMEKNEE_R "shin.R"
#define NAMEKNEE_L "shin.L"
#define NAMEFOOT_R "foot.R"
#define NAMEFOOT_L "foot.L"
/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/
/**************************************************
 * VERSION
 *************************************************/
  float Version = 1.01;
  String VersionString = String(Version,2);
/**************************************************
 *************************************************/

  
// all of the types of sensors we will have.
typedef enum 
{
      CHEST,                                       
      BICEP_R,                                    
      FOREARM_R,                                     
      HAND_R,                                      
      BICEP_L,                                      
      FOREARM_L,                                    
      HAND_L,                                    
      HEAD,                                       
      WAIST,                                       
      THIGH_R,                                    
      KNEE_R,                                     
      FOOT_R,                                     
      THIGH_L,                                    
      KNEE_L,                                    
      FOOT_L                                     
} sensor_id;
 
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

/*
syntax for this is first item is 
device id, not sure what this is, 
device addres either 0x29 or 0x28 for BNO055
mux 1 port:the sensor is on for the mux (unused if next value is not -1)
mux 2 port is the port on the second mux daisy chained off of the first mux at a port defined in the header file.

*/
/*Adafruit_BNO055::Adafruit_BNO055(int32_t sensorID, uint8_t address, int32_t mux1port, int32_t mux2port)*/
Adafruit_BNO055 chest = Adafruit_BNO055(-1,BNO055_ADDRESS_B, 6, -1);
bool chestPresent;

/* Head */
Adafruit_BNO055 head = Adafruit_BNO055(-1,BNO055_ADDRESS_A, 4, -1);
bool headPresent;

/* Right ARm*/
Adafruit_BNO055 forearmR = Adafruit_BNO055(-1,BNO055_ADDRESS_A, 0, -1);
bool forearmRPresent;
Adafruit_BNO055 bicepR = Adafruit_BNO055(-1,BNO055_ADDRESS_B, 0, -1);
bool bicepRPresent;
Adafruit_BNO055 handR = Adafruit_BNO055(-1,BNO055_ADDRESS_B, 1, -1);
bool handRPresent;

/* Left ARm*/
Adafruit_BNO055 forearmL = Adafruit_BNO055(-1,BNO055_ADDRESS_A, 2, -1);
bool forearmLPresent;
Adafruit_BNO055 bicepL = Adafruit_BNO055(-1,BNO055_ADDRESS_B, 2, -1);
bool bicepLPresent;
Adafruit_BNO055 handL = Adafruit_BNO055(-1,BNO055_ADDRESS_B, 3, -1);
bool handLPresent;

/* waist */
Adafruit_BNO055 waist = Adafruit_BNO055(-1,BNO055_ADDRESS_B, -1, 0);
bool waistPresent;


/* Right Leg*/
Adafruit_BNO055 thighR = Adafruit_BNO055(-1,BNO055_ADDRESS_A, -1, 1);
bool thighRPresent;
Adafruit_BNO055 kneeR = Adafruit_BNO055(-1,BNO055_ADDRESS_B, -1, 1);
bool kneeRPresent;
Adafruit_BNO055 footR = Adafruit_BNO055(-1,BNO055_ADDRESS_B, -1, 2);
bool footRPresent;


/* Left Leg*/
Adafruit_BNO055 thighL = Adafruit_BNO055(-1,BNO055_ADDRESS_A, -1, 3);
bool thighLPresent;
Adafruit_BNO055 kneeL = Adafruit_BNO055(-1,BNO055_ADDRESS_B, -1, 3);
bool kneeLPresent;
Adafruit_BNO055 footL = Adafruit_BNO055(-1,BNO055_ADDRESS_B, -1, 4);
bool footLPresent;

bool QuatMode;
int TakeNumber = 1;
bool IMUsInitialized = false;
bool IMUsCalibrated = false;

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;
const int chipSelect = 10;

// real time clock.
RTC_PCF8523 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

bool sdCardPresentandIntialized = false;
const int buttonPin = 5;     // the number of the pushbutton pin
//LEDs
const int ledGreenPin =  7;// the number of the LED pin
const int ledRedPin =  6;// the number of the LED pin
unsigned long CaptureDelay = 0;

//LCD Display
#define LCD_DISPLAY_ADDR 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);
bool DisplayFound = false;

/**************************************************************************/
/*
    LED Function- all Good
*/
/**************************************************************************/
bool CheckForLCDConnect(void)
{
  byte error;

    Wire.beginTransmission(LCD_DISPLAY_ADDR);
    error = Wire.endTransmission();     // stop transmitting
    if (error == 0)
    {
      DisplayFound = true;
      lcd.begin();
      Serial.println("LCD Display Found at 0x27...");
    } else
    {
      Serial.println("No LCD Display found...");
    }
  
}
void ClearRowOfDisplay(int RowNum){
  if(RowNum == 1)
  {
    lcd.setCursor(0, 0);
    lcd.print("                ");
  } else
  {
    lcd.setCursor(0, 1);
    lcd.print("                ");
  }
}
void PrintLCDLine1(String c)
{
  if(DisplayFound)
  {
    ClearRowOfDisplay(1);
    lcd.setCursor(0, 0);
    lcd.print(c);
  }
}
void PrintLCDLine2(String c)
{
  if(DisplayFound)
  {
    ClearRowOfDisplay(2);
    lcd.setCursor(0, 1);
    lcd.print(c);
  }
}
/**************************************************************************/
/*
    LED Function- all Good
*/
/**************************************************************************/
void SetupLEDs(){
  pinMode(ledGreenPin, OUTPUT);
  pinMode(ledRedPin, OUTPUT);
}
void StatusAllGood(){
  digitalWrite(ledGreenPin, HIGH);
  digitalWrite(ledRedPin, LOW);
}
void Busy(){
  digitalWrite(ledGreenPin, !digitalRead(ledGreenPin));
  digitalWrite(ledRedPin, LOW);
}
void Recording(){
  digitalWrite(ledRedPin, !digitalRead(ledRedPin));
  digitalWrite(ledGreenPin, LOW);
}
void StatusAllBad(){
  digitalWrite(ledGreenPin, LOW);
  digitalWrite(ledRedPin, HIGH);
}
/**************************************************************************/
/*
    Set Capture Speed
*/
/**************************************************************************/
unsigned long SetSpeed( unsigned long CurrentDelay ){
  unsigned long SpeedDelayOut = 0;
  long int userInput;
  
  Serial.print("Current Delay is Set To: ");
  Serial.println(CurrentDelay, DEC);
  Serial.println("Please Enter the speed you want to capture at:");
  Serial.println("0 As Fast As Possible");  
  Serial.println("1 is Super Fast");        
  Serial.println("2 Fast");             
  Serial.println("3 Medium");              
  Serial.println("4 Slow");              
  Serial.println("5 Super Slow");   
    while (Serial.available()==0)  { }
  userInput = Serial.parseInt();

  switch(userInput){
  case 0:
    SpeedDelayOut = 0;
    break;
  case 1:
    SpeedDelayOut = 10;
    break;
  case 2:
    SpeedDelayOut = 50;
    break;
  case 3:
    SpeedDelayOut = 100;
    break;
  case 4:
    SpeedDelayOut = 200;
    break;
  case 5:
    SpeedDelayOut = 500;
    break;
  default:
    SpeedDelayOut = BNO055_SAMPLERATE_DELAY_MS;

  }   
  Serial.print("Delay is Set To: ");
  Serial.println(SpeedDelayOut, DEC);
  return SpeedDelayOut;
}
  
/**************************************************************************/
/*
    Set the Clock
*/
/**************************************************************************/
void GetTimeFromUserAndSet(){
  long int userYear; 
  long int userMonth = 1; 
  long int userDay = 21; 
  long int userHour = 8; 
  long int userMinute = 30; 
  long int userSecond = 0; 
  DateTime now = rtc.now();

  Serial.print("Please Enter The Year: (i.e. ");
  Serial.print(now.year(), DEC);
  Serial.println(")");        //Prompt User for input
    while (Serial.available()==0)  { }
    userYear = Serial.parseInt();

  Serial.print("Please Enter The Month: (i.e. ");
  Serial.print(now.month(), DEC);
  Serial.println(")");        //Prompt User for input
    while (Serial.available()==0)  { }
    userMonth = Serial.parseInt();
    
  Serial.print("Please Enter The Day: (i.e. ");
  Serial.print(now.day(), DEC);
  Serial.println(")");        //Prompt User for input
    while (Serial.available()==0)  { }
    userDay = Serial.parseInt();
    
  Serial.print("Please Enter The Hour: (i.e. ");
  Serial.print(now.hour(), DEC);
  Serial.println(")");        //Prompt User for input
    while (Serial.available()==0)  { }
    userHour = Serial.parseInt();
    
  Serial.print("Please Enter The Minute: (i.e. ");
  Serial.print(now.minute(), DEC);
  Serial.println(")");        //Prompt User for input
    while (Serial.available()==0)  { }
    userMinute = Serial.parseInt();
    
  Serial.print("Please Enter The Second: (i.e. ");
  Serial.print(now.second(), DEC);
  Serial.println(")  When you press enter the time will be committed.");        //Prompt User for input
    while (Serial.available()==0)  { }
    userSecond = Serial.parseInt();

    
    rtc.adjust(DateTime(userYear,userMonth, userDay, userHour, userMinute, userSecond));
    
  Serial.println("Time Set To:" + GetTime());        //Prompt User for input
}
/**************************************************************************/
/*
    SDCard Info Display
*/
/**************************************************************************/
bool DetectAndDisplaySDCardInfo(){

  bool cardPassedInit = true;
  Serial.print("\nInitializing SD card...");
  Busy();
  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    cardPassedInit = false;
    StatusAllBad();
  } else {
    Serial.println("Wiring is correct and a card is present.");
    Busy();
  }

  if(cardPassedInit){
    // print the type of card
    Serial.print("\nCard type: ");
    switch (card.type()) {
      case SD_CARD_TYPE_SD1:
        Serial.println("SD1");
        break;
      case SD_CARD_TYPE_SD2:
        Serial.println("SD2");
        break;
      case SD_CARD_TYPE_SDHC:
        Serial.println("SDHC");
        break;
      default:
        Serial.println("Unknown");
    Busy();
    }
  
    // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
    if (!volume.init(card)) {
      Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
      cardPassedInit = false;
      return cardPassedInit;
      Busy();
    }
  
  
    // print the type and size of the first FAT-type volume
    uint32_t volumesize;
    Serial.print("\nVolume type is FAT");
    Serial.println(volume.fatType(), DEC);
    Serial.println();
    Busy();
  
    volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
    volumesize *= volume.clusterCount();       // we'll have a lot of clusters
    volumesize *= 512;                            // SD card blocks are always 512 bytes
    Serial.print("Volume size (bytes): ");
    Serial.println(volumesize);
    Serial.print("Volume size (Kbytes): ");
    volumesize /= 1024;
    Serial.println(volumesize);
    Serial.print("Volume size (Mbytes): ");
    volumesize /= 1024;
    Serial.println(volumesize);
    Busy();
  
  
    Serial.println("\nFiles found on the card (name, date and size in bytes): ");
    root.openRoot(volume);
  
    // list all files in the card with date and size
    root.ls(LS_R | LS_DATE | LS_SIZE);
    Busy();
  }
  
  return cardPassedInit;
}
/**************************************************************************/
/*
    Get Time String
*/
/**************************************************************************/
String GetTime(){
  
    DateTime now = rtc.now();
    String TimeString = String(now.year(), DEC);
    TimeString += '/';
    TimeString += String(now.month(), DEC);
    TimeString += '/';
    TimeString += String(now.day(), DEC);
    TimeString += " (";
    TimeString += String(daysOfTheWeek[now.dayOfTheWeek()]);
    TimeString += ") ";
    TimeString += String(now.hour(), DEC);
    TimeString += ':';
    TimeString += String(now.minute(), DEC);
    TimeString += ':';
    TimeString += String(now.second(), DEC);
    return TimeString;
}
/**************************************************************************/
/*
    SDCard Init
*/
/**************************************************************************/
void initializeSDCardForDataLogging(){
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  } else{
  sdCardPresentandIntialized = true;
  Serial.println("card initialized.");
  }
}
/**************************************************************************/
/*
    Get FileName for datalogging
*/
/**************************************************************************/
String DataLoggingName(){
    DateTime now = rtc.now();
    int filenum = 1;
    String FinalString = "";
    String Temp = String(now.year(), DEC);
    String TimeString = Temp.substring(Temp.length() -1);
    TimeString += String(now.month(), DEC);
    TimeString += String(now.day(), DEC);
    TimeString += "_";
    FinalString = TimeString + String(filenum, DEC) + ".TXT";
    while(SD.exists(FinalString)){
      filenum = filenum + 1;
      FinalString = TimeString + String(filenum, DEC) + ".TXT";
    }    
  return FinalString;
}
/**************************************************************************/
/*
    Print Line to File
*/
/**************************************************************************/
void PrintLine2File(String Line, File dataFile){
  if (dataFile) {
    dataFile.println(Line);
  }
}
/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
bool displayCalStatus(sensor_id senseID)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bool sensorExists = true;
  bool calibrated;
  String SensorName;
  Serial.print("~~");   
  switch (senseID)
  {
    case CHEST: 
      if(chestPresent)
      {
        SensorName = "NAMECHEST";
        chest.getCalibration(&system, &gyro, &accel, &mag);
      };
      break;
    case BICEP_R: 
      if(bicepRPresent)
      {
        SensorName = "NAMEBICEP_R";
        bicepR.getCalibration(&system, &gyro, &accel, &mag);
      };
      break;
    case FOREARM_R: 
      if(forearmRPresent)
      {
        SensorName = "NAMEFOREARM_R";
        forearmR.getCalibration(&system, &gyro, &accel, &mag);
      };
      break;  
    case HAND_R: 
      if(handRPresent)
      {
        SensorName = "NAMEHAND_R";
        handR.getCalibration(&system, &gyro, &accel, &mag);
      };
    case HEAD: 
      if(headPresent)
      {
        SensorName = "HEAD";
        head.getCalibration(&system, &gyro, &accel, &mag);
      };
    case WAIST: 
      if(waistPresent)
      {
        SensorName = "WAIST";
        waist.getCalibration(&system, &gyro, &accel, &mag);
      };
    case THIGH_R: 
      if(thighRPresent)
      {
        SensorName = "THIGH_R";
        thighR.getCalibration(&system, &gyro, &accel, &mag);
      };
    case KNEE_R: 
      if(kneeRPresent)
      {
        SensorName = "KNEE_R";
        kneeR.getCalibration(&system, &gyro, &accel, &mag);
      };
    case FOOT_R: 
      if(footRPresent)
      {
        SensorName = "FOOT_R";
        footR.getCalibration(&system, &gyro, &accel, &mag);
      };
    case THIGH_L: 
      if(thighLPresent)
      {
        SensorName = "THIGH_L";
        thighL.getCalibration(&system, &gyro, &accel, &mag);
      };
    case KNEE_L: 
      if(kneeLPresent)
      {
        SensorName = "KNEE_L";
        kneeL.getCalibration(&system, &gyro, &accel, &mag);
      };
    case FOOT_L: 
      if(footLPresent)
      {
        SensorName = "FOOT_L";
        footL.getCalibration(&system, &gyro, &accel, &mag);
      };
      break;  
    default: 
      sensorExists = false;
      break;
  }
  
  if(sensorExists)
  {
    /* The data should be ignored until the system calibration is > 0 */
    //Serial.print("\t");
    if (!system)
    {
      Serial.print("~~");
      Serial.print(SensorName);
      Serial.print("NOCAL");
      calibrated = false;
    }else {
      calibrated = true;
    }
  
    /* Display the individual values */
//    Serial.print(system, DEC);
//    Serial.print("G:");
//    Serial.print(gyro, DEC);
//    Serial.print("A:");
//    Serial.print(accel, DEC);
//    Serial.print("M:");
//    Serial.print(mag, DEC);
    
  } else {
    calibrated = true;;
  }
  return calibrated;
}
/**************************************************************************/
/*
    Enumerate I2C bus
*/
/**************************************************************************/
void enumerateI2CBus(void)
{  byte error, address, muxport;
  int nDevices;
  boolean mux1Found;
  boolean mux2Found;
  boolean RTCFound;
 
  mux1Found = false;
  mux2Found = false;
  RTCFound = false;
  
  Serial.println("Scanning...");

    CheckForLCDConnect();
      PrintLCDLine1("Enumerating I2C..");
      PrintLCDLine2("");
    Wire.beginTransmission(0x70);
    error = Wire.endTransmission();     // stop transmitting
    if (error == 0)
    {
      mux1Found = true;
      Serial.println("Chest I2C Mux found at address 0x70...");
    } else
    {
      Serial.println("No I2C Mux in chest found...");
    }

    if(mux1Found){
      //set mux1 to mux2 port
      Wire.beginTransmission(0x70);
      Wire.write(0x1 << MUX2_MUX1_PORT);
      error = Wire.endTransmission();     // stop transmitting
      Wire.beginTransmission(0x71);
      error = Wire.endTransmission();     // stop transmitting
      if (error == 0)
      {
        mux2Found = true;
        Serial.println("Waist I2C Mux found at address 0x71...");
      } else
      {
        Serial.println("No I2C Mux in waist found...");
      }
    } else
    {
        Serial.println("Didn't even look for second Mux because first mux missing...");
    }
    
    Wire.beginTransmission(0x68);
    error = Wire.endTransmission();     // stop transmitting
    if (error == 0)
    {
      RTCFound = true;
      Serial.println("Real Time Clock found at address 0x68...");
    } else
    {
      Serial.println("No Real Time Clock found in chest...");
    }
    
  nDevices = 0;
  
  if (mux1Found)
    {
    for(muxport = 0; muxport <= 7; muxport++ )
    {
      Serial.print("Checking Mux1 Port ");
      Serial.print(muxport,HEX);
      Serial.println(".");
      Wire.beginTransmission(112); // transmit to device #112 (0x70)
                                // device address is specified in datasheet
      Wire.write(0x1 << muxport);             // sends value byte  
      Wire.endTransmission();     // stop transmitting
  
      for(address = 1; address < 127; address++ )
      {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
     
        if (error == 0 && address != 0x71 && address != 0x70  && address != 0x68 && address != LCD_DISPLAY_ADDR)
        {
          Serial.print("I2C device found at address 0x");
          if (address<16)
            Serial.print("0");
          Serial.print(address,HEX);
          Serial.println("  !");
     
          nDevices++;
        }
        else if (error==4)
        {
          Serial.print("Unknown error at address 0x");
          if (address<16)
            Serial.print("0");
          Serial.println(address,HEX);
        }    
      }
    }
    if(mux2Found)
    {
      //set mux1 to mux2 port
      Wire.beginTransmission(0x70);
      Wire.write(0x1 << MUX2_MUX1_PORT);
      error = Wire.endTransmission();     // stop transmitting
      for(muxport = 0; muxport <= 7; muxport++ )
      {
        Serial.print("Checking Mux2 Port ");
        Serial.print(muxport,HEX);
        Serial.println(".");
        Wire.beginTransmission(0x71); // transmit to device #112 (0x71)
                                  // device address is specified in datasheet
        Wire.write(0x1 << muxport);             // sends value byte  
        Wire.endTransmission();     // stop transmitting
    
        for(address = 1; address < 127; address++ )
        {
          // The i2c_scanner uses the return value of
          // the Write.endTransmisstion to see if
          // a device did acknowledge to the address.
          Wire.beginTransmission(address);
          error = Wire.endTransmission();
       
          if (error == 0 && address != 0x71 && address != 0x70  && address != 0x68 && address != LCD_DISPLAY_ADDR)
          {
            Serial.print("I2C device found at address 0x");
            if (address<16)
              Serial.print("0");
            Serial.print(address,HEX);
            Serial.println("  !");
       
            nDevices++;
          }
          else if (error==4)
          {
            Serial.print("Unknown error at address 0x");
            if (address<16)
              Serial.print("0");
            Serial.println(address,HEX);
          }    
        }
      }
    }
  } else
  {
      for(address = 1; address < 127; address++ )
      {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
     
        if (error == 0)
        {
          Serial.print("I2C device found at address 0x");
          if (address<16)
            Serial.print("0");
          Serial.print(address,HEX);
          Serial.println("  !");
     
          nDevices++;
        }
        else if (error==4)
        {
          Serial.print("Unknown error at address 0x");
          if (address<16)
            Serial.print("0");
          Serial.println(address,HEX);
        }    
      } 
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
      PrintLCDLine1("Done 12C");
      PrintLCDLine2("Enumeration.");
 }
/**************************************************************************/
/*
    Initialize Chest
*/
/**************************************************************************/
void InitChest(void)
{
  /* Initialise the sensor */
  if(!chest.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Chest Not Found.");
    PrintLCDLine1("Chest");
    PrintLCDLine2("NOT FOUND!!");
    chestPresent = false;
    StatusAllBad();
  } else
  {
    Serial.println("Chest Initialized..");
    PrintLCDLine1("Chest");
    PrintLCDLine2("Initialized");
    chestPresent = true;
    chest.setExtCrystalUse(true);
    StatusAllGood();
  }
  /* Display the current temperature */
  //int8_t temp = chest.getTemp();
  //Serial.print("Current Temperature: ");
  //Serial.print(temp);
  //Serial.println(" C");
}

/**************************************************************************/
/*
    Initialize Bicep Left
*/
/**************************************************************************/
void InitbicepL(void)
{
  /* Initialise the sensor */
  if(!bicepL.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Left Bicep Not Found.");
    PrintLCDLine1("Left Bicep");
    PrintLCDLine2("NOT FOUND!!");
    bicepLPresent = false;
    StatusAllBad();
  } else
  {
    Serial.println("Left Bicep Initialized..");
    PrintLCDLine1("Left Bicep");
    PrintLCDLine2("Initialized");
    bicepLPresent = true;
    bicepL.setExtCrystalUse(true);
    StatusAllGood();
  }
}
/**************************************************************************/
/*
    Initialize ForeARm Left
*/
/**************************************************************************/
void InitForeARmL(void)
{
  /* Initialise the sensor */
  if(!forearmL.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Left Fore Arm Not Found.");
    PrintLCDLine1("Left ForeArm");
    PrintLCDLine2("NOT FOUND!!");
    forearmLPresent = false;
    StatusAllBad();
  } else
  {
    Serial.println("Left Fore Arm Initialized..");
    PrintLCDLine1("Left ForeArm");
    PrintLCDLine2("Initialized");
    forearmLPresent = true;
    forearmL.setExtCrystalUse(true);
    StatusAllGood();
  }
}
/**************************************************************************/
/*
    Initialize Hand Left
*/
/**************************************************************************/
void InitHandL(void)
{
  /* Initialise the sensor */
  if(!handL.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Left Hand Not Found.");
    PrintLCDLine1("Left Hand");
    PrintLCDLine2("NOT FOUND!!");
    handLPresent = false;
    StatusAllBad();
  } else
  {
    Serial.println("Left Hand Initialized..");
    PrintLCDLine1("Left Hand");
    PrintLCDLine2("Initialized");
    handLPresent = true;
    forearmL.setExtCrystalUse(true);
    StatusAllGood();
  }
}
/**************************************************************************/
/*
    Initialize Bicep Right
*/
/**************************************************************************/
void InitbicepR(void)
{
  /* Initialise the sensor */
  if(!bicepR.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Right Bicep Not Found.");
    PrintLCDLine1("Right Bicep");
    PrintLCDLine2("NOT FOUND!!");
    bicepRPresent = false;
    StatusAllBad();
  } else
  {
    Serial.println("Right Bicep Initialized..");
    PrintLCDLine1("Right Bicep");
    PrintLCDLine2("Initialized");
    bicepRPresent = true;
    bicepR.setExtCrystalUse(true);
    StatusAllGood();
  }
}
/**************************************************************************/
/*
    Initialize ForeARm Right
*/
/**************************************************************************/
void InitForeARmR(void)
{
  /* Initialise the sensor */
  if(!forearmR.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Right Fore Arm Not Found.");
    PrintLCDLine1("Right Fore Arm");
    PrintLCDLine2("NOT FOUND!!");
    forearmRPresent = false;
    StatusAllBad();
  } else
  {
    Serial.println("Right Fore Arm Initialized..");
    PrintLCDLine1("Right Fore Arm");
    PrintLCDLine2("Initialized");
    forearmRPresent = true;
    forearmR.setExtCrystalUse(true);
    StatusAllGood();
  }
}
/**************************************************************************/
/*
    Initialize Hand Right
*/
/**************************************************************************/
void InitHandR(void)
{
  /* Initialise the sensor */
  if(!handR.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Right Hand Not Found.");
    PrintLCDLine1("Right Hand");
    PrintLCDLine2("NOT FOUND!!");
    handRPresent = false;
    StatusAllBad();
  } else
  {
    Serial.println("Right Hand Initialized..");
    PrintLCDLine1("Right Hand");
    PrintLCDLine2("Initialized");
    handRPresent = true;
    forearmR.setExtCrystalUse(true);
    StatusAllGood();
  }
}
/**************************************************************************/
/*
    Initialize Head
*/
/**************************************************************************/
void Inithead(void)
{
  /* Initialise the sensor */
  if(!head.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("head Not Found.");
    PrintLCDLine1("Head");
    PrintLCDLine2("NOT FOUND!!");
    headPresent = false;
    StatusAllBad();
  } else
  {
    Serial.println("head Initialized..");
    PrintLCDLine1("Head");
    PrintLCDLine2("Initialized");
    headPresent = true;
    head.setExtCrystalUse(true);
    StatusAllGood();
  }
}
/**************************************************************************/
/*
    Initialize Waist
*/
/**************************************************************************/
void Initwaist(void)
{
  /* Initialise the sensor */
  if(!waist.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("waist Not Found.");
    PrintLCDLine1("Waist");
    PrintLCDLine2("NOT FOUND!!");
    waistPresent = false;
    StatusAllBad();
  } else
  {
    Serial.println("waist Initialized..");
    PrintLCDLine1("Waist");
    PrintLCDLine2("Initialized");
    waistPresent = true;
    waist.setExtCrystalUse(true);
    StatusAllGood();
  }
}
/**************************************************************************/
/*
    Initialize Right Thigh
*/
/**************************************************************************/
void InitthighR(void)
{
  /* Initialise the sensor */
  if(!thighR.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Right Thigh Not Found.");
    PrintLCDLine1("Right Thigh");
    PrintLCDLine2("NOT FOUND!!");
    thighRPresent = false;
    StatusAllBad();
  } else
  {
    Serial.println("Right Thigh Initialized..");
    PrintLCDLine1("Right Thigh");
    PrintLCDLine2("Initialized");
    thighRPresent = true;
    thighR.setExtCrystalUse(true);
    StatusAllGood();
  }
}
/**************************************************************************/
/*
    Initialize Right Knee
*/
/**************************************************************************/
void InitkneeR(void)
{
  /* Initialise the sensor */
  if(!kneeR.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Right Knee Not Found.");
    PrintLCDLine1("Righ Knee");
    PrintLCDLine2("NOT FOUND!!");
    kneeRPresent = false;
    StatusAllBad();
  } else
  {
    Serial.println("Right Knee Initialized..");
    PrintLCDLine1("Righ Knee");
    PrintLCDLine2("Initialized");
    kneeRPresent = true;
    kneeR.setExtCrystalUse(true);
    StatusAllGood();
  }
}
/**************************************************************************/
/*
    Initialize Right foot
*/
/**************************************************************************/
void InitfootR(void)
{
  /* Initialise the sensor */
  if(!footR.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Right Foot Not Found.");
    PrintLCDLine1("Right Foot");
    PrintLCDLine2("NOT FOUND!!");
    footRPresent = false;
    StatusAllBad();
  } else
  {
    Serial.println("Right Foot Initialized..");
    PrintLCDLine1("Right Foot");
    PrintLCDLine2("Initialized");
    footRPresent = true;
    footR.setExtCrystalUse(true);
    StatusAllGood();
  }
}
/**************************************************************************/
/*
    Initialize Left Thigh
*/
/**************************************************************************/
void InitthighL(void)
{
  /* Initialise the sensor */
  if(!thighL.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Left Thigh Not Found.");
    PrintLCDLine1("Left Thigh");
    PrintLCDLine2("NOT FOUND!!");
    thighLPresent = false;
    StatusAllBad();
  } else
  {
    Serial.println("Left Thigh Initialized..");
    PrintLCDLine1("Left Thigh");
    PrintLCDLine2("Initialized");
    thighLPresent = true;
    thighL.setExtCrystalUse(true);
    StatusAllGood();
  }
}
/**************************************************************************/
/*
    Initialize Left Knee
*/
/**************************************************************************/
void InitkneeL(void)
{
  /* Initialise the sensor */
  if(!kneeL.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Left Knee Not Found.");
    PrintLCDLine1("Left Knee");
    PrintLCDLine2("NOT FOUND!!");
    kneeLPresent = false;
    StatusAllBad();
  } else
  {
    Serial.println("Left Knee Initialized..");
    PrintLCDLine1("Left Knee");
    PrintLCDLine2("Initialized");
    kneeLPresent = true;
    kneeL.setExtCrystalUse(true);
    StatusAllGood();
  }
}
/**************************************************************************/
/*
    Initialize Right foot
*/
/**************************************************************************/
void InitfootL(void)
{
  /* Initialise the sensor */
  if(!footL.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Left Foot Not Found.");
    PrintLCDLine1("Left Foot");
    PrintLCDLine2("NOT FOUND!!");
    footLPresent = false;
    StatusAllBad();
  } else
  {
    Serial.println("Left Foot Initialized..");
    
    PrintLCDLine1("Left Foot");
    PrintLCDLine2("Initialized");
    footLPresent = true;
    footL.setExtCrystalUse(true);
    StatusAllGood();
  }
}
/**************************************************************************/
/*
    Print Quaternion Data
*/
/**************************************************************************/
String printQuat(imu::Quaternion &quat, String SensorName){
        String ReturnString = "~~";
        ReturnString += SensorName;
        ReturnString += "~qW:";
        ReturnString += String(quat.w(), 4);
        ReturnString += "~qX:";
        ReturnString += String(quat.y(), 4);
        ReturnString += "~qY:";
        ReturnString += String(quat.x(), 4);
        ReturnString += "~qZ:";
        ReturnString += String(quat.z(), 4);
        ReturnString += "~";
        return ReturnString;
}
/**************************************************************************/
/*
    Print Euler Data
*/
/**************************************************************************/
String printEuler(imu::Vector<3> &euler, String SensorName){
        String ReturnString = "~~";
        ReturnString += SensorName;
        // Display the floating point data 
        ReturnString += "~X:";
        ReturnString += String(euler.x());
        ReturnString += "~Y:";
        ReturnString += String(euler.y());
        ReturnString += "~Z:";
        ReturnString += String(euler.z());
        ReturnString += "~";

        return ReturnString;
}
/**************************************************************************/
/*
    Initialize Chest
*/
/**************************************************************************/
void InitAllIMUs(void)
{
  Inithead();
  
  InitChest();
  
  InitbicepR();
  InitForeARmR();
  InitHandR();
  
  InitbicepL();
  InitForeARmL();
  InitHandL();
  
  Initwaist();
  
  InitthighR();
  InitkneeR();
  InitfootR();
  
  InitthighL();
  InitkneeL();
  InitfootL();
  
      PrintLCDLine1("DoneInitializing");
      PrintLCDLine2("Button to Cal.");
}
/**************************************************************************/
/*
    Read a Line of Calibration Data
*/
/**************************************************************************/
bool ReadLineCalData(String LineStartText)
{
  bool AllCalibrated = true;
    Serial.print(LineStartText);
    if(chestPresent)
    {
       if(!displayCalStatus(CHEST))
       {
        AllCalibrated = false;
        PrintLCDLine1("Chest");
        PrintLCDLine2("NOT CALIBRATED!");
       }
    }
    if(handRPresent)
    {
       if(!displayCalStatus(HAND_R))
       {
        AllCalibrated = false;
        PrintLCDLine1("Right Hand");
        PrintLCDLine2("NOT CALIBRATED!");
       }
    }
    if(bicepRPresent)
    {
       if(!displayCalStatus(BICEP_R))
       {
        AllCalibrated = false;
        PrintLCDLine1("Right Bicep");
        PrintLCDLine2("NOT CALIBRATED!");
       }
    }
    if(forearmRPresent)
    {
       if(!displayCalStatus(FOREARM_R))
       {
        AllCalibrated = false;
        PrintLCDLine1("Right ForeArm");
        PrintLCDLine2("NOT CALIBRATED!");
       }
    }
    if(headPresent)
    {
       if(!displayCalStatus(HEAD))
       {
        AllCalibrated = false;
        PrintLCDLine1("Head");
        PrintLCDLine2("NOT CALIBRATED!");
       }
    }
    if(waistPresent)
    {
       if(!displayCalStatus(WAIST))
       {
        AllCalibrated = false;
        PrintLCDLine1("Waist");
        PrintLCDLine2("NOT CALIBRATED!");
       }
    }
    if(thighRPresent)
    {
       if(!displayCalStatus(THIGH_R))
       {
        AllCalibrated = false;
        PrintLCDLine1("Right Thigh");
        PrintLCDLine2("NOT CALIBRATED!");
       }
    }
    if(kneeRPresent)
    {
       if(!displayCalStatus(KNEE_R))
       {
        AllCalibrated = false;
        PrintLCDLine1("Right Knee");
        PrintLCDLine2("NOT CALIBRATED!");
       }
    }
    if(footRPresent)
    {
       if(!displayCalStatus(FOOT_R))
       {
        AllCalibrated = false;
        PrintLCDLine1("Right Foot");
        PrintLCDLine2("NOT CALIBRATED!");
       }
    }
    if(thighLPresent)
    {
       if(!displayCalStatus(THIGH_L))
       {
        AllCalibrated = false;
        PrintLCDLine1("Right Thigh");
        PrintLCDLine2("NOT CALIBRATED!");
       }
    }
    if(kneeLPresent)
    {
       if(!displayCalStatus(KNEE_L))
       {
        AllCalibrated = false;
        PrintLCDLine1("Left Knee");
        PrintLCDLine2("NOT CALIBRATED!");
       }
    }
    if(footLPresent)
    {
       if(!displayCalStatus(FOOT_L))
       {
        AllCalibrated = false;
        PrintLCDLine1("Left Foot");
        PrintLCDLine2("NOT CALIBRATED!");
       }
    }
    if(AllCalibrated){
      Serial.println("All Sensors Calibrated");
      PrintLCDLine1("All SensorsCal'd");
      PrintLCDLine2("Button to Record");
    } else {
      Serial.println("");
    }
    return AllCalibrated;
}
/**************************************************************************/
/*
    Read a Line of Sensor Data
*/
/**************************************************************************/
void ReadLineOfData(String LineStartText, File dataFile, unsigned long refTime)
{
    
    String FinalTxtLine = LineStartText;
    FinalTxtLine += "  ~Take";
    FinalTxtLine += String(TakeNumber, DEC);
    FinalTxtLine += "~ ";

    FinalTxtLine += "  ~Time";
    FinalTxtLine += String(refTime, DEC);
    FinalTxtLine += "~  ";
    if(chestPresent)
    {
      // Possible vector values can be:
      // - VECTOR_ACCELEROMETER - m/s^2
      // - VECTOR_MAGNETOMETER  - uT
      // - VECTOR_GYROSCOPE     - rad/s
      // - VECTOR_EULER         - degrees
      // - VECTOR_LINEARACCEL   - m/s^2
      // - VECTOR_GRAVITY       - m/s^2
      if (QuatMode){
        // Quaternion data
        imu::Quaternion quat = chest.getQuat();
        FinalTxtLine += printQuat(quat,NAMECHEST);
      }else
      {
        imu::Vector<3> euler = chest.getVector(Adafruit_BNO055::VECTOR_EULER);
        FinalTxtLine += printEuler(euler,NAMECHEST);
      }
      FinalTxtLine += "  ";
      
    }
    if(handRPresent)
    {
      if (QuatMode){
        // Quaternion data
        imu::Quaternion quathandr = handR.getQuat();
        FinalTxtLine += printQuat(quathandr,NAMEHAND_R);
      }else
      {
        imu::Vector<3> euler = handR.getVector(Adafruit_BNO055::VECTOR_EULER);
        // Display the floating point data 
        FinalTxtLine += printEuler(euler,NAMEHAND_R);
      }
      FinalTxtLine += "  ";
    }
    if(bicepRPresent)
    {
      if (QuatMode){
        // Quaternion data
        imu::Quaternion quatbicepr = bicepR.getQuat();
        FinalTxtLine += printQuat(quatbicepr,NAMEBICEP_R);
      }else
      {
        imu::Vector<3> euler = bicepR.getVector(Adafruit_BNO055::VECTOR_EULER);
        // Display the floating point data 
        FinalTxtLine += printEuler(euler,NAMEBICEP_R);
      }
      FinalTxtLine += "  ";
    }
      
    if(forearmRPresent)
    {
      if (QuatMode){
        // Quaternion data
        imu::Quaternion quatforearmR = forearmR.getQuat();
        FinalTxtLine += printQuat(quatforearmR,NAMEFOREARM_R);
      }else
      {
        imu::Vector<3> euler = forearmR.getVector(Adafruit_BNO055::VECTOR_EULER);
        // Display the floating point data 
        FinalTxtLine += printEuler(euler,NAMEFOREARM_R);
      }
      FinalTxtLine += "  ";
    }
      
    if(forearmLPresent)
    {
      if (QuatMode){
        // Quaternion data
        imu::Quaternion quatforearmL = forearmL.getQuat();
        FinalTxtLine += printQuat(quatforearmL,NAMEFOREARM_L);
      }else
      {
        imu::Vector<3> euler = forearmL.getVector(Adafruit_BNO055::VECTOR_EULER);
        // Display the floating point data 
        FinalTxtLine += printEuler(euler,NAMEFOREARM_L);
      }
      FinalTxtLine += "  ";
    }
    if(bicepLPresent)
    {
      if (QuatMode){
        // Quaternion data
        imu::Quaternion quatbicepL = bicepL.getQuat();
        FinalTxtLine += printQuat(quatbicepL,NAMEBICEP_L);
      }else
      {
        imu::Vector<3> euler = bicepL.getVector(Adafruit_BNO055::VECTOR_EULER);
        // Display the floating point data 
        FinalTxtLine += printEuler(euler,NAMEBICEP_L);
      }
      FinalTxtLine += "  ";
    }
    if(handLPresent)
    {
      if (QuatMode){
        // Quaternion data
        imu::Quaternion quathandL = handL.getQuat();
        FinalTxtLine += printQuat(quathandL,NAMEHAND_L);
      }else
      {
        imu::Vector<3> euler = handL.getVector(Adafruit_BNO055::VECTOR_EULER);
        // Display the floating point data 
        FinalTxtLine += printEuler(euler,NAMEHAND_L);
      }
      FinalTxtLine += "  ";
    }
    if(headPresent)
    {
      if (QuatMode){
        // Quaternion data
        imu::Quaternion quathead = head.getQuat();
        FinalTxtLine += printQuat(quathead,NAMEHEAD);
      }else
      {
        imu::Vector<3> euler = head.getVector(Adafruit_BNO055::VECTOR_EULER);
        // Display the floating point data 
        FinalTxtLine += printEuler(euler,NAMEHEAD);
      }
      FinalTxtLine += "  ";
    }
    if(waistPresent)
    {
      if (QuatMode){
        // Quaternion data
        imu::Quaternion quatwaist = waist.getQuat();
        FinalTxtLine += printQuat(quatwaist,NAMEWAIST);
      }else
      {
        imu::Vector<3> euler = waist.getVector(Adafruit_BNO055::VECTOR_EULER);
        // Display the floating point data 
        FinalTxtLine += printEuler(euler,NAMEWAIST);
      }
      FinalTxtLine += "  ";
    }
    if(thighLPresent)
    {
      if (QuatMode){
        // Quaternion data
        imu::Quaternion quatthighL = thighL.getQuat();
        FinalTxtLine += printQuat(quatthighL,NAMETHIGH_L);
      }else
      {
        imu::Vector<3> euler = thighL.getVector(Adafruit_BNO055::VECTOR_EULER);
        // Display the floating point data 
        FinalTxtLine += printEuler(euler,NAMETHIGH_L);
      }
      FinalTxtLine += "  ";
    }
    if(kneeLPresent)
    {
      if (QuatMode){
        // Quaternion data
        imu::Quaternion quatkneeL = kneeL.getQuat();
        FinalTxtLine += printQuat(quatkneeL,NAMEKNEE_L);
      }else
      {
        imu::Vector<3> euler = kneeL.getVector(Adafruit_BNO055::VECTOR_EULER);
        // Display the floating point data 
        FinalTxtLine += printEuler(euler,NAMEKNEE_L);
      }
      FinalTxtLine += "  ";
    }
    if(footLPresent)
    {
      if (QuatMode){
        // Quaternion data
        imu::Quaternion quattfootL = footL.getQuat();
        FinalTxtLine += printQuat(quattfootL,NAMEFOOT_L);
      }else
      {
        imu::Vector<3> euler = footL.getVector(Adafruit_BNO055::VECTOR_EULER);
        // Display the floating point data 
        FinalTxtLine += printEuler(euler,NAMEFOOT_L);
      }
      FinalTxtLine += "  ";
    }
    if(thighRPresent)
    {
      if (QuatMode){
        // Quaternion data
        imu::Quaternion quatthighR = thighR.getQuat();
        FinalTxtLine += printQuat(quatthighR,NAMETHIGH_R);
      }else
      {
        imu::Vector<3> euler = thighR.getVector(Adafruit_BNO055::VECTOR_EULER);
        // Display the floating point data 
        FinalTxtLine += printEuler(euler,NAMETHIGH_R);
      }
      FinalTxtLine += "  ";
    }
    if(kneeRPresent)
    {
      if (QuatMode){
        // Quaternion data
        imu::Quaternion quatkneeR = kneeR.getQuat();
        FinalTxtLine += printQuat(quatkneeR,NAMEKNEE_R);
      }else
      {
        imu::Vector<3> euler = kneeR.getVector(Adafruit_BNO055::VECTOR_EULER);
        // Display the floating point data 
        FinalTxtLine += printEuler(euler,NAMEKNEE_R);
      }
      FinalTxtLine += "  ";
    }
    if(footRPresent)
    {
      if (QuatMode){
        // Quaternion data
        imu::Quaternion quattfootR = footR.getQuat();
        FinalTxtLine += printQuat(quattfootR,NAMEFOOT_R);
      }else
      {
        imu::Vector<3> euler = footR.getVector(Adafruit_BNO055::VECTOR_EULER);
        // Display the floating point data 
        FinalTxtLine += printEuler(euler,NAMEFOOT_R);
      }
      FinalTxtLine += "  ";
    }
      /* New line for the next sample */
    FinalTxtLine += "PNTEND";
    Serial.println(FinalTxtLine);
    if(dataFile){
      PrintLine2File(FinalTxtLine, dataFile);
    }
  
}
/**************************************************************************/
/*
    Set Mux to Port with command Line
*/
/**************************************************************************/
void setMuxToPort(void)
{
  int incomingByte = 0;
  
  Serial.println("Enter Port To Set To:");
  
  Serial.println("");
  
  while(  Serial.available() == 0) {}
  incomingByte = Serial.read();
  
  Wire.beginTransmission(112); // transmit to device #112 (0x70)
                              // shifting mux to port six output.
  Wire.write(0x1 << incomingByte);             // sends value byte  
  Wire.endTransmission();     // stop transmitting
  
}
/**************************************************************************/
/*
    Run TPose Command with countdown.
*/
/**************************************************************************/
void RunTPose(File FileIn)
{

    Busy();
    Serial.println("Get into T Pose Position, taking TPose In:");
      PrintLCDLine1("TPose In..");
      PrintLCDLine2("Countdown...");
    delay(1000);  
    Busy();
    Serial.println("5 Seconds");
      PrintLCDLine1("55555");
      PrintLCDLine2("Get In TPose Now");
    delay(1000);  
    Busy();
    Serial.println("4 Seconds");
      PrintLCDLine1("4444");
      //PrintLCDLine2("Get In TPose Now");
    delay(1000);  
    Busy();
    Serial.println("3 Seconds");
      PrintLCDLine1("333");
      //PrintLCDLine2("Get In TPose Now");
    delay(1000);  
    Busy();
    Serial.println("2 Seconds");
      PrintLCDLine1("22");
      //PrintLCDLine2("Get In TPose Now");
    delay(1000);  
    Busy();
    Serial.println("1 Seconds");
      PrintLCDLine1("1");
      //PrintLCDLine2("Get In TPose Now");
    delay(1000);  
    Busy();
      File uselessFile;
      ReadLineOfData("TPOS", FileIn, 0);
  
}
/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Wire.begin();
  /*Some processors also support 10000 (low speed mode), 1000000 (fast mode plus) and 3400000 (high speed mode). */
  Wire.setClock(400000); //fastmode.... 100000 is normal
  QuatMode = true;
  Serial.begin(115200);
  Serial.println("");
  Serial.println("***************************************");
  Serial.println("KeeCap Motion Capture Intializing"); 
  Serial.println("            Version " + VersionString);
  Serial.println("***************************************");
  PrintLCDLine1("KeeCap Init");
  PrintLCDLine2("SDCard Init");

      
  Serial.print("Initializing SD card...");

  initializeSDCardForDataLogging();
  //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  SetupLEDs();
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT_PULLUP);
  CheckForLCDConnect();
  PrintLCDLine1("KeeCap v" + VersionString);
  PrintLCDLine2("Press to Init.");
}
/**************************************************************************/
/*
    Arduino loop function, 
*/
/**************************************************************************/
void loop(void)
{
  unsigned long StartTime;
  unsigned long CurrentTime;
  int incomingByte = 0;
  bool ButtonPushed = false;
  String TimeLine = "";
  Serial.println("Spark MoCap Version: " + VersionString);
  Serial.println("Use the Keyboard to select the following options");
  Serial.println("e : enumerate I2C devices");
  Serial.println("i : Initialize Available Devices");
  Serial.println("r : read from device (press any key to stop)");
  Serial.println("s : Set Mux to Port Number you enter");
  Serial.println("q : Set output read to quaternion mode.");
  Serial.println("w : Set output read to euler mode.");
  Serial.println("t : Begin Take by starting T Pose Process");
  Serial.println("c : calibrate Sensor Process");
  Serial.println("d : SD Card Detect and Display Results.");
  Serial.println("j : Print Current Time");
  Serial.println("v : Set Date and Time");
  Serial.println("P : Set Speed of Capture");
  Serial.println("DoneMenus");
  StatusAllGood();
  while(  Serial.available() == 0 && !ButtonPushed) {
    ButtonPushed = !digitalRead(buttonPin);
  }
  if(ButtonPushed){
    if(!IMUsInitialized && !IMUsCalibrated){
      incomingByte = 105; //initialize i
    }else 
    if(IMUsInitialized && !IMUsCalibrated){
      incomingByte = 99; //calibrate c
    } else {
      incomingByte = 114; //read data....
    }
  }else{
    incomingByte = Serial.read();
  }
  Busy();
  
  // keyboard 't' is 116 decimal
  if (incomingByte == 116)
  {    
    
    TakeNumber = TakeNumber + 1;
    File uselessFile;
      RunTPose(uselessFile);
  } else
  // keyboard 'v' is 118 decimal
  if (incomingByte == 118)
  {
  GetTimeFromUserAndSet();
  } else
  // keyboard 'p' is 112 decimal
  if (incomingByte == 112)
  {
  CaptureDelay = SetSpeed(CaptureDelay);
  } else
  // keyboard 'c' is 99 decimal
  if (incomingByte == 99)
  {    
    if(IMUsInitialized){
      while(!ReadLineCalData("Cal Status:") && (Serial.available() == 0)){
        Busy();
        delay(BNO055_SAMPLERATE_DELAY_MS);
      }
      IMUsCalibrated = true;
      File emptyFile;
      //have user walk around for several seconds
      PrintLCDLine1("Pace Back&Forth");
      //Current Start Time in tick counts
      StartTime = millis();
      CurrentTime = 0;
      String TimeString = "";
      int WalkTimeSeconds = 30;
      while(CurrentTime < WalkTimeSeconds*1000){
        TimeString = String((float(WalkTimeSeconds) - (float(CurrentTime)/1000)),1);
        PrintLCDLine2("Done In:" + TimeString);
        Serial.println("Walk Time" + TimeString);
        CurrentTime = millis() - StartTime;
        ReadLineOfData("CALPNT", emptyFile, CurrentTime);
        delay(100); 
      }
      Serial.println("Done Calibration Successful");
      PrintLCDLine1("Done Calibration");
      PrintLCDLine2("Button 2 Record");
      
     } else {
        StatusAllBad();
        Serial.println("You Must Initialize the Sensors Prior to Calibration");
        PrintLCDLine1("Cannot Cal.");
        PrintLCDLine2("Must Init 1st.");
        delay(2000); 
     }
   incomingByte = Serial.read();
  } else
  // keyboard 'r' is 114 decimal
  if (incomingByte == 114)
  {    
    if(IMUsInitialized && IMUsCalibrated){
      String filename = DataLoggingName();
      Serial.println("Creating File:" + filename);
      File dataFile = SD.open(filename, FILE_WRITE);

      //Run TPose
      RunTPose(dataFile);
      
      PrintLCDLine1("Recording 2 File");
      PrintLCDLine2(filename);
      //Current Start Time in tick counts
      StartTime = millis();
      
      //Printing Start Time..
      TimeLine = "STARTIME " + GetTime();
      Serial.println(TimeLine);
      if(dataFile){
        PrintLine2File(TimeLine,dataFile);
      }

      //main read loop
      ButtonPushed = false;
      while(Serial.available() == 0 && !ButtonPushed ) {
        //setting LED to blining red..
        Recording();
        //checking button to see if we stop.
        ButtonPushed = !digitalRead(buttonPin);
        
        CurrentTime = millis() - StartTime;
        ReadLineOfData("PNTSTART", dataFile, CurrentTime);
        delay(CaptureDelay);
      }
      PrintLCDLine1("Done:" + filename);
      PrintLCDLine2("Button 2 Record");
      if(dataFile){
        dataFile.close();
      }
        StatusAllBad();
        delay(1000); 
    }else {
      StatusAllBad();
      Serial.println("You Must Initialize and Calibrate ALL of the Sensors Prior to Calibration");
      delay(2000); 
    }
    incomingByte = Serial.read();
  } else 
  if (incomingByte == 100)
  {
    sdCardPresentandIntialized = DetectAndDisplaySDCardInfo();
  } else 
  if (incomingByte == 101)
  {
    enumerateI2CBus();
  } else 
  if (incomingByte == 106)
  {
    PrintLCDLine1("Current Time:");
    PrintLCDLine2(GetTime());
    Serial.println(GetTime());
  } else 
  if (incomingByte == 115)
  {
    setMuxToPort();
  }else 
  if (incomingByte == 105)
  {
    InitAllIMUs();
    IMUsInitialized = true;
    Serial.println("IMUs Initialized.");
  } else
  if (incomingByte == 113){
      Serial.println("QuatMode Mode Enabled.");
      QuatMode = true;
  }else
  if (incomingByte == 119){
      Serial.println("Euler Mode Enabled.");
      QuatMode = false;
  }else
  {   
    // say what you got:
    Serial.print("I didn't understand the char I received with value: ");
    Serial.println(incomingByte, DEC);
  }
}
