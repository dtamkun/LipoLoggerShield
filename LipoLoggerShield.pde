//-----------------------------------------------------------------------
// $Id: LipoGasGauge.pde 78 2011-06-12 17:26:30Z davidtamkun $
//-----------------------------------------------------------------------
//         Program: Lipo Logger TFT
//         $Author: davidtamkun $
//           $Date: 2011-06-12 10:26:30 -0700 (Sun, 12 Jun 2011) $
//            $Rev: 78 $
//
// Source/Based On: Sample App by J.C. Woltz
//
//         History: DMT 02/22/2011 Created Original Version
//                  DMT 02/23/2011 v1.1.0 LCD to use i2c
//                  DMT 02/26/2011 V1.1.1 LCD using SPI.  Unable to get Log Shield Working
//                  DMT ??/??/2011 V1.2.0 
//                  DMT 03/31/2011 V1.3.0 Attempting to get Log Shield Working
//                  DMT 05/15/2011 V1.4.0 Giving up on Logging Shield for now.
//                                        Renamed file to LipoGasGauge.pde and
//                                        will keep under SVN going forward.
//                                        Added SVN Header Info and code
//                                        to display SVN Info
//                  DMT 05/28/2011 V1.4.0 Rev 46, 50 - still trying to get to work with
//                                        Nokia LCD.  Still having problems.  Rev 50
//                                        Working again with BIG LCD.
//                  DMT 05/30/2011 V1.5.0 Got this code working with Nokia Display on the
//                                        Mega and added % of Battery Capacity on the 20x4 LCD
//                  DMT 05/30/2011 V1.5.1 Confirmed that the Nokia LCD display now works on the
//                                        Uno.
//                  DMT 05/31/2011 V1.5.2 Corrected Nokia LCD display if current flow was
//                                        more than 5 characters.
//                  DMT 06/04/2011 V1.5.3 Enabled Gas Gauge Sleep Mode and Power Switch On/Off
//                                        code works, but needs improvement.
//                  DMT 06/05/2011 V1.6.0 Battery Capacity is now stored in the EEPROM of the
//                                        Gas Gauge chip, instead of being hardcoded.  Also
//                                        you can now set/reset the battery capacity and
//                                        accumulated current (gas level) from a menu
//                                        on the Serial Monitor.
//                  DMT 06/11/2011 V1.6.2 Added the capability to turn sleep mode on and
//                                        off from the serial menu.
//                  DMT 06/12/2011 V2.0.0 Split DS2764 code off into separate files which could
//                                        be in a separate library.  Removed old code from
//                                        logging shield and most of the debug code.  Also removed
//                                        all the version display code to get the memory size down.
//                                        Added logic to check for Discharges of more than 999 mA
//                                        and eliminated the decimal point for these values to
//                                        prevent the display from getting messed up.
//                  DMT 06/18/2011 V2.1.0 Moved DS2764 code into a separate library.
//
//    Compiliation: Arduino IDE
//
//        Hardware: Arduino Uno - ATMEGA328P-PU - 16Mhz
//
//      Components: Gas Gauge on Proto Board with: 
//                       Maxim DS2764+025 Lipo Protector Chip
//                            i2C Slave Address 0x34
//
//                  HD44780 Character 20x4 LCD (White on Blue)
//                  LCD Backback from Adafruit
//                  -- OR --
//                  Nokia LCD Display using PCD8544 driver chip
//
//                  NOT WORKING YET!! Data Logging Shield from Adafruit
//
//                
//       Libraries: Wire/Liquid Crystal for LCD Display
//                  PCD8544 if using Nokia Display
//                  SdFat & RTClib for Data Logging Shield
//
//
//   Communication: Uses i2C for RTC and Gas Gauge
//                       SPI for LCD, Nokia LCD, & SD Card
//
//         Voltage: 5 Volts OK
//
//          Status: Experimental      <=========
//                  Work in Progress
//                  Stable
//                  Complete
//
//          Wiring: LCD Backback from Adafruit
//                  ---------------------------------------------------
//                  SPI LCDBpck    Arduino
//                  -----------    -------
//                  GRN LATCH      D4      SPI Latch
//                  YEL DATA       D3      SPI Data
//                  ORN CLK        D2      SPI Clock
//                  RED +5V        +5v
//                  BLK GND        GND
//
//
//                  Dave's LCD Backpack for Nokia LCD Display - PCD8544
//                  ---------------------------------------------------
//                  MyBkpk         Arduino         Device
//                  Color          Pin             Pin
//                  ------         -------         ------
//                  Brown          GND             GND
//                  Red            +5V             VCC & Bklight LED
//                  Orange         D3              RST*
//                  Yellow         D4              CS*
//                  Green          D5              D/C*
//                  Blue           D6              DIN*
//                  Purple         D7              SCLK*
//                                                 * Level shifted
//
//
//                  Gas Gauge Chip Proto Board - Maxim DS2764+025
//                  ---------------------------------------------------
//                  Shield         Arduino
//                  -----------    -------
//                  GND            GND     Ground
//                  DATA Yellow    A4      i2C Data    (pin 20 on Mega)
//                  CLK  Orange    A5      i2C Clock   (pin 21 on Mega)
//                  
//-----------------------------------------------------------------------
/////////////////////////////////////////////////////////////////////
// Example Test application for the Maxim DS2764 IC. 
// A LiPo protection IC
/////////////////////////////////////////////////////////////////////
// J.C. Woltz 2010.11.15
/////////////////////////////////////////////////////////////////////
// This work is licensed under the Creative Commons 
// Attribution-ShareAlike 3.0 Unported License. To view a copy of 
// this license, visit http://creativecommons.org/licenses/by-sa/3.0/
// or send a letter to 
// Creative Commons, 
// 171 Second Street, 
// Suite 300, 
// San Francisco, California, 94105, USA.
/////////////////////////////////////////////////////////////////////



//******************************************************************************
//** Define Options
//******************************************************************************
//#define DEBUG          2    // Uncomment to display addl diagnostic msgs on the Serial Monitor

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  1000 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 10000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

// the digital pins that connect to the LEDs
//#define redLEDpin 2
//#define greenLEDpin 3
// for Data Logging Shield
#define redLEDpin        9
#define greenLEDpin      8


// for the data logging shield, we use digital pin 10 for the SD cs line
// The chip select pin for the SD card on the shield
// Use 10 for the Data Logging Shield
// Use 5  for the TFT LCD TouchShield
#define SD_CS 10



#define NUMCOLS        20    // number of LCD Columns
#define NUMROWS         4    // number of LCD Lines
//#define LINEBUFSIZE    (NUMCOLS * NUMROWS) + 1    // 6 lines of 14 characters plus trailing NULL
#define LINEBUFSIZE    24

// String Buffer Sizes
#define FLOATBUFSIZE        10    // size for buffer string to contain character equivalents of floating point numbers
//#define LOGBUFSIZE          20    // number of characters for a line written to the log
//#define DATEBUFSIZE         NUMCOLS + 1
//#define REVISIONBUFSIZE     NUMCOLS + 1

#define TEMPBUFSIZE         FLOATBUFSIZE + 1
#define CURRENTBUFSIZE      FLOATBUFSIZE + 1
#define VOLTBUFSIZE         FLOATBUFSIZE + 1

// Color definitions
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0 
#define WHITE           0xFFFF



//******************************************************************************
//** Include Library Code
//******************************************************************************
#include <avr/pgmspace.h>   // needed for PROGMEM
#include <PString.h>
#include <Wire.h>           // required for I2C communication with Gas Gauge
#include "RTClib.h"
#include "DS2764.h"
#include <SD.h>
#include <SPI.h>
//#include "TFTLCD.h"
#include <LiquidCrystal.h>


//******************************************************************************
//** Declare Global Variables
//******************************************************************************
RTC_DS1307 RTC; // define the Real Time Clock object

const int chipSelect = SD_CS;

// the logging file
File logfile;

      char gszLineBuf[LINEBUFSIZE];
      char gszTempBuf[TEMPBUFSIZE];
      char gszCurrBuf[CURRENTBUFSIZE];
      char gszPctBuf[TEMPBUFSIZE];
      char gszVoltBuf[VOLTBUFSIZE];
      
  PString  pstrLine   (gszLineBuf, LINEBUFSIZE);
  PString  pstrTemp   (gszTempBuf, TEMPBUFSIZE);
  PString  pstrCurrent(gszCurrBuf, CURRENTBUFSIZE);
  PString  pstrPercent(gszPctBuf,  TEMPBUFSIZE);
  PString  pstrVolts  (gszVoltBuf, VOLTBUFSIZE);
//  PString  pstrLog    (gszLogBuf,  LOGBUFSIZE);
      

DS2764 gasGauge     = DS2764();


// These are the pins as connected in the shield
//#define LCD_CS A3    // Chip Select goes to Analog 3
//#define LCD_CD A2    // Command/Data goes to Analog 2
//#define LCD_WR A1    // LCD Write goes to Analog 1
//#define LCD_RD A0    // LCD Read goes to Analog 0

// In the SD card, place 24 bit color BMP files (be sure they are 24-bit!)
// There are examples in the sketch folder

// our TFT wiring
//TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, 0);
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(2, 3, 4, 5, 6, 7 );


/************* HARDWARE SPI ENABLE/DISABLE */
// we want to reuse the pins for the SD card and the TFT - to save 2 pins. this means we have to
// enable the SPI hardware interface whenever accessing the SD card and then disable it when done
int8_t saved_spimode;

void disableSPI(void) {
  //saved_spimode = SPCR;
  //SPCR = 0;
}

void enableSPI(void) {
  //SPCR = saved_spimode; 
}
/******************************************/

// variables created by the build process when compiling the sketch
// used to calculate freeMemory
extern int __bss_end;
extern void *__brkval;



//******************************************************************************
//** Start Main Logic
//******************************************************************************


//------------------------------------------------------------------------------
// setup
//
// First function run by the microcontroller and is used to initialize and 
// set up variables and classes.
//
// Arguments:
//     None
//
// Return Value:
//     None
//------------------------------------------------------------------------------
void setup() { 
    Serial.begin(9600);          // start serial communication at 9600bps
    
    delay(500);
    // use debugging LEDs
    pinMode(redLEDpin, OUTPUT);
    pinMode(greenLEDpin, OUTPUT);

//    Serial.print("Free Memory: ");
//    Serial.println(memoryFree(), DEC);
    
    fillBuffer(gszLineBuf, LINEBUFSIZE, '\0');
    fillBuffer(gszTempBuf, TEMPBUFSIZE, '\0');
    fillBuffer(gszCurrBuf, CURRENTBUFSIZE,  '\0');
    fillBuffer(gszVoltBuf, VOLTBUFSIZE, '\0');

    Wire.begin();        // join i2c bus (address optional for master)
    
    gasGauge.dsInit();
    
    delay(500);
  
  /*     
    tft.reset();
  
  // find the TFT display
  uint16_t identifier = tft.readRegister(0x0);
  if (identifier == 0x9325) {
    Serial.println("Found ILI9325");
  } else if (identifier == 0x9328) {
    Serial.println("Found ILI9328");
  } else {
    Serial.print("Unknown driver chip ");
    Serial.println(identifier, HEX);
    while (1);
  }  
 
  tft.initDisplay();
  // the image is a landscape, so get into landscape mode
  tft.setRotation(1);
  
  tft.fillScreen(BLACK);
  tft.setTextColor(GREEN);
  tft.setTextSize(3);
  */
  
  // set up the LCD's number of rows and columns: 
  lcd.begin(NUMCOLS, NUMROWS);
  lcd.clear();


  // connect to RTC
  if (!RTC.begin()) {
      Serial.println("RTC failed");
  }

  // set System Clock from RTC
  //setSyncProvider(RTC.get);   // the function to get the time from the RTC
  
  //if(timeStatus()!= timeSet) { 
  //   Serial.println("Unable to sync with the RTC");
  //}
  //else {
  //   Serial.println("RTC has set the system time"); 
  //}
  
  enableSPI();  

  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  if (!SD.begin(SD_CS)) {
    Serial.println("failed!");
    return;
  }
  Serial.println("SD OK!");
  
  // create a new file
  char filename[] = "BATLOG00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    error("couldnt create file");
  }
  
  Serial.print("Logging to: "); 
  Serial.println(filename);

  
  logfile.println("timestamp,current(mAh),voltage(mV),acc curr(mAh),% of Cap,Temp,VoltStat,ChrgEnabled,ChrgOn,ChrgStat,DChrgEnabled,DChrgOn,DChrgStat");    

  
  disableSPI();    // release SPI so we can use those pins to draw
 
  // disable the SD card interface after we are done!
//  disableSPI();

    gasGauge.dsSetPowerSwitchOn();
    delay(500);
    
    // Reset any Protection Flags, so the Gas Gauge can re-evaluate them  
    gasGauge.dsResetProtection(DS_RESET_ENABLE);

//    Serial.print("Free Memory: ");
//    Serial.println(memoryFree(), DEC);
  
} 







//------------------------------------------------------------------------------
// loop
//
// Function called repeatedly after setup completes.  You must never "return"
// from this function.  Your main code execution goes here.
//
// Arguments:
//     None
//
// Return Value:
//     None
//------------------------------------------------------------------------------
void loop() { 
    
    gasGauge.dsRefresh();
    
    //disableSPI();    // release SPI so we can use those pins to draw   
    DisplayData();
    //enableSPI();
    
    delay(1500);
} 




void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  //digitalWrite(redLEDpin, HIGH);

  while(1);
}








// function to return the amount of free RAM
int memoryFree() {
    int freeValue;

    if((int)__brkval == 0)
        freeValue = ((int)&freeValue) - ((int)&__bss_end);
    else
        freeValue = ((int)&freeValue) - ((int)__brkval);
        
    return freeValue;
}





void DisplayData() {
  
    //Serial.println("Entering DisplayData()");
    int   i           = 0;
    int   iPrecision  = 1;
    char  szHi[]      = "HI";
    char  szLo[]      = "LO";
    char  szOk[]      = "Ok";
    char  szOn[]      = "On";
    char  szOff[]     = "Off";
    char* szVoltStat  = NULL;
    char* szChrgStat  = NULL;
    char* szDChrgStat = NULL;
    char* szChrgOn    = NULL;
    char* szDChrgOn   = NULL;
    DateTime    dtNow;

    
    digitalWrite(redLEDpin, HIGH);
    pstrLine.begin();
    pstrTemp.begin();
    pstrCurrent.begin();
    pstrPercent.begin();
    pstrVolts.begin();
    
    dtNow = RTC.now();
    
    
    
    if(gasGauge.dsGetCurrent() < -999.9) {
        iPrecision = 0;   
    }
    else {
        iPrecision = 1;
    }
    
    pstrCurrent.print(gasGauge.dsGetCurrent(),               iPrecision);    
    pstrTemp.print   (gasGauge.dsGetTempF(),                          1);        
    pstrPercent.print(gasGauge.dsGetBatteryCapacityPercent(),         1);
    pstrVolts.print ((gasGauge.dsGetBatteryVoltage() / 1000.0),       2);
    
    // set Voltage Status
    i = gasGauge.dsGetVoltageStatus();
    if (i == DS_VOLTS_HI) { 
        szVoltStat = szHi; 
    }
    else if (i == DS_VOLTS_LOW) { 
        szVoltStat = szLo; 
    }
    else {
        szVoltStat = szOk; 
    }

    i = gasGauge.dsGetChargeStatus();
    // set Charge Current Status
    if (i == DS_CHARGE_CURRENT_HI) {
        // charge current is over the threshold 
        szChrgStat = szHi;
    }
    else {
        szChrgStat = szOk;
    }     

    if (gasGauge.dsIsChargeOn()) { 
        szChrgOn = szOn;
    }
    else {
        szChrgOn = szOff;
    }


    i = gasGauge.dsGetDischargeStatus();
    // set Discharge Current Status
    if (i == DS_DISCHARGE_CURRENT_HI) {
        // discharge current is over the threshold 
        szDChrgStat = szHi; 
    }
    else {
        szDChrgStat = szOk; 
    } 
    
    
    if (gasGauge.dsIsDischargeOn()) { 
        szDChrgOn = szOn;
    }
    else {
        szDChrgOn = szOff;
    }
    
    // Sampling Done
    digitalWrite(redLEDpin, LOW);

    enableSPI();    // enable SD Card access
    
    //06/20/2011 16:05:20,687.5,3879,3288,49.8,89.2,Ok,1,On,Ok,1,On,Ok
    //timestamp,current(mAh),voltage(mV),acc curr(mAh),% of Cap,Temp,VoltStat,ChrgEnabled,ChrgOn,ChrgStat,DChrgEnabled,DChrgOn,DChrgStat
    
    pstrLine.format("%02d/%02d/%04d %02d:%02d:%02d,", dtNow.month(), dtNow.day(), dtNow.year(), dtNow.hour(), dtNow.minute(), dtNow.second());
    logfile.print(gszLineBuf);
    Serial.println(gszLineBuf);
    
    logfile.print(gasGauge.dsGetCurrent(), 1);
    logfile.print(",");
    
    logfile.print(gasGauge.dsGetBatteryVoltage());
    logfile.print(",");
    
    logfile.print(gasGauge.dsGetAccumulatedCurrent());
    logfile.print(",");
    
    logfile.print(gasGauge.dsGetBatteryCapacityPercent(), 1);
    logfile.print(",");
    
    logfile.print(gasGauge.dsGetTempF(), 1);
    logfile.print(",");
    
    //Voltage
    logfile.print(szVoltStat);
    logfile.print(",");
    
    //charging enabled?
    if(gasGauge.dsIsChargeEnabled()) {
        logfile.print("1,");
    }
    else {
        logfile.print("0,");
    }
    
    logfile.print(szChrgOn);
    logfile.print(",");
    
    logfile.print(szChrgStat);
    logfile.print(",");
    
    
    if(gasGauge.dsIsDischargeEnabled()) {
        logfile.print("1,");
    }
    else {
        logfile.print("0,");
    }
    
    logfile.print(szDChrgOn);
    logfile.print(",");
    
    logfile.println(szDChrgStat);
    
    //don't sync too often - requires 2048 bytes of I/O to SD card
    //This is where the buffered data gets written back to the SD Card.
    if ((millis() - syncTime) >=  SYNC_INTERVAL) {
        // blink LED to show we are syncing data to the card & updating FAT!
        digitalWrite(greenLEDpin, HIGH);
        logfile.flush();
        syncTime = millis();
        digitalWrite(greenLEDpin, LOW);
    }
     
    // restore SPI control to LCD.
    disableSPI();
    
    
    /*        
    if (iProtect & DS00CE) { 
        Serial.println("           Charging: Enabled"); 
    }
    else {
        Serial.println("           Charging: Disabled");
    }
    
    if (iProtect & DS00DE) { 
        Serial.println("        Discharging: Enabled");
    }
    else {
        Serial.println("        Discharging: Disabled");
    }
    */        

    
    // these are the global buffers
    //char   gszTempBuf[FLOATBUF    + 1];
    //char   gszCurrBuf[FLOATBUF    + 1];
    //char   gszLineBuf[NUMCOLS     + 1];
            
    // LCD Lines
    //         1  1  1   2
    //12345678901234567890
    //   -0.6mA     4191mV
    //Acc: 2110mAh  78.4oF
    //VOLT CHARGE  DCHARGE
    //Volts:OK Temp:100.1F
    //Chg:OK OFFDch:OK OFF
    
    pstrLine.begin();
    
    pstrLine.format("%6smA%5sV", gszCurrBuf, gszVoltBuf);
    lcd.setCursor(0, 0);
    lcd.print(gszLineBuf);
    
    pstrLine.begin();
    pstrLine.format("%4dmAh %5s%%", gasGauge.dsGetAccumulatedCurrent(), gszPctBuf);
    lcd.setCursor(0, 1);
    lcd.print(gszLineBuf);
    
    pstrLine.begin();
    pstrLine.format("Volts:%2s Temp:%5sF", szVoltStat, gszTempBuf);
    lcd.setCursor(0, 2);
    lcd.print(gszLineBuf);
    
    pstrLine.begin();
    pstrLine.format("Chg:%2s %-3sDch:%2s %-3s", szChrgOn, szChrgStat, szDChrgOn, szDChrgStat);
    lcd.setCursor(0, 3);
    lcd.print(gszLineBuf);
    
    
    
    //nokia.clear();    // clear the screen

    //Serial.println("got past Nokia.clear");
    
    
    
    //01234567890123
    //999.9mA 110.5o
    //2180mAh  99.9%
    //Voltage     OK
    //Charging    OK
    //Discharge   OK
    
    //123456789012341234567890123412345678901234123456789012341234567890123412345678901234
    //--------------==============--------------==============--------------==============
    //-700.0mA 3.78V1000mAh ---.-%Voltage     OKCharge  Off OKDCharge Off OKTemp:  102.6oF


    //pstrLine.format("%6smA%5sV%4dmAh %5s%%Voltage     %2sCharge  %3s %2sDcharge %3s %2sTemp:  %5s F", 
    //                gszCurrBuf, gszVoltBuf, gasGauge.dsGetAccumulatedCurrent(), gszPctBuf, szVoltStat, szChrgOn, szChrgStat, szDChrgOn, szDChrgStat, gszTempBuf);
 
    /*
    tft.fillScreen(BLACK);
    tft.setTextColor(GREEN);
    tft.setTextSize(3);
    tft.setCursor(0, 0);
    
    pstrLine.format("%6smA%5sV", gszCurrBuf, gszVoltBuf);
    tft.println(gszLineBuf);
    
    pstrLine.begin();
    pstrLine.format("%4dmAh %5s%%", gasGauge.dsGetAccumulatedCurrent(), gszPctBuf);
    tft.println(gszLineBuf);
    
    
    pstrLine.begin();
    pstrLine.format("Voltage     %2s", szVoltStat);
    tft.println(gszLineBuf);

    pstrLine.begin();
    pstrLine.format("Charge  %3s %2s", szChrgOn, szChrgStat);
    tft.println(gszLineBuf);

    pstrLine.begin();
    pstrLine.format("Dcharge %3s %2s", szDChrgOn, szDChrgStat);
    tft.println(gszLineBuf);
     
    pstrLine.begin();
    pstrLine.format("Temp:  %5s F", gszTempBuf);
    tft.println(gszLineBuf);
        
    */                
    //nokia.drawstring(0, 0, gszLineBuf);
    //tft.print(gszLineBuf);
    //tft.setTextColor(WHITE);
    //tft.setTextSize(1);
    
    //tft.println("         1         2         3         4         5");
    //tft.println("12345678901234567890123456789012345678901234567890");
    
    //tft.setTextColor(GREEN);
    //tft.setTextSize(2);
    //tft.println("12345678901234567890123456");
    
    //tft.setTextColor(BLUE);
    //tft.setTextSize(3);
    //tft.println("12345678901234567");
    
    
    //Serial.println(gszLineBuf);
    
    if (!(gasGauge.dsIsChargeEnabled())) {
        // Charging is disabled 
        //nokia.drawline(0, 3*8 + 4, 7*6, 3*8 + 4, BLACK); 
    }
    
    if (!(gasGauge.dsIsDischargeEnabled())) {
        //Discharging is disabled 
        //nokia.drawline(0, 4*8 + 4, 7*6, 4*8 + 4, BLACK); 
    }
    
    //nokia.drawbitmap(72, 40, degree_bmp, 5, 8, BLACK);
    
    //nokia.display();
    
    //if (logfile.writeError) {
    //    Serial.println("Write Error");
    //}
    

    // Sampling Done
    //digitalWrite(redLEDpin, LOW);

    //don't sync too often - requires 2048 bytes of I/O to SD card
    //This is where the buffered data gets written back to the SD Card.
    //if ((millis() - syncTime) <  SYNC_INTERVAL) return;
    //syncTime = millis();
  
    // blink LED to show we are syncing data to the card & updating FAT!
    //digitalWrite(greenLEDpin, HIGH);

    //if (!logfile.sync()) {
    //    Serial.println("Sync Error");
    //}

    //digitalWrite(greenLEDpin, LOW);

}
















//------------------------------------------------------------------------------
// fillBuffer
//
// Fills the specified character array with the specified character
//
// Arguments:
//     None
//
// Return Value:
//     None
//------------------------------------------------------------------------------
void fillBuffer(char* aszBuff, int aiSize, char acChar) {

  int i = 0;

  for(i = 0; i < aiSize; i++) {
    aszBuff[i] = acChar;
  }  
}




