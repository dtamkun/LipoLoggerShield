//         Program: Lipo Logger Shield
// 
//     Description: This is the file containing the code for the Lipo Gas 
//                  Gauge Logger using the Data Logging Shield from Adafruit
//                  and a 1.8 inch TFT LCD display, also from Adafruit.
// 
//          Author: Dave Tamkun 
// 
//         History: DMT 07/02/2011 Created Original Version
//                  DMT 10/23/2011 Corrected Wiring info in Contents
//
//    Compiliation: Arduino IDE
//
//        Hardware: Arduino Uno - ATMEGA328P-PU - 16Mhz
//
//      Components: Gas Gauge on Proto Board with: 
//                       Maxim DS2764+025 Lipo Protector Chip
//                            i2C Slave Address 0x34
//
//                  Adafruit Data Logging Shield
//                  1.8" TFT LCD from Adafruit
//
//       Libraries: avr/pgmspace    needed for PROGMEM
//                  PString
//                  Wire            required for I2C communication with Gas Gauge
//                  RTClib          Be sure to get Version 3 for Arduino 1.0 Compatibility 
//                  DS2764          My Library for the Gas Gauge Chip
//                  SD              Adafruit's SD Card Library
//                  ST7735          Adafruit's 1.8" TFT LCD Library
//                  SPI
//
//
//   Communication: Uses i2C for RTC and Gas Gauge
//                       Hardware SPI for TFT LCD and SD Card
//                       Note that the SD Card onboard the LCD
//                       is not used.
//
//         Voltage: 5 Volts OK
//
//          Status: Experimental
//                  Work in Progress
//                  Stable           <=========
//                  Complete
//
//          Wiring: Data Logging Shield from Adafruit
//                  Ard. Pin     Device Pin   Device Use                                                         
//                  ----------   ----------   -------------------------------------------------------------------
//                  A4                        i2c for RTC - DAT                                                  
//                  A5                        i2c for RTC - CLK
//                  D6                        Green LED - Data Write to SD Card
//                  D7                        Red LED   - Data Sample Captured
//                  D10                       SS   - SD Card                                                     
//                  D11                       MOSI - HW SPI                                                      
//                  D12                       MISO - HW SPI                                                      
//                  D13                       SCK  - HW SPI                                                      
//
//                  1.8" TFT LCD from Adafruit 
//                  Ard. Pin     Device Pin   Device Use                                                         
//                  ----------   ----------   -------------------------------------------------------------------
//                  GND          GND                                                                             
//                  +5v          VCC                                                                             
//                  D8           Reset        Optional or could connect to Arduino Reset Pin                     
//                  D9           D/C          Data / Command Select                                              
//                  n/c          CARD_CS      SD Card Chip/Slave Select (can change pin, optional if no card use)
//                  D4           TFT_CS       TFT LCD Chip/Slave Select                                          
//                  D11          MOSI         HW SPI Master Out Serial In                                           
//                  D13          SCK          HW SPI Clock                                                          
//                  D12          MISO         HW SPI Master In Serial Out (optional if no SD card use)              
//                  +5V          LITE         Backlight +5v, can PWM                                             
//
//                  Gas Gauge Chip Proto Board - Maxim DS2764+025
//                  Ard. Pin     Device Pin   Device Use                                                         
//                  ----------   ----------   -------------------------------------------------------------------
//                  GND          GND          Ground                                                             
//                  A4           DAT          i2C - Data                                                         
//                  A5           CLK          i2C - Clock                                                        


//******************************************************************************
//** Define Options
//******************************************************************************
// the digital pins that connect to the LEDs
#define redLEDpin       7     // indicates sampling of gas gauge
#define greenLEDpin     6     // indicates log data being flushed to SD card

// for the data logging shield, we use digital pin 10 for the SD cs line
// The chip select pin for the SD card on the shield
// Use 10 for the Data Logging Shield
// Use 5  for the TFT LCD TouchShield
#define SD_CS           10

#define cs              4     // TFT LCD "Chip Select" for MEGAs you probably want this to be pin 53
#define dc              9     // TFT LCD "Data/Command" pin
#define rst             8     // TFT LCD "Reset" pin, you can also connect this to the Arduino reset

#define LOG_INTERVAL    1500  // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL   10000 // mills between calls to flush() - to write data to the card


#define NUMCOLS         21    // number of LCD Columns
#define NUMROWS         8     // number of LCD Lines
//#define LINEBUFSIZE    (NUMCOLS * NUMROWS) + 1    // 6 lines of 14 characters plus trailing NULL
#define LINEBUFSIZE     24

#define TFT_ROW_HEIGHT  9     // number of pixels for each row
#define TFT_COL_WIDTH   6     // number of pixels for each column

// String Buffer Sizes
#define FLOATBUFSIZE    10    // size for buffer string to contain character equivalents of floating point numbers
#define TEMPBUFSIZE     FLOATBUFSIZE + 1
#define CURRENTBUFSIZE  FLOATBUFSIZE + 1
#define VOLTBUFSIZE     FLOATBUFSIZE + 1

// Color definitions
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF
#define GRAY            0x94AF 

//My color Definitions
#define DT_DARK_GREEN   0x05A0
#define DT_DARKER_GREEN 0x0400    // Battery Level Color over 30%

// Color Usage
#define LABEL_COLOR     CYAN
#define DATA_COLOR      WHITE
#define DISABLED_COLOR  GRAY
#define ERROR_COLOR     RED
#define WARN_COLOR      YELLOW
#define GOOD_COLOR      GREEN


// x,y position of upper left corner of battery image
#define BATX    20
#define BATY    137

// Width and Height of Battery Image
#define BATW    78
#define BATH    20

// x,y position of upper left corner of positive terminal on battery image
//     followed by width and height
#define TERMX   98
#define TERMY   142
#define TERMW   5
#define TERMH   10



//******************************************************************************
//** Include Library Code
//******************************************************************************
#include <avr/pgmspace.h>   // needed for PROGMEM
#include <PString.h>
#include <Wire.h>           // required for I2C communication with Gas Gauge
#include "RTClib.h"
#include "DS2764.h"
#include <SD.h>
#include <ST7735.h>
#include <SPI.h>



//******************************************************************************
//** Declare Global Variables
//******************************************************************************
RTC_DS1307 RTC;                                       // define the Real Time Clock object
DS2764     gasGauge                  = DS2764();      // Gas Gauge Object
ST7735     tft                       = ST7735(cs, dc, rst);    

uint32_t   syncTime                   = 0;            // Used to track time of last Flush to SD Card

const int  chipSelect                 = SD_CS;
int        iRedraw                    = 1;            // 1 means clear and redraw the entire screen
char       filename[]                 = "BATLOG00.CSV";
File       logfile;                                   // the logging file
char       gszLineBuf[LINEBUFSIZE];
char       gszTempBuf[TEMPBUFSIZE];
char       gszCurrBuf[CURRENTBUFSIZE];
char       gszPctBuf[TEMPBUFSIZE];
char       gszVoltBuf[VOLTBUFSIZE];
      
PString    pstrLine   (gszLineBuf, LINEBUFSIZE);
PString    pstrTemp   (gszTempBuf, TEMPBUFSIZE);
PString    pstrCurrent(gszCurrBuf, CURRENTBUFSIZE);
PString    pstrPercent(gszPctBuf,  TEMPBUFSIZE);
PString    pstrVolts  (gszVoltBuf, VOLTBUFSIZE);
      
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
    
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(10,          OUTPUT);
    delay(500);
    pinMode(redLEDpin,   OUTPUT);
    pinMode(greenLEDpin, OUTPUT);

    Serial.print("Free Memory: ");
    Serial.println(memoryFree(), DEC);
    
    fillBuffer(gszLineBuf, LINEBUFSIZE,    '\0');
    fillBuffer(gszTempBuf, TEMPBUFSIZE,    '\0');
    fillBuffer(gszCurrBuf, CURRENTBUFSIZE, '\0');
    fillBuffer(gszVoltBuf, VOLTBUFSIZE,    '\0');

    Wire.begin();        // join i2c bus (address optional for master)

    // connect to RTC
    if (!RTC.begin()) {
      Serial.println("RTC failed");
    }    
    
    gasGauge.dsInit();
    
    delay(500);
  
    tft.initR();               // initialize a ST7735R chip
    tft.writecommand(ST7735_DISPON);
    tft.fillScreen(BLACK);
  
    // Use the setRotation Command to change the alignment
    // of the TFT Display.
    //   0 = Portrait,  SD card to the bottom
    // 192 = Portrait,  SD card to the top
    // 160 = Landscape, SD card to the left
    //  96 = Landscape, SD card to the right
    //tft.setRotation(192);
  
    delay(1000);  

    Serial.print("Initializing SD card...");

    if (!SD.begin(SD_CS)) {
      Serial.println("failed!");
      return;
    }
    Serial.println("SD OK!");
  
    // create a new file
    //char filename[] = "BATLOG00.CSV";
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

    gasGauge.dsSetPowerSwitchOn();
    delay(500);
    
    // Reset any Protection Flags, so the Gas Gauge can re-evaluate them  
    gasGauge.dsResetProtection(DS_RESET_ENABLE);

    Serial.print("Free Memory: ");
    Serial.println(memoryFree(), DEC);
  
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
    DisplayData();
    delay(LOG_INTERVAL);
} 






void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);

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
  
    int      i             = 0;
    int      iPrecision    = 1;
    char     szHi[]        = "HI";
    char     szLo[]        = "LO";
    char     szOk[]        = "Ok";
    char     szOn[]        = "On";
    char     szOff[]       = "Off";
    char*    szVoltStat    = NULL;
    char*    szChrgStat    = NULL;
    char*    szDChrgStat   = NULL;
    char*    szChrgOn      = NULL;
    char*    szDChrgOn     = NULL;
    DateTime dtNow;
    float    fPct          = 0.0;
    uint16_t uiBatColor    = WHITE;
    uint16_t uiVoltColor   = GREEN;
    uint16_t uiChgColor    = GREEN;
    uint16_t uiDChgColor   = GREEN;
    uint16_t uiChgEColor   = WHITE;
    uint16_t uiDChgEColor  = WHITE;
    int      iRectWidth    = 0;        // width of shaded area on battery image
                                       // showing current battery capacity remaining
 
   
    //******************************************************************************
    //** Get and Format Data for Logging and/or Display
    //******************************************************************************
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
    
    fPct = gasGauge.dsGetBatteryCapacityPercent();
    
    pstrCurrent.print(gasGauge.dsGetCurrent(),               iPrecision);    
    pstrTemp.print   (gasGauge.dsGetTempF(),                          1);        
    pstrPercent.print(fPct,                                           1);
    pstrVolts.print ((gasGauge.dsGetBatteryVoltage() / 1000.0),       2);
    
    // set Voltage Status
    i = gasGauge.dsGetVoltageStatus();
    if (i == DS_VOLTS_HI) { 
        szVoltStat = szHi; 
        uiVoltColor = ERROR_COLOR;
    }
    else if (i == DS_VOLTS_LOW) { 
        szVoltStat = szLo; 
        uiVoltColor = ERROR_COLOR;
    }
    else {
        szVoltStat = szOk;
        uiVoltColor = GOOD_COLOR; 
    }

    i = gasGauge.dsGetChargeStatus();
    // set Charge Current Status
    if (i == DS_CHARGE_CURRENT_HI) {
        // charge current is over the threshold 
        szChrgStat = szHi;
        uiChgColor = ERROR_COLOR;
    }
    else {
        szChrgStat = szOk;
        uiChgColor = GOOD_COLOR;
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
        uiDChgColor = ERROR_COLOR;
    }
    else {
        szDChrgStat = szOk; 
        uiDChgColor = GOOD_COLOR;
    } 
    
    
    if (gasGauge.dsIsDischargeOn()) { 
        szDChrgOn = szOn;
    }
    else {
        szDChrgOn = szOff;
    }
    
    // Sampling Done
    digitalWrite(redLEDpin, LOW);
    
        
    //******************************************************************************
    //** Log the Data
    //******************************************************************************
    
    //06/20/2011 16:05:20,687.5,3879,3288,49.8,89.2,Ok,1,On,Ok,1,On,Ok
    //timestamp,current(mAh),voltage(mV),acc curr(mAh),% of Cap,Temp,VoltStat,ChrgEnabled,ChrgOn,ChrgStat,DChrgEnabled,DChrgOn,DChrgStat    
    pstrLine.format("%02d/%02d/%04d %02d:%02d:%02d,", dtNow.month(), dtNow.day(), dtNow.year(), dtNow.hour(), dtNow.minute(), dtNow.second());
    logfile.print(gszLineBuf);
    
    logfile.print(gasGauge.dsGetCurrent(), 1);
    logfile.print(",");
    
    logfile.print(gasGauge.dsGetBatteryVoltage());
    logfile.print(",");
    
    logfile.print(gasGauge.dsGetAccumulatedCurrent());
    logfile.print(",");
    
    logfile.print(fPct, 1);
    logfile.print(",");
    
    logfile.print(gasGauge.dsGetTempF(), 1);
    logfile.print(",");
    
    //Voltage
    logfile.print(szVoltStat);
    logfile.print(",");
    
    //charging enabled?
    if(gasGauge.dsIsChargeEnabled()) {
        logfile.print("1,");
        uiChgEColor = LABEL_COLOR;
    }
    else {
        logfile.print("0,");
        uiChgEColor = DISABLED_COLOR;
    }
    
    logfile.print(szChrgOn);
    logfile.print(",");
    
    logfile.print(szChrgStat);
    logfile.print(",");
    
    
    if(gasGauge.dsIsDischargeEnabled()) {
        logfile.print("1,");
        uiDChgEColor = LABEL_COLOR;
    }
    else {
        logfile.print("0,");
        uiDChgEColor = DISABLED_COLOR;
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


         
    //******************************************************************************
    //** Display on 1.8" TFT LCD  128 x 160
    //******************************************************************************
    fPct = fPct / 100.0;
        
    if(fPct > 1.0) {
        fPct = 1.0;
    }
    else if(fPct < .01) {
        fPct = .01;
    }
    
    //fPct = .03;
    
    iRectWidth    = ((BATW - 3) * fPct);
        
    if(fPct > .34) {
        uiBatColor = DT_DARK_GREEN;
    }
    else if(fPct > .2) {
        uiBatColor = WARN_COLOR;
    }
    else {
        uiBatColor = ERROR_COLOR;
    }
    
    // Ran out of memory when trying to make the display fancier.
    // originally wanted to only refresh the dynamic parts of the screen
    // but the program size was too big for an UNO.
    if(iRedraw > 0) {
        tft.fillScreen(BLACK);    // Clear Screen by making it all black
                                  // this results in a noticeable flicker.
        
        // Show logfile name on Line 1
        pstrLine.begin();   
        pstrLine.format("Logfile: %12s", filename);
        tft.drawString(0, 0, gszLineBuf, LABEL_COLOR);
        
        // Line 2 is dynamic, contains date and time
        
        // Draw Line 3
        tft.drawString(0, (TFT_ROW_HEIGHT * 3), "Current:     Voltage:", LABEL_COLOR);

        // Draw Line 7
        tft.drawString(0, (TFT_ROW_HEIGHT * 6), "Accumulated          ", LABEL_COLOR);
        // Draw Line 8  
        tft.drawString(0, (TFT_ROW_HEIGHT * 7), "Current:     Temp  F:", LABEL_COLOR); 
        
        // Draw Battery Outline
        //              X     Y     W     H
        tft.drawRect(BATX, BATY, BATW, BATH, WHITE);
        //tft.drawRect(21, 138, 76, 18, BLUE);
    
        // Draw and fill Battery Positive Terminal
        tft.drawRect(TERMX, TERMY,  TERMW,     TERMH,     WHITE);
        tft.fillRect(TERMX, TERMY,  TERMW - 1, TERMH + 1, WHITE);
        //tft.drawRect(98, 142,  5, 10, BLUE);
        
        //iRedraw = 0;
    }
     

    // clear line 2
    //tft.fillRect((TFT_COL_WIDTH * 13), (TFT_ROW_HEIGHT * 1), (TFT_COL_WIDTH * 8), TFT_ROW_HEIGHT, BLUE);
    //pstrLine.begin();
    //pstrLine.print(byte(218));
    //fillBuffer(gszLineBuf, 21, byte(01));
    
    // redraw line 2
    pstrLine.begin();
    pstrLine.format("%02d/%02d/%04d   %02d:%02d:%02d", dtNow.month(), dtNow.day(), dtNow.year(), dtNow.hour(), dtNow.minute(), dtNow.second());
    tft.drawString(0, (TFT_ROW_HEIGHT * 1), gszLineBuf, DATA_COLOR);
    
    // clear line 5
    //pstrLine.begin();
    //fillBuffer(gszLineBuf, 20, ' ');
    //tft.drawString(0, (TFT_ROW_HEIGHT * 4), gszLineBuf, BLUE);
   // tft.fillRect((TFT_COL_WIDTH * 0), (TFT_ROW_HEIGHT * 4),  (TFT_COL_WIDTH * 6), TFT_ROW_HEIGHT, BLUE);
    //tft.fillRect((TFT_COL_WIDTH * 13), (TFT_ROW_HEIGHT * 4), (TFT_COL_WIDTH * 8), TFT_ROW_HEIGHT, BLUE);
    
    // redraw line 5  
    //pstrLine.begin();
    //pstrLine.format("%6smA     %4sV %2s", gszCurrBuf, gszVoltBuf, szVoltStat);
    //tft.drawString(0, (TFT_ROW_HEIGHT * 4), gszLineBuf, WHITE);
    pstrLine.begin();
    pstrLine.format("%6smA     %4sV", gszCurrBuf, gszVoltBuf);
    tft.drawString(0, (TFT_ROW_HEIGHT * 4), gszLineBuf, WHITE);
    tft.drawString((19 * 6), (TFT_ROW_HEIGHT * 4), szVoltStat, uiVoltColor);
             
    // Clear Line 9
    
    // Redraw Line 9
    pstrLine.begin();
    pstrLine.format("%4dmAh      %5s   ", gasGauge.dsGetAccumulatedCurrent(), gszTempBuf);
    tft.drawString(0, (TFT_ROW_HEIGHT * 8), gszLineBuf, DATA_COLOR);

    // Redraw Line 11 - since text is static and only color might change
    // we don't have to clear first.   
    tft.drawString(0, (TFT_ROW_HEIGHT * 10), "Charge  Stat./Current", uiChgEColor); 
    
    // Clear Line 12
    
    // Redraw Line 12
    pstrLine.begin();
    pstrLine.format("    %3s         %2s", szChrgOn, szChrgStat);
    tft.drawString(0, (TFT_ROW_HEIGHT * 11), gszLineBuf, uiChgColor);
  
  
    // Redraw Line 14 - since text is static and only color might change
    // we don't have to clear first. 
    tft.drawString(0, (TFT_ROW_HEIGHT * 13), "Dischrg Stat./Current", uiDChgEColor); 
    
    // clear line 15
    
    // redraw line 15
    pstrLine.begin();
    pstrLine.format("    %3s         %2s", szDChrgOn, szDChrgStat);
    tft.drawString(0, (TFT_ROW_HEIGHT * 14), gszLineBuf, uiDChgColor);
    
    // clear battery drained
    //tft.fillRect((BATX + 1 + iRectWidth), (BATY + 1), (BATW - 2 - iRectWidth), (BATH - 2), RED);
    
    // draw battery level
    tft.fillRect((BATX + 1), (BATY + 1), iRectWidth, (BATH - 2), uiBatColor);

    pstrLine.begin();
    pstrLine.format("       %5s%%", gszPctBuf);
    tft.drawString(0, (TFT_ROW_HEIGHT * 16), gszLineBuf, DATA_COLOR);
     
    /*   
    if (!(gasGauge.dsIsChargeEnabled())) {
        // Charging is disabled 
        tft.drawHorizontalLine(0, (TFT_ROW_HEIGHT * 10) + 4, (6*12), RED);
    }
    
    if (!(gasGauge.dsIsDischargeEnabled())) {
        //Discharging is disabled 
        tft.drawHorizontalLine(0, (TFT_ROW_HEIGHT * 13) + 4, (6*12), RED);
    }
    */
    
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





