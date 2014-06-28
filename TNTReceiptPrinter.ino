/*************************************************** 
 This is a library for the Adafruit Thermal Printer
 
 Pick one up at --> http://www.adafruit.com/products/597
 These printers use TTL serial to communicate, 2 pins are required
 
 Adafruit invests time and resources providing this open source code, 
 please support Adafruit and open-source hardware by purchasing 
 products from Adafruit!
 
 Written by Limor Fried/Ladyada for Adafruit Industries.  
 MIT license, all text above must be included in any redistribution
 ****************************************************/

/* TODO: 
 * Store password in eeprom
 * New password command
 * Connection dropped checking
 * Commands for changing printer settings
 * End Connection Command
 */


// BOF preprocessor bug prevent - insert me on top of your arduino-code
// From: http://www.a-control.de/arduino-fehler/?lang=en
#if 1
__asm volatile ("nop");
#endif

#pragma mark - Communication Protocol Definition

/* Codes: P = Print V = Value
 *
 * P00 VPassword - Authorize User (Returns "Authorized" if the password is corrct)
 * P01 VNewPassword - Change the password
 *
 * P10 - Open Cash Drawer
 *
 * P20 - Wakeup printer (This command takes 50ms to execute)
 * P21 - Sleep printer
 * P22 - Take printer online
 * P23 - Take printer offline. Print commands sent after this will be ignored until P03 is called
 * P24 - Restore printer settings to default
 * P25 - Restore everything to defaults and reset
 * P26 - Reset Printer
 *
 * P30 VHello - Print Text (e.g. P02 VHello. "This is a quote." Now I'm done printing.)
 * P31 - Print default bitmap
 *
 * P40 - All settings to default
 * P41 V0 - Inverse (Value codes: 0 = Off, 1 = On)
 * P42 V0 - UpsideDown (Value codes: 0 = Off, 1 = On)
 * P43 V0 - Double Height (Value codes: 0 = Off, 1 = On)
 * P44 V0 - Double Width (Value codes: 0 = Off, 1 = On)
 * P45 V0 - Strike (Value codes: 0 = Off, 1 = On)
 * P46 V0 - Set Bold (Value codes: 0 = Off, 1 = On)
 * P47 V1 - Justify (Value codes: 0 = Left, 1 = Center, 2 = Right)
 * P48 V5 - Line Feed (e.g. F16 V5 Feeds 5 lines)
 * P49 V3 - Pixel Feed (e.g. F17 V3 Feeds 3 rows of pixels)
 * P50 V1 - Set printing size (line spacing??) (Value codes: 0 = Small, 1 = Medium, 2 = Large)
 * P51 V0 - Underline (Value codes: 0 = Off, 1 = Normal Underline, 2 = Thick Underline)
 * P52 V0 - Set Character Spacing (Values Codes represent character spacing in pixels???)
 * P53 V32 - Set Line Height (Values codes represent line height in pixels. Default is 32)
 * P54 V1 - Add Tabs (Values codes represent the number of tabs to add)
 */

#include <SoftwareSerial.h>
#include <Adafruit_Thermal.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include "TheBitmap.h"
#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include <iostream>
#include <MemoryFree.h>
#include <stdlib>

#pragma mark - Variable Definitions

//The ASCII buffer for recieving from serial:
#define BUFFER_SIZE 40
#define DEFAULT_PASSWORD "James"
#define VERSION_NUMBER 1
#define EEPROM_ADDRESS 0
#define DEBUG_PRINTING // Comment out to disable
#define EOL '\n'

// BLE Definitions
#define BLE_RDY 2 // BLE READY (data ready interrupt from BTLE)
#define BLE_RST 9 // BLE RESET (resetting board on startup)
#define BLE_REQ 10 // BLE REQ (SPI chip select)
#define BLE_MOSI 11 // BLE MOSI (SPI MOSI)
#define BLE_MISO 12 // BLE MISO (SPI MISO)
#define BLE_CLK 13 // BLE CLK (SPI clock)
//#define BTLEact 8 // BLE ACTIVE (lets host know BTLE is busy)

// Pin definitions
#define CASH_DRAWER_PIN 7 // This is the single yellow wire
#define PRINTER_RX_PIN 6 // This is the green wire
#define PRINTER_TX_PIN 5 // This is the yellow wire

// EEPROM Variables
int EEPROMAddress = 0;
char *password;
int versionNumber;

// Command buffer variables
static char commandBuffer[BUFFER_SIZE];
static int bufferIndex = 0;

// Bitmap printing variables
int bitmapWidth = 0;
int bitmapHeight = 0;
uint8_t bitmapValue;
uint8_t bitmapRow[45];
int bitmapRowIndex = 0;
boolean printingBitmap = false;
char hexString[2];

// Variables for BTLE Communication
bool authorized = 0;
bool validCommandReceived = 0;
aci_evt_opcode_t previousBLEStatus = ACI_EVT_DISCONNECTED;
aci_evt_opcode_t currentBLEStatus = ACI_EVT_DISCONNECTED;
Adafruit_BLE_UART BLEserial = Adafruit_BLE_UART(BLE_REQ, BLE_RDY, BLE_RST);

Adafruit_Thermal printer(PRINTER_RX_PIN, PRINTER_TX_PIN);
// For printer tabs
int numberOfTabs = 0;

#pragma mark - Method Definitions

//Data Handling
void processCommand();
void getCommand();
int parseCommandCode(char);
char *parseCommandString(char code);
void sendCommandToBLE(int dataCommand, int dataValue);

// Debugging
void terminal();

// EEPROM
void readEEPROMValues();
void writeEEPROMDefaults();
void writeEEPROMValues();

#pragma mark - Setup

// Code Start
void setup()
{
    // Cash Drawer
    pinMode(CASH_DRAWER_PIN, OUTPUT);
    
    // Serial setup
    Serial.begin(9600);
    // BLE setup
    BLEserial.begin();
    // Read password form EEPROM
    readEEPROMValues();
    // Printer powerup and setup
    printer.begin();
    
#ifdef DEBUG_PRINTING
    Serial.print(F("freeRam: "));
    Serial.print(freeMemory());
    Serial.println(F(" bytes"));
#endif
}

void loop()
{
    // Tell BLE to go to work (needs to happen frequently)
    BLEserial.pollACI();
    
    //get current status of BTLE
    currentBLEStatus = BLEserial.getState();
    
    // BLE Status change
    if (currentBLEStatus != previousBLEStatus)
    {
#ifdef DEBUG_PRINTING
        if (currentBLEStatus == ACI_EVT_DEVICE_STARTED)
        {
            Serial.println(F("* Advertising started"));
        }
        if (currentBLEStatus == ACI_EVT_CONNECTED)
        {
            Serial.println(F("* Connected!"));
        }
        if (currentBLEStatus == ACI_EVT_DISCONNECTED)
        {
            Serial.println(F("* Disconnected or advertising timed out"));
        }
#endif
        
        // Handle disconnections
        if((currentBLEStatus == ACI_EVT_DEVICE_STARTED || currentBLEStatus == ACI_EVT_DISCONNECTED) && previousBLEStatus == ACI_EVT_CONNECTED)
        {
            authorized = 0;
        }
        
        previousBLEStatus = currentBLEStatus;
    }
    
    // BLE connected, get commands if any
    if (currentBLEStatus == ACI_EVT_CONNECTED)
    {
        // start getting commands since BLE is connected
        getCommand();
    }
}

#pragma mark - Data Handling

void processCommand()
{
    if(authorized)
    {
#ifdef DEBUG_PRINTING
        Serial.print(F("commandBuffer:"));
        Serial.println(commandBuffer);
#endif
        
        // Determine which command was received
        switch(parseCommandCode('P'))
        {
            case 0: // P00 Password authentication
                validCommandReceived = 1;
                
                if(strcmp(password, parseCommandString('V')) == 0)
                {
                    authorized = 1;
                }
#ifdef DEBUG_PRINTING
                Serial.print(F("Password attempt:"));
                Serial.println(parseCommandString('V'));
#endif
                break;
            case 1: // P01 VNewPassword - Change the password
                validCommandReceived = 1;
                
                password = parseCommandString('V');
                writeEEPROMValues();
                break;
            case 10: // P10 - Open Cash Drawer
                validCommandReceived = 1;
                
#ifdef DEBUG_PRINTING
                Serial.print(F("Open Cash Drawer:"));
                Serial.println(password);
#endif
                
                digitalWrite(CASH_DRAWER_PIN, HIGH);
                // Wait 50 ms to ensure the solenoid has been energized
                delay(50);
                digitalWrite(CASH_DRAWER_PIN, LOW);
                delay(50);
                break;
            case 20: // P20 Wakeup Printer
                validCommandReceived = 1;
                
                printer.wake();
                break;
            case 21: // P21 Sleep Printer
                validCommandReceived = 1;
                
                printer.sleep();
                break;
            case 22: // P22 Take Printer Online
                validCommandReceived = 1;
                
                printer.online();
                break;
            case 23: // P23 Take Printer Offline
                validCommandReceived = 1;
                
                printer.offline();
                break;
            case 24: // P24 Restore printer settings to default
                validCommandReceived = 1;
                
                printer.setDefault();
                break;
            case 25: // P25 Restore EEPROM defaults and reset
                validCommandReceived = 1;
                
                writeEEPROMDefaults();
                printer.setDefault();
                
                printer.begin();
                break;
            case 26: // P26 Reset Printer
                printer.reset();
                break;
            case 30: // P30 Print text
                validCommandReceived = 1;
                
#ifdef DEBUG_PRINTING
                Serial.print(F("Printing text:"));
                Serial.println(parseCommandString('V'));
#endif
                
                printer.print(parseCommandString('V'));
                break;
            case 31: // P31 - Print default bitmap
                validCommandReceived = 1;
                
                printer.printBitmap(TheBitmapWidth, TheBitmapHeight, TheBitmap);
                break;
            case 40: // P40 - All settings to default
                validCommandReceived = 1;
                
                printer.setDefault();
                break;
            case 41: // P41 V0 - Inverse (Value codes: 0 = Off, 1 = On)
                if(parseCommandCode('V') == 0)
                {
                    validCommandReceived = 1;
                    printer.inverseOff();
                }
                else if(parseCommandCode('V') == 1)
                {
                    validCommandReceived = 1;
                    printer.inverseOn();
                }
                break;
            case 42: // P42 V0 - UpsideDown (Value codes: 0 = Off, 1 = On)
                if(parseCommandCode('V') == 0)
                {
                    validCommandReceived = 1;
                    printer.upsideDownOff();
                }
                else if(parseCommandCode('V') == 1)
                {
                    validCommandReceived = 1;
                    printer.upsideDownOn();
                }
                break;
            case 43: // P43 V0 - Double Height (Value codes: 0 = Off, 1 = On)
                if(parseCommandCode('V') == 0)
                {
                    validCommandReceived = 1;
                    printer.doubleHeightOff();
                }
                else if(parseCommandCode('V') == 1)
                {
                    validCommandReceived = 1;
                    printer.doubleHeightOn();
                }
                break;
            case 44: //  P44 V0 - Double Width (Value codes: 0 = Off, 1 = On)
                if(parseCommandCode('V') == 0)
                {
                    validCommandReceived = 1;
                    printer.doubleWidthOff();
                }
                else if(parseCommandCode('V') == 1)
                {
                    validCommandReceived = 1;
                    printer.doubleWidthOn();
                }
                break;
            case 45: // P45 V0 - Strike (Value codes: 0 = Off, 1 = On)
                if(parseCommandCode('V') == 0)
                {
                    validCommandReceived = 1;
                    printer.strikeOff();
                }
                else if(parseCommandCode('V') == 1)
                {
                    validCommandReceived = 1;
                    printer.strikeOn();
                }
                break;
            case 46: // P46 V0 - Set Bold (Value codes: 0 = Off, 1 = On)
                if(parseCommandCode('V') == 0)
                {
                    validCommandReceived = 1;
                    printer.boldOff();
                }
                else if(parseCommandCode('V') == 1)
                {
                    validCommandReceived = 1;
                    printer.boldOn();
                }
                break;
            case 47: // P47 V1 - Justify (Value codes: 0 = Left, 1 = Center, 2 = Right)
                if(parseCommandCode('V') == 0)
                {
                    validCommandReceived = 1;
                    printer.justify('L');
                }
                else if(parseCommandCode('V') == 1)
                {
                    validCommandReceived = 1;
                    printer.justify('C');
                }
                else if(parseCommandCode('V') == 2)
                {
                    validCommandReceived = 1;
                    printer.justify('R');
                }
                break;
            case 48: // P48 V5 - Line Feed (e.g. F16 V5 Feeds 5 lines)
                validCommandReceived = 1;
                
                printer.feed(parseCommandCode('V'));
                break;
            case 49: // P49 V3 - Pixel Feed (e.g. F17 V3 Feeds 3 rows of pixels)
                validCommandReceived = 1;
                
                printer.feedRows(parseCommandCode('V'));
                break;
            case 50: // P50 V1 - Set printing size (Value codes: 0 = Small, 1 = Medium, 2 = Large)
                if(parseCommandCode('V') == 0)
                {
                    validCommandReceived = 1;
                    printer.setSize('S');
                }
                else if(parseCommandCode('V') == 1)
                {
                    validCommandReceived = 1;
                    printer.setSize('M');
                }
                else if(parseCommandCode('V') == 2)
                {
                    validCommandReceived = 1;
                    printer.setSize('L');
                }
                break;
            case 51: // P51 V0 - Underline (Value codes: 0 = Off, 1 = Normal Underline, 2 = Thick Underline)
                validCommandReceived = 1;
                
                printer.underlineOn(parseCommandCode('V'));
                break;
            case 52: // P52 V0 - Set Character Spacing (Values Codes represent character spacing in pixels???)
                validCommandReceived = 1;
                
                printer.setCharSpacing(parseCommandCode('V'));
                break;
            case 53: // P53 V32 - Set Line Height (Values codes represent line height in pixels. Default is 32)
                validCommandReceived = 1;
                
                printer.setLineHeight(parseCommandCode('V'));
                break;
            case 54: // P54 V1 - Add Tabs (Values codes represent the number of tabs to add)
                validCommandReceived = 1;
                
                numberOfTabs = parseCommandCode('V');
                
                for(; numberOfTabs > 0; numberOfTabs --)
                {
                    printer.tab();
                }
                break;
            default:
                validCommandReceived = 0;
                break;
        }
        
        if (validCommandReceived)
        {
            BLEserial.print("AOK\n");
        }
        else
        {
            BLEserial.print("NOK\n");
        }
    }
    else
    {
        if(!authorized)
        {
            BLEserial.print("Not Authorized\n");
        }
    }
}

void getCommand()
{
#ifdef DEBUG_PRINTING
    if (BLEserial.available())
    {
        Serial.print(F("* "));
        Serial.print(BLEserial.available());
        Serial.println(F(" bytes available from BTLE"));
    }
#endif
    
    // If data is available
    while (BLEserial.available())
    {
        // Receive one character from BLE
        char c = BLEserial.read();
        
        // As long as character is not EOL and is not exceeding buffer size limit
        if (c != EOL && (bufferIndex < BUFFER_SIZE))
        {
            // Store character in command buffer
            commandBuffer[bufferIndex] = c;
#ifdef DEBUG_PRINTING
            // print received character (for debugging)
            Serial.print(commandBuffer[bufferIndex]);
#endif
            // Increment command buffer index
            bufferIndex ++;
        }
        // Now that a full command has been received
        else
        {
#ifdef DEBUG_PRINTING
            Serial.print(F("\n"));
#endif
            processCommand();
            //restart buffer
            bufferIndex = 0;
            memset(commandBuffer, '\0', bufferIndex);
        }
    }
}

// Function to retrieve command code value
int parseCommandCode(char code)
{
    char *ptr = commandBuffer;
    
    while(ptr && *ptr && ptr < commandBuffer + bufferIndex)
    {
        if(*ptr == code)
        {
            return atoi(ptr + 1);
        }
        
        ptr = strchr(ptr, ' ') + 1;
    }
    
    return -1;
}

// Function to retrieve command string value
char *parseCommandString(char code)
{
    char *ptr = commandBuffer;
    
    while(ptr && *ptr && ptr < commandBuffer + bufferIndex)
    {
        if(*ptr == code)
        {
            return ptr;
        }
        
        ptr = strchr(ptr, ' ') + 1;
    }
    
    return '\0';
}

// Function to prepare data and send via BTLE to iOS
void sendCommandToBLE(int dataCommand, int dataValue)
{
    char tempBuffer[5];
    char dataToSend[13];
    
    itoa(dataCommand, tempBuffer, 10);
    dataToSend[0] = 'C';
    
    int sizeBuffer = strlen(tempBuffer);
    
    //**for debugging**
    //Serial.println(sizeBuffer);
    //**end of debugging
    
    for (int i = 0; i < sizeBuffer; i++)
    {
        dataToSend[i+1] = tempBuffer[i];
    }
    
    //fill in empty spaces for command
    if(sizeBuffer == 1)
    {
        dataToSend[2] = ' ';
        dataToSend[3] = ' ';
        dataToSend[4] = ' ';
        dataToSend[5] = ' ';
    }
    else if(sizeBuffer == 2)
    {
        dataToSend[3] = ' ';
        dataToSend[4] = ' ';
        dataToSend[5] = ' ';
    }
    else if(sizeBuffer == 3)
    {
        dataToSend[4] = ' ';
        dataToSend[5] = ' ';
    }
    else
    {
        dataToSend[5] = ' ';
    }
    
    dataToSend[6] = 'D';
    
    itoa(dataValue, tempBuffer, 10);
    sizeBuffer = strlen(tempBuffer);
    //Serial.println(sizeBuffer);
    
    for (int i = 0; i < sizeBuffer; i++)
    {
        dataToSend[i+7] = tempBuffer[i];
    }
    
    //fill in empty spaces for value
    if(sizeBuffer == 1)
    {
        dataToSend[8] = ' ';
        dataToSend[9] = ' ';
        dataToSend[10] = ' ';
    }
    else if(sizeBuffer == 2)
    {
        dataToSend[9] = ' ';
        dataToSend[10] = ' ';
    }
    else if(sizeBuffer == 3)
    {
        dataToSend[10] = ' ';
    }
    else
    {
        //max length for dataValue acheived, no need to fill in empty spaces
    }
    
    dataToSend[11] = '\n';
    dataToSend[12] = '\0';
    
    //for debugging**
    Serial.print(F("sendDataBLE:"));
    Serial.println(dataToSend);
    
    //tell BTLE to go to work (needs to happen frequently)
    BLEserial.pollACI();
    //get current status of BTLE
    currentBLEStatus = BLEserial.getState();
    
    //BTLE connected
    if ((currentBLEStatus == ACI_EVT_CONNECTED))
    {
        BLEserial.print(dataToSend);
    }
}

void terminal()
{
    /*if(wifly.available())
    {
        char c = wifly.read();
        debug.write(c);
        
        // Loop back
        wifly.write(c);
    }
    if(debug.available())
    {
        wifly.write(debug.read());
    }*/
}

#pragma mark - EEPROM Methods

void readEEPROMValues()
{
    EEPROMAddress = EEPROM_ADDRESS;
    EEPROM_readAnything(&EEPROMAddress, versionNumber);
    
#ifdef DEBUG_PRINTING
    Serial.print(F("EEPROM versionNumber:"));
    Serial.println(versionNumber);
#endif
    
    // First powerup. Write defaults to EEPROM
    if(versionNumber == -1 || versionNumber > VERSION_NUMBER)
    {
        writeEEPROMDefaults();
    }
    
    // Version 1.0 Values/Order
    if(versionNumber <= 1)
    {
        EEPROM_readAnything(&EEPROMAddress, password);
        
#ifdef DEBUG_PRINTING
        Serial.println(F("Proper Version"));
        Serial.print(F("EEPROM Password:"));
        Serial.println(password);
#endif
    }
}
 
void writeEEPROMDefaults()
{
#ifdef DEBUG_PRINTING
    Serial.println(F("EEPROM Defaults"));
#endif
    
    // Defaults
    versionNumber = VERSION_NUMBER;
    password = DEFAULT_PASSWORD;
    writeEEPROMValues();
    
    // Read the new version Number
    EEPROMAddress = EEPROM_ADDRESS;
    EEPROM_readAnything(&EEPROMAddress, versionNumber);
}

void writeEEPROMValues()
{
    EEPROMAddress = EEPROM_ADDRESS;
    
    // Version 1.0 Values/Order
    if(versionNumber <= 1)
    {
        EEPROM_writeAnything(&EEPROMAddress, versionNumber);
        EEPROM_writeAnything(&EEPROMAddress, password);
    }
}
