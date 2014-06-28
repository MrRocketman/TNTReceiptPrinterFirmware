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

/* Codes: P = Print F = Format V = Value
 *
 * P00 VPassword - Authorize User (Returns "Authorized" if the password is corrct)
 * P01 VHello - Print Text (e.g. P02 VHello. "This is a quote." Now I'm done printing.)
 * P02 - Wakeup printer (This command takes 50ms to execute)
 * P03 - Sleep printer
 * P04 - Take printer online
 * P05 - Take printer offline. Print commands sent after this will be ignored until P03 is called
 * P06 VNewPassword - Change the password
 * P07 VNetworkName - Change the network name
 * P08 VWidth SHeight - Start printing bitmap. With width and height. Expects the appropiate amount of hex values and then a P09.
 * P09 VHex - Bitmap value
 * P10 - End printing bitmap
 * P11 - Print default bitmap
 * P12 - Open Cash Drawer
 * P97 - Restore printer settings to default
 * P98 - Restore everything to defaults and reset
 * P99 - Reset Printer
 *
 * F00 - All settings to default
 * F01 V0 - Inverse (Value codes: 0 = Off, 1 = On, F = Off, T = On)
 * F02 V0 - UpsideDown (Value codes: 0 = Off, 1 = On, F = Off, T = On)
 * F03 V0 - Double Height (Value codes: 0 = Off, 1 = On, F = Off, T = On)
 * F04 V0 - Double Width (Value codes: 0 = Off, 1 = On, F = Off, T = On)
 * F05 V0 - Strike (Value codes: 0 = Off, 1 = On, F = Off, T = On)
 * F06 V0 - Set Bold (Value codes: 0 = Off, 1 = On, F = Off, T = On)
 * F07 VL - Justify (Value codes: L = Left, C = Center, R = Right)
 * F08 V5 - Line Feed (e.g. F16 V5 Feeds 5 lines)
 * F09 V3 - Pixel Feed (e.g. F17 V3 Feeds 3 rows of pixels)
 * F10 VS - Set printing size (line spacing??) (Value codes: S = Small, M = Medium, L = Large)
 * F11 V0 - Underline (Value codes: 0 = Off, 1 = Normal Underline, 2 = Thick Underline)
 * F12 V0 - Set Character Spacing (Values Codes represent character spacing in pixels???)
 * F13 V32 - Set Line Height (Values codes represent line height in pixels. Default is 32)
 * F14 V1 - Add Tabs (Values codes represent the number of tabs to add)
 */

#pragma mark - WiFly and Main Variables

//The ASCII buffer for recieving from serial:
#define MAX_COMMAND_SIZE 256
#define BUFFER_SIZE 2
#define DEFAULT_PASSWORD "James"
#define DEFAULT_AP_NETWORK_NAME "TNT Receipt_Printer"
#define VERSION_NUMBER 1
#define EEPROM_ADDRESS 0
#define DEBUG_TO_WIFLY 0
#define DEBUG_TO_SERIAL 0
#define COMMAND_FINISHED_CHARACTER 5

#include <WiFlyHQ.h>
#include <SoftwareSerial.h>
#include <Adafruit_Thermal.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include "TheBitmap.h"

// Pin definitions
int statusPin = 13;
int cashDrawerPin = 7; // 7
int printerRXPin = 6;  // this is the green wire // 6
int printerTXPin = 5;  // this is the yellow wire // 5
int debugRXPin = 4;
int debugTXPin = 3;

WiFly wifly;
SoftwareSerial debug(debugRXPin, debugTXPin);
Adafruit_Thermal printer(printerRXPin, printerTXPin);

// EEPROM Variables
int EEPROMAddress = 0;
char *password;
int versionNumber;
char *apNetworkName;
char *joinNetorkName;
char *joinNetworkPassword;

boolean authorized = false;

static char commandBuffer[BUFFER_SIZE][MAX_COMMAND_SIZE];
static int bufferReadIndex = 0;
static int bufferWriteIndex = 0;
static int bufferLength = 0;
static char serialCharacter;
static int serialCount = 0;
static boolean commentMode = false;
static char *stringCharacterPointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc
long previousCommandMilliseconds = 0;

int bitmapWidth = 0;
int bitmapHeight = 0;
uint8_t bitmapValue;
uint8_t bitmapRow[45];
int bitmapRowIndex = 0;
boolean printingBitmap = false;
char hexString[2];

// Methods
void powerup();
int freeRam();
void printFreeRamToDebug();
void printFreeRamToDebugAndWifly();

void getCommand();
void processCommands();
void hexStringCodeValue();
char * stringCodeValue();
float floatCodeValue();
int intCodeValue();
bool codeSeen(char codeString[]);
bool codeSeen(char code);
void sendCommandFinished();
void FlushSerialRequestResend();
void ClearToSend();

bool stringIsTrueOrFalse(char *string);
void endConnectionAndReset();
void terminal();

void readEEPROMValues();
void writeEEPROMDefaults();
void writeEEPROMValues();

void debugPrint(PGM_P s);
void debugPrintln(PGM_P s);
void wiflyPrint(PGM_P s);
void wiflyPrintln(PGM_P s);

void convertHexCharacters(char *commandValues, int length);

#pragma mark - WiFly and Main Code

// Code Start
void setup()
{
    // Blinky Light
    pinMode(statusPin, OUTPUT);
    digitalWrite(statusPin, LOW);
    // Cash Drawer
    pinMode(cashDrawerPin, OUTPUT);
    digitalWrite(statusPin, LOW);
    
    // Serial setup
    Serial.begin(9600); // This is the wifly Serial
    debug.begin(19200);
    //printer.begin(19200);
    
    powerup();
}

void powerup()
{
    printFreeRamToDebug();
    
    // DO NOT PRINT ANYTHING TO WIFLY BEFORE IT IS BOOTED UP ELSE MAYHEM ENSUES! Don't do anything before it powers up for that matter
    
    // WiFly powerup
    if(!wifly.begin(&Serial, &debug)) 
    {
        if(DEBUG_TO_SERIAL)
            debugPrintln(PSTR("WiFly Power up FAILED!"));
            
            if(DEBUG_TO_SERIAL)
                debugPrintln(PSTR("AP Network Creation FAILED!"));
            
            // Blink light to show ap Network Created failure
            for(int i = 0; i < 3; i ++)
            {
                digitalWrite(statusPin, HIGH);
                delay(100);
                digitalWrite(statusPin, LOW);
                delay(100);
            }
        
        digitalWrite(statusPin, LOW);
    }
    // Wifly AP Network Creation/Network Joining. EEPROM Reading.
    else
    {
        if(DEBUG_TO_SERIAL)
            debugPrintln(PSTR("WiFly Powered up"));
            
        if(DEBUG_TO_SERIAL)
                debugPrintln(PSTR("AP Network Created"));
            
            digitalWrite(statusPin, HIGH);
            wifly.setIpProtocol(WIFLY_PROTOCOL_TCP);
        
        // Read password form EEPROM
        readEEPROMValues();
        
        // AP Network Creation Success
        /*if(wifly.createSoftAPNetwork(apNetworkName, 8))
        {
            if(DEBUG_TO_SERIAL)
                debugPrintln(PSTR("AP Network Created"));
            
            digitalWrite(statusPin, HIGH);
            wifly.setIpProtocol(WIFLY_PROTOCOL_TCP);
        }
        // AP Network Creation Failed
        else
        {
            if(DEBUG_TO_SERIAL)
                debugPrintln(PSTR("AP Network Creation FAILED!"));
            
            // Blink light to show ap Network Created failure
            for(int i = 0; i < 3; i ++)
            {
                digitalWrite(statusPin, HIGH);
                delay(100);
                digitalWrite(statusPin, LOW);
                delay(100);
            }
        }*/
    }
    
    // Printer powerup and setup
    printer.begin();
    
    printFreeRamToDebugAndWifly();
}

int freeRam() 
{
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void printFreeRamToDebug()
{
    if(DEBUG_TO_SERIAL)
    {
        debugPrint(PSTR("RAM:"));
        debug.println(freeRam());
    }
}

void printFreeRamToDebugAndWifly()
{
    int sram = freeRam();
    
    if(DEBUG_TO_SERIAL)
    {
        debugPrint(PSTR("RAM:"));
        debug.println(sram);
    }
    if(DEBUG_TO_WIFLY)
    {
        wiflyPrint(PSTR("RAM:"));
        wifly.println(sram);
    }
}

void loop()
{
    if(bufferLength < (BUFFER_SIZE - 1))
        getCommand();
    
    if(bufferLength)
    {
        processCommands();
        
        bufferLength = (bufferLength - 1);
        bufferReadIndex = (bufferReadIndex + 1) % BUFFER_SIZE;
    }
}

void processCommands()
{
    int codeNumber; //throw away variable
    char *codeCharacters;
    int vValue = 0;
    bool foundCommand = true;
    
    if(codeSeen('P'))
    {
        codeNumber = intCodeValue();
        if(DEBUG_TO_WIFLY)
        {
            wifly.println();
            wifly.write('P');
            wifly.println(codeNumber);
        }
        if(DEBUG_TO_SERIAL)
        {
            debug.println();
            debug.write('P');
            debug.println(codeNumber);
        }
        
        if(authorized || codeNumber == 0)
        {
            switch(codeNumber)
            {
                case 0: // P00 Password authentication
                    char *passwordAttempt;
                    if(codeSeen('V'))
                        passwordAttempt = stringCodeValue();
                    
                    if(DEBUG_TO_WIFLY)
                    {
                        wiflyPrint(PSTR("Password:"));
                        wifly.println(password);
                        wiflyPrint(PSTR("Password attempt:"));
                        wifly.println(passwordAttempt);
                    }
                    if(DEBUG_TO_SERIAL)
                    {
                        debugPrint(PSTR("Password:"));
                        debug.println(password);
                        debugPrint(PSTR("Password attempt:"));
                        debug.println(passwordAttempt);
                    }
                    
                    // Check the password
                    if(strcmp(password, passwordAttempt) == 0)
                    {
                        authorized = true;
                        wiflyPrintln(PSTR("Authorized"));
                    }
                    break;
                case 1: // P01 Print text
                    char *textToPrint;
                    if(codeSeen('V'))
                        textToPrint = stringCodeValue();
                    
                    if(DEBUG_TO_WIFLY)
                    {
                        wiflyPrint(PSTR("Printing text:"));
                        wifly.println(textToPrint);
                    }
                    if(DEBUG_TO_SERIAL)
                    {
                        debugPrint(PSTR("Printing text:"));
                        debug.println(textToPrint);
                    }
                    
                    printer.print(textToPrint);
                    break;
                case 2: // P02 Wakeup Printer
                    printer.wake();
                    break;
                case 3: // P03 Sleep Printer
                    printer.sleep();
                    break;
                case 4: // P04 Take Printer Online
                    printer.online();
                    break;
                case 5: // P05 Take Printer Offline
                    printer.offline();
                    break;
                case 6: // P06 VNewPassword - Change the password
                    if(codeSeen('V'))
                        password = stringCodeValue();
                    
                    writeEEPROMValues();
                    break;
                case 7: // P07 VNetworkName - Change the network name
                    char *newAPNetworkName;
                    if(codeSeen('V'))
                        newAPNetworkName = stringCodeValue();
                    
                    strcpy(apNetworkName, newAPNetworkName);
                    writeEEPROMValues();
                    break;
                case 8: // P08 VWidth SHeight - Start printing bitmap. With width and height. Expects the appropiate amount of hex values and then a P09.
                    if(codeSeen('V'))
                        bitmapWidth = intCodeValue();
                    if(codeSeen('S'))
                        bitmapHeight = intCodeValue();
                    
                    printingBitmap = true;
                    /*
                    // Maximum width of the printer
                    if (bitmapWidth > 384) 
                        break;
                    
                    printer.write(18);
                    printer.write(42);
                    printer.write((int)(bitmapWidth / 8.0 * bitmapHeight / 16.0));
                    printer.write((int)(bitmapWidth / 8));*/
                    break;
                case 9: // P09 VFF - Bitmap value
                    if(printingBitmap)
                    {
                        for(int i = 0; i < (int)(bitmapWidth / 8.0); i ++)
                        {
                            bitmapValue = 0;
                            if(codeSeen('V'))
                            {
                                hexStringCodeValue();
                                if(DEBUG_TO_SERIAL)
                                {
                                    debugPrint(PSTR("Hex:"));
                                    debug.println(hexString);
                                }
                                convertHexCharacters(hexString, 2);
                                bitmapValue |= (hexString[0] << 4);
                                bitmapValue |= (hexString[1] << 0);
                            }
                            
                            bitmapRow[bitmapRowIndex++] = bitmapValue;
//                            printer.write(bitmapValue);
                        }
                        
                        printer.printBitmapRowFromSRAM(bitmapWidth, bitmapRow);
                        bitmapRowIndex = 0;
                    }
                    break;
                case 10: // P10 - End printing bitmap
//                    delay(500);
                    printingBitmap = false;
                    break;
                case 11: // P11 - Print default bitmap
                    printer.printBitmap(TheBitmapWidth, TheBitmapHeight, TheBitmap);
                    break;
                case 12: // P12 - Open Cash Drawer
                    if(DEBUG_TO_SERIAL)
                    {
                        debugPrint(PSTR("Open Cash Drawer:"));
                        debug.println(password);
                    }
                    digitalWrite(cashDrawerPin, HIGH);
                    // Wait 50 ms to ensure the solenoid has been energized
                    delay(50);
                    digitalWrite(cashDrawerPin, LOW);
                    delay(50);
                    break;
                case 97: // P97 Restore printer settings to default
                    printer.setDefault();
                    break;
                case 98: // P98 Restore EEPROM defaults and reset
                    writeEEPROMDefaults();
                    printer.setDefault();
                    
                    printer.begin();
                    break;
                case 99: // P99 Reset Printer
                    printer.reset();
                    break;
                default:
                    foundCommand = false;
                    break;
            }
        }
        else
        {
            wiflyPrintln(PSTR("Not Authorized!"));
        }
    }
    else if(codeSeen('F'))
    {
        codeNumber = intCodeValue();
        if(DEBUG_TO_WIFLY)
        {
            wifly.println();
            wifly.write('F');
            wifly.println(codeNumber);
        }
        if(DEBUG_TO_SERIAL)
        {
            debug.println();
            debug.write('F');
            debug.println(codeNumber);
        }
        
        if(authorized)
        {
            switch(codeNumber) 
            {
                case 0: // F00 - All settings to default
                    printer.setDefault();
                    break;
                case 1: // F01 V0 - Inverse (Value codes: 0 = Off, 1 = On, F = Off, T = On)
                    if(codeSeen('V')) 
                        codeCharacters = stringCodeValue();
                    
                    if(stringIsTrueOrFalse(codeCharacters))
                        printer.inverseOn();
                    else
                        printer.inverseOff();
                    break;
                case 2: // F02 V0 - UpsideDown (Value codes: 0 = Off, 1 = On, F = Off, T = On)
                    if(codeSeen('V')) 
                        codeCharacters = stringCodeValue();
                    
                    if(stringIsTrueOrFalse(codeCharacters))
                        printer.upsideDownOn();
                    else
                        printer.upsideDownOff();
                    break;
                case 3: // F03 V0 - Double Height (Value codes: 0 = Off, 1 = On, F = Off, T = On)
                    if(codeSeen('V')) 
                        codeCharacters = stringCodeValue();
                    
                    if(stringIsTrueOrFalse(codeCharacters))
                        printer.doubleHeightOn();
                    else
                        printer.doubleHeightOff();
                    break;
                case 4: //  F04 V0 - Double Width (Value codes: 0 = Off, 1 = On, F = Off, T = On)
                    if(codeSeen('V')) 
                        codeCharacters = stringCodeValue();
                    
                    if(stringIsTrueOrFalse(codeCharacters))
                        printer.doubleWidthOn();
                    else
                        printer.doubleWidthOff();
                    break;
                case 5: // F05 V0 - Strike (Value codes: 0 = Off, 1 = On, F = Off, T = On)
                    if(codeSeen('V')) 
                        codeCharacters = stringCodeValue();
                    
                    if(stringIsTrueOrFalse(codeCharacters))
                        printer.strikeOn();
                    else
                        printer.strikeOff();
                    break;
                case 6: // F06 V0 - Set Bold (Value codes: 0 = Off, 1 = On, F = Off, T = On)
                    if(codeSeen('V')) 
                        codeCharacters = stringCodeValue();
                    
                    if(stringIsTrueOrFalse(codeCharacters))
                        printer.boldOn();
                    else
                        printer.boldOff();
                    break;
                case 7: // F07 VL - Justify (Value codes: L = Left, C = Center, R = Right)
                    if(codeSeen('V')) 
                        codeCharacters = stringCodeValue();
                    
                        printer.justify(codeCharacters[0]);
                    break;
                case 8: // F08 V5 - Line Feed (e.g. F16 V5 Feeds 5 lines)
                    if(codeSeen('V')) 
                        vValue = intCodeValue();
                    
                    printer.feed(vValue);
                    break;
                case 9: // F09 V3 - Pixel Feed (e.g. F17 V3 Feeds 3 rows of pixels)
                    if(codeSeen('V')) 
                        vValue = intCodeValue();
                    
                    printer.feedRows(vValue);
                    break;
                case 10: // F10 VS - Set printing size (Value codes: S = Small, M = Medium, L = Large)
                    if(codeSeen('V')) 
                        codeCharacters = stringCodeValue();
                    
                    debug.println(codeCharacters[0]);
                    printer.setSize(codeCharacters[0]);
                    break;
                case 11: // F11 V0 - Underline (Value codes: 0 = Off, 1 = Normal Underline, 2 = Thick Underline)
                    if(codeSeen('V')) 
                        codeNumber = intCodeValue();
                    
                    printer.underlineOn(codeNumber);
                    break;
                case 12: // F12 V0 - Set Character Spacing (Values Codes represent character spacing in pixels???)
                    if(codeSeen('V')) 
                        vValue = intCodeValue();
                    
                    printer.setCharSpacing(vValue);
                    break;
                case 13: // F13 V32 - Set Line Height (Values codes represent line height in pixels. Default is 32)
                    if(codeSeen('V')) 
                        vValue = intCodeValue();
                    
                    printer.setLineHeight(vValue);
                    break;
                case 14: // F14 V1 - Add Tabs (Values codes represent the number of tabs to add)
                    if(codeSeen('V')) 
                        vValue = intCodeValue();
                    
                    for(; vValue > 0; vValue --)
                        printer.tab();
                    break;
                default:
                    foundCommand = false;
                    break;
            }
        }
        else
        {
            wiflyPrintln(PSTR("Not Authorized!"));
        }
    }
    else
    {
        // Error
        wiflyPrintln(PSTR("Invalild Command Code!"));
        foundCommand = false;
    }
    
    if(foundCommand)
        sendCommandFinished();
    
    ClearToSend();
}

void getCommand() 
{ 
  // if wifly.availible then we did have a connection but it has now been closed
    while(wifly.available() > 0  && bufferLength < BUFFER_SIZE) 
    {
        serialCharacter = wifly.read();
        // If end of 1 command line
        if(serialCharacter == '\n' || serialCharacter == '\r' || serialCount >= (MAX_COMMAND_SIZE - 1)) 
        {
            // If empty line
            if(!serialCount) 
            { 
                commentMode = false; //for new command
                return;
            }
            commandBuffer[bufferWriteIndex][serialCount] = 0; //terminate string
            
            if(!commentMode)
            {
                commentMode = false; //for new command
                
                codeSeen('P');
                codeSeen('F');
                
                bufferWriteIndex = (bufferWriteIndex + 1) % BUFFER_SIZE;
                bufferLength += 1;
            }
            serialCount = 0; //clear buffer
        }
        else
        {
            if(serialCharacter == ';') 
                commentMode = true;
            if(!commentMode) 
                commandBuffer[bufferWriteIndex][serialCount++] = serialCharacter;
        }
    }
}

void hexStringCodeValue()
{
    // Copy the characters
    for(int i = 0; i < 2; i ++)
    {
        hexString[i] = commandBuffer[bufferReadIndex][stringCharacterPointer - commandBuffer[bufferReadIndex] + 1 + i];
    }
    // Erase the 'V' so the next one can be found
    commandBuffer[bufferReadIndex][stringCharacterPointer - commandBuffer[bufferReadIndex]] = '0';
}

char * stringCodeValue()
{
    return &commandBuffer[bufferReadIndex][stringCharacterPointer - commandBuffer[bufferReadIndex] + 1];
}

float floatCodeValue() 
{ 
    return (strtod(&commandBuffer[bufferReadIndex][stringCharacterPointer - commandBuffer[bufferReadIndex] + 1], NULL)); 
}

int intCodeValue() 
{ 
    return (int)(strtol(&commandBuffer[bufferReadIndex][stringCharacterPointer - commandBuffer[bufferReadIndex] + 1], NULL, 10)); 
}

// Return True if the string was found
bool codeSeen(char codeString[])
{ 
    return (strstr(commandBuffer[bufferReadIndex], codeString) != NULL); 
}  

// Return True if the char was found
bool codeSeen(char code)
{
    stringCharacterPointer = strchr(commandBuffer[bufferReadIndex], code);
    return (stringCharacterPointer != NULL);  //Return True if a character was found
}

void sendCommandFinished()
{
    wifly.write(COMMAND_FINISHED_CHARACTER);
    debug.println(COMMAND_FINISHED_CHARACTER);
}

void FlushSerialRequestResend()
{
    //char cmdbuffer[bufindr][100]="Resend:";
    wifly.flush();
    ClearToSend();
}

void ClearToSend()
{
    previousCommandMilliseconds = millis();
    //SERIAL_PROTOCOLLNPGM(MSG_OK); 
}

void terminal()
{
    if(wifly.available())
    {
        char c = wifly.read();
        debug.write(c);
        
        // Loop back
        wifly.write(c);
    }
    if(debug.available())
    {
        wifly.write(debug.read());
    }
}

bool stringIsTrueOrFalse(char *string)
{
    if(string[0] == '0' || string[0] == 'F' || string[0] == 'f')
        return false;
    else if(string[0] == '1' || string[0] == 'T' || string[0] == 't')
        return true;
}

void endConnectionAndReset()
{
    wifly.close();
    authorized = false;
}

void readEEPROMValues()
{
    EEPROMAddress = EEPROM_ADDRESS;
    EEPROM_readAnything(&EEPROMAddress, versionNumber);
    
    if(DEBUG_TO_SERIAL)
    {
        debugPrint(PSTR("EEPROM versionNumber:"));
        debug.println(versionNumber);
    }
    
    // First powerup. Write defaults to EEPROM
    if(versionNumber == -1 || versionNumber > VERSION_NUMBER)
    {
        writeEEPROMDefaults();
    }
    
    // Version 1.0 Values/Order
    if(versionNumber <= 1)
    {
        if(DEBUG_TO_SERIAL)
            debugPrintln(PSTR("Proper Version"));
        
        EEPROM_readAnything(&EEPROMAddress, password);
        EEPROM_readAnything(&EEPROMAddress, apNetworkName);
        EEPROM_readAnything(&EEPROMAddress, joinNetorkName);
        EEPROM_readAnything(&EEPROMAddress, joinNetworkPassword);
        
        if(DEBUG_TO_SERIAL)
        {
            debugPrint(PSTR("EEPROM Password:"));
            debug.println(password);
            debugPrint(PSTR("EEPROM apNetworkName:"));
            debug.println(apNetworkName);
            debugPrint(PSTR("EEPROM joinNetworkName:"));
            debug.println(joinNetorkName);
            debugPrint(PSTR("EEPROM joinNetworkPassword:"));
            debug.println(joinNetworkPassword);
        }
    }
}
 
void writeEEPROMDefaults()
{
    if(DEBUG_TO_SERIAL)
        debugPrintln(PSTR("Defaults"));
    // Defaults
    versionNumber = VERSION_NUMBER;
    password = DEFAULT_PASSWORD;
    apNetworkName = DEFAULT_AP_NETWORK_NAME;
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
        EEPROM_writeAnything(&EEPROMAddress, apNetworkName);
        EEPROM_writeAnything(&EEPROMAddress, joinNetorkName);
        EEPROM_writeAnything(&EEPROMAddress, joinNetworkPassword);
    }
}

void debugPrint(PGM_P s) 
{
    char c;
    while((c = pgm_read_byte(s++)) != 0)
        debug.print(c);
}

void debugPrintln(PGM_P s) 
{
    char c;
    while((c = pgm_read_byte(s++)) != 0)
        debug.print(c);
    
    debug.println();
}

void wiflyPrint(PGM_P s) 
{
    char c;
    while((c = pgm_read_byte(s++)) != 0)
        wifly.print(c);
}

void wiflyPrintln(PGM_P s) 
{
    char c;
    while((c = pgm_read_byte(s++)) != 0)
        wifly.print(c);
    
    wifly.println();
}

void convertHexCharacters(char *commandValues, int length)
{
	short i;
	// Only convert the hex characters to 0 - 15 from their ASCII Character values
	for(i = 0; i < length; i ++)
	{
		// Convert upper case letters
		if(commandValues[i] >= 'A' && commandValues[i] <= 'F')
			commandValues[i] -= 55;
		// Convert lower case letters
		else if(commandValues[i] >= 'a' && commandValues[i] <= 'f')
			commandValues[i] -= 87;
		// Convert numbers
		else if(commandValues[i] >= '0' && commandValues[i] <= '9')
			commandValues[i] -= 48;
		//printf("cvrt %c", commandValues[i]);
	}
}
