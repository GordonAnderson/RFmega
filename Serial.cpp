/*
 * serial.c
 *
 * This file contains the code for the serial IO hardware including the ring buffer
 * code and the serial command processor.
 *
 * Developed by: Gordon Anderson
 */
#include "Arduino.h"
#include "string.h"
#include "Serial.h"
#include "Errors.h"
#include <FlashStorage.h>
#include <Wire.h>
#include <SPI.h>
#include "reset.h"

extern ThreadController control;

Stream *serial = &Serial;
bool SerialMute = false;

#define MaxToken 20

char Token[MaxToken];
char Sarg1[MaxToken];
char Sarg2[MaxToken];
unsigned char Tptr;

int ErrorCode = 0;   // Last communication error that was logged

Ring_Buffer  RB;     // Receive ring buffer

// ACK only string, two options. Need a comma when in the echo mode
char *ACKonlyString1 = "\x06";
char *ACKonlyString2 = ",\x06";
char *SelectedACKonlyString = ACKonlyString1;

bool echoMode = false;

Commands  CmdArray[] =   {
// General commands
  {"GVER",  CMDstr, 0, (char *)Version},                                  // Report version
  {"GERR",  CMDint, 0, (char *)&ErrorCode},                               // Report the last error code
  {"GNAME", CMDstr, 0, (char *)rfmegadata.Name},                          // Report system name
  {"SNAME", CMDstr, 1, (char *)rfmegadata.Name},                          // Set system name
  {"MUTE",  CMDfunctionStr, 1, (char *)Mute},                             // Turns on and off the serial response from the MIPS system
  {"ECHO",  CMDbool, 1, (char *)&echoMode},                               // Turns on and off the serial echo mode where the command is echoed to host, TRUE or FALSE
  {"DELAY", CMDfunction, 1, (char *)DelayCommand},                        // Generates a delay in milliseconds. This is used by the macro functions
                                                                          // to define delays in voltage ramp up etc.
  {"GCMDS",  CMDfunction, 0, (char *)GetCommands},                        // Send a list of all commands
  {"RESET",  CMDfunction, 0, (char *)Software_Reset},                     // System reboot
  {"SAVE",   CMDfunction, 0, (char *)SaveSettings},                       // Save settings
  {"RESTORE", CMDfunction, 0, (char *)RestoreSettings},                   // Restore settings
  {"FORMAT", CMDfunction, 0, (char *)FormatFLASH},                        // Format FLASH
  {"DEBUG", CMDfunction, 1, (char *)Debug},                               // Debug function, its function varies
  {"THREADS", CMDfunction, 0, (char *)ListThreads},                       // List all threads, there IDs, and there last runtimes
  {"STHRDENA", CMDfunctionStr, 2, (char *)SetThreadEnable},               // Set thread enable to true or false
  {"GOTO", CMDfunctionStr, 1, (char *)ProgramGOTO},                       // Go to a user defined hex address
  {"WHERE", CMDfunctionStr, 0, (char *)WhereAmI},                         // Reports the current function location in HEX
  {"ERASEU", CMDfunctionStr, 0, (char *)EraseUpper},                      // Erases the upper half of flash
  {"BLOAD", CMDfunction, 0, (char *)bootloader},                           // Jump to bootloader
 // Ethernet commands
  {"ENTEST", CMDfunction, 0, (char *)Ethernet_test},                 // Tests the eternet adapter connection
  {"GEIP", CMDfunction, 0, (char *)ReportEIP},                       // Report the ethernet adapter IP address
  {"SEIP", CMDfunctionStr, 1, (char *)SetEIP},                       // Set the ethernet adapter IP address
  {"GESNIP", CMDfunction, 0, (char *)ReportSNEIP},                   // Report the ethernet adapter Subnet IP address
  {"SESNIP", CMDfunctionStr, 1, (char *)SetSNEIP},                   // Set the ethernet adapter Subnet IP address
  {"GEPORT", CMDfunction, 0, (char *)ReportEport},                   // Report the ethernet adapter port number
  {"SEPORT", CMDfunction, 1, (char *)SetEport},                      // Set the ethernet adapter port number
  {"GEGATE", CMDfunction, 0, (char *)ReportEGATE},                   // Report the ethernet adapter gateway IP address
  {"SEGATE", CMDfunctionStr, 1, (char *)SetEGATE},                   // Set the ethernet adapter gateway IP address
// SPI flash file IO function
  {"SAVEF", CMDfunction, 0, (char *)saveDefaults},                         // Save current setting to flash FS
  {"LOADF", CMDfunction, 0, (char *)loadDefaults},                         // Load current setting from flash FS
  {"SAVECAL", CMDfunction, 0, (char *)saveCalibrations},                   // Save current calibration data to flash FS
  {"LOADCAL", CMDfunction, 0, (char *)saveCalibrations},                   // Load current calibration from flash FS
// RF mega commands
  {"TUNE",  CMDbool, 1, (char *)&TuneRequest},                            // Auto tune
  {"SENA",  CMDbool, 1, (char *)&rfmegadata.Enable},                      // RF head enable command, TRUE to enable
  {"GENA",  CMDbool, 0, (char *)&rfmegadata.Enable},                      // Returns the enable status
  {"RADC",  CMDfunction, 1, (char *)ReadADC},                             // Read the selected ADC channel
  {"SFREQ", CMDint, 1, (char *)&rfmegadata.Freq},                         // Sets frequency 
  {"GFREQ", CMDint, 0, (char *)&rfmegadata.Freq},                         // Returns frequency 
  {"SDRV", CMDfloat, 1, (char *)&rfmegadata.Drive},                       // Sets drive 
  {"GDRV", CMDfloat, 0, (char *)&rfmegadata.Drive},                       // Returns drive 
  {"SRFV", CMDfloat, 1, (char *)&rfmegadata.Setpoint},                    // Sets RF voltage setpoint 
  {"GRFV", CMDfloat, 0, (char *)&rfmegadata.Setpoint},                    // Returns RF voltage setpoint
  {"GRFAVP", CMDfloat, 0, (char *)&RFvoltageP},                           // Returns RF + voltage actual value
  {"GRFAVN", CMDfloat, 0, (char *)&RFvoltageN},                           // Returns RF - voltage actual value
  {"GDRVV", CMDfloat, 0, (char *)&DriveVolt},                             // Returns Driver voltage level
  {"GDRVI", CMDfloat, 0, (char *)&DriveCurrent},                          // Returns Driver current, in amps
  {"GPWR", CMDfloat, 0, (char *)&Power},                                  // Returns Driver power in watts
  {"SGAIN", CMDfloat, 1, (char *)&rfmegadata.Gain},                       // Sets closed loop gain
  {"GGAIN", CMDfloat, 0, (char *)&rfmegadata.Gain},                       // Returns closed loop gain
  {"SMODE", CMDbool, 1, (char *)&rfmegadata.Mode},                        // Sets mode, TRUE = closed loop mode, FALSE = open loop
  {"GMODE", CMDbool, 0, (char *)&rfmegadata.Mode},                        // Returns mode
  {"SMDRV", CMDfloat, 1, (char *)&rfmegadata.MaxDrive},                   // Sets max drive level
  {"GMDRV", CMDfloat, 0, (char *)&rfmegadata.MaxDrive},                   // Returns max drive level
  {"SMPWR", CMDfloat, 1, (char *)&rfmegadata.MaxPower},                   // Sets max power level
  {"GMPWR", CMDfloat, 0, (char *)&rfmegadata.MaxPower},                   // Returns max power level

  {"CALRFL", CMDfunctionStr, 1, (char *)genPWLcalTable},                  // Calibrate RF level table
  {"CALCUR", CMDfunction, 0, (char *)calCurrent},                         // Calibrate the current sensor
// End of table marker
  {0},
};


void WhereAmI(void)
{
   uint32_t addr = (uint32_t)&WhereAmI;
   serial->println(addr,16);
}

void EraseUpper(void)
{
  uint32_t addr = (uint32_t)&EraseUpper;
  if(addr < 0x20000)
  {
    FlashClass fc((void *)0x20000,8);
    fc.erase((void *)0x20000,8);
    return;
  }
  Serial.end();
  //this should reset the hardware USB
  USB->DEVICE.CTRLA.bit.SWRST = 1;
  while (USB->DEVICE.CTRLA.bit.SWRST == 1);
  // Stop the SPI and TWI interface
  SPI.end();
  Wire.end();
  // Stop all timers
  TCC0->CTRLA.bit.SWRST = 1;
  TCC1->CTRLA.bit.SWRST = 1;
  TCC2->CTRLA.bit.SWRST = 1;
  TC3->COUNT16.CTRLA.reg = 1;
//  TC4->COUNT16.CTRLA.reg = 1;
//  TC5->COUNT16.CTRLA.reg = 1;
  // Stop the system tick
  SysTick->LOAD = 0;
  SysTick->VAL = 0;
  SysTick->CTRL = 0;
  NVIC_DisableIRQ(SysTick_IRQn);
  NVIC_ClearPendingIRQ(SysTick_IRQn);

  FlashClass fc((void *)0x20000,8);
  fc.erase((void *)0x20000,8);
 
  
  /* Jump to application Reset Handler in the application */
  volatile uint32_t* Start_Address;
  Start_Address = (uint32_t *)0x2000;
  __set_MSP(Start_Address[0]);
  SCB->VTOR = (0x2000 & SCB_VTOR_TBLOFF_Msk);
  asm("bx %0"::"r"(Start_Address[1]));
}

// This function will jump to the application loaded at 0x20000 if the following:
// - port A pin 31 if tied low
// - This current application is not running in the 0x20000 bank
void LoadAltRev(void)
{
   // Set  PortA pin 31 as input with a pull up
   PORT->Group[PORTA].PINCFG[31].reg = (uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN) ;
   PORT->Group[PORTA].DIRCLR.reg     = (uint32_t)(1<<31) ;
   PORT->Group[PORTA].OUTSET.reg     = (uint32_t)(1<<31) ;
   // Read the input pin, eexit if high
   for(int i=0;i<100;i++) if((PORT->Group[PORTA].IN.reg & (1ul << 31)) != 0) return;
   // Determine the location of this function, exit if lower than 0x20000
   uint32_t addr = (uint32_t)&LoadAltRev;
   if(addr > 0x20000) return;
   // Try and vector to the new location
   ProgramGOTO("0x20000");
}

// This function will jump to the defined location if the location appears to be programmed.
void ProgramGOTO(char *location)
{
  uint32_t addr = strtol(location, NULL, 16);

  volatile uint32_t* Start_Address;
  Start_Address = (uint32_t *)addr;
  if(Start_Address[1] == 0xFFFFFFFF) return;
  Serial.end();
  //this should reset the hardware USB
  USB->DEVICE.CTRLA.bit.SWRST = 1;
  while (USB->DEVICE.CTRLA.bit.SWRST == 1);
  // Stop the SPI and TWI interface
  SPI.end();
  Wire.end();
  // Stop all timers
  TCC0->CTRLA.bit.SWRST = 1;
  TCC1->CTRLA.bit.SWRST = 1;
  TCC2->CTRLA.bit.SWRST = 1;
  TC3->COUNT16.CTRLA.reg = 1;
//  TC4->COUNT16.CTRLA.reg = 1;
//  TC5->COUNT16.CTRLA.reg = 1;
  // Stop the system tick
  SysTick->LOAD = 0;
  SysTick->VAL = 0;
  SysTick->CTRL = 0;
  NVIC_DisableIRQ(SysTick_IRQn);
  NVIC_ClearPendingIRQ(SysTick_IRQn);
  /* Jump to application Reset Handler in the application */
  __set_MSP(Start_Address[0]);
  SCB->VTOR = (addr & SCB_VTOR_TBLOFF_Msk);
  asm("bx %0"::"r"(Start_Address[1]));
}

// Sends a list of all commands
void GetCommands(void)
{
  int  i;

  SendACKonly;
  // Loop through the commands array and send all the command tokens
  for (i = 0; CmdArray[i].Cmd != 0; i++)
  {
    serial->println((char *)CmdArray[i].Cmd);
  }
}

// Delay command, delay is in millisecs
void DelayCommand(int dtime)
{
  delay(dtime);
  SendACK;
}

// Turns on and off responses from the MIPS system
void Mute(char *cmd)
{
  if (strcmp(cmd, "ON") == 0)
  {
    SerialMute = true;
    SendACK;
    return;
  }
  else if (strcmp(cmd, "OFF") == 0)
  {
    SerialMute = false;
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void SerialInit(void)
{
  Serial.begin(115200);
  RB_Init(&RB);
}

// This function builds a token string from the characters passed.
void Char2Token(char ch)
{
  Token[Tptr++] = ch;
  if (Tptr >= MaxToken) Tptr = MaxToken - 1;
}

// Get token from string object, delimited with comma
// Token are numbered 1 through n, returns empty string
// at end of tokens
String GetToken(String cmd, int TokenNum)
{
  int i, j, k;
  String Token;

  // If TokenNum is 0 or 1 then return the first token.
  // Return up to delimiter of the whole string.
  cmd.trim();
  if (TokenNum <= 1)
  {
    if ((i = cmd.indexOf(',')) == -1) return cmd;
    Token = cmd.substring(0, i);
    return Token;
  }
  // Find the requested token
  k = 0;
  for (i = 2; i <= TokenNum; i++)
  {
    if ((j = cmd.indexOf(',', k)) == -1) return "";
    k = j + 1;
  }
  Token = cmd.substring(k);
  Token.trim();
  if ((j = Token.indexOf(',')) == -1) return Token;
  Token = Token.substring(0, j);
  Token.trim();
  return Token;
}

// This function reads the serial input ring buffer and returns a pointer to a ascii token.
// Tokens are comma delimited. Commands strings end with a semicolon or a \n.
// The returned token pointer points to a standard C null terminated string.
// This function does not block. If there is nothing in the input buffer null is returned.
char *GetToken(bool ReturnComma)
{
  unsigned char ch;

  // Exit if the input buffer is empty
  while (1)
  {
    ch = RB_Next(&RB);
    if (ch == 0xFF) return NULL;
    if (Tptr >= MaxToken) Tptr = MaxToken - 1;
    if ((ch == '\n') || (ch == ';') || (ch == ':') || (ch == ',') || (ch == ']') || (ch == '['))
    {
      if (Tptr != 0) ch = 0;
      else
      {
        Char2Token(RB_Get(&RB));
        ch = 0;
      }
    }
    else RB_Get(&RB);
    // Place the character in the input buffer and advance pointer
    Char2Token(ch);
    if (ch == 0)
    {
      Tptr = 0;
      if ((Token[0] == ',') && !ReturnComma) return NULL;
      return Token;
    }
  }
}

char  *UserInput(char *message, void (*function)(void))
{
  char *tkn;
  
  // Flush the input ring buffer
  RB.Head=RB.Tail=RB.Count=RB.Commands=0;
  serial->print(message);
  // Wait for a line to be detected in the ring buffer
  while(RB.Commands == 0) 
  {
    ProcessSerial(false);
    if(function != NULL) function();
  }
  // Read the token
  tkn = GetToken(true);
  if(tkn != NULL) if(tkn[0] == '\n') tkn = NULL;
  return tkn;
}

int   UserInputInt(char *message, void (*function)(void))
{
  char   *tkn;
  String arg;

  tkn = UserInput(message,function);
  // Flush the input ring buffer
  RB.Head=RB.Tail=RB.Count=RB.Commands=0;
  arg = tkn;
  return arg.toInt();
}

float UserInputFloat(char *message, void (*function)(void))
{
  char   *tkn;
  String arg;

  tkn = UserInput(message,function);
  // Flush the input ring buffer
  RB.Head=RB.Tail=RB.Count=RB.Commands=0;
  arg = tkn;
  return arg.toFloat();
}

void ExecuteCommand(Commands *cmd, int arg1, int arg2, char *args1, char *args2, float farg1)
{
  if (echoMode) SelectedACKonlyString = ACKonlyString2;
  else SelectedACKonlyString = ACKonlyString1;
  switch (cmd->Type)
  {
    case CMDbool:
      if (cmd->NumArgs == 0)   // If true then write the value
      {
        SendACKonly;
        if (!SerialMute)
        {
          if (*(cmd->pointers.boolPtr)) serial->println("TRUE");
          else serial->println("FALSE");
        }
      }
      if (cmd->NumArgs == 1)  // If true then read the value
      {
        if ((strcmp(args1, "TRUE") == 0) || (strcmp(args1, "FALSE") == 0))
        {
          if (strcmp(args1, "TRUE") == 0) *(cmd->pointers.boolPtr) = true;
          else *(cmd->pointers.boolPtr) = false;
          SendACK;
          break;
        }
        SetErrorCode(ERR_BADARG);
        SendNAK;
      }
      break;
    case CMDstr:
      if (cmd->NumArgs == 0)   // If true then write the value
      {
        SendACKonly;
        if (!SerialMute) serial->println(cmd->pointers.charPtr);
        break;
      }
      if (cmd->NumArgs == 1)  // If true then read the value
      {
        strcpy(cmd->pointers.charPtr, args1);
        SendACK;
        break;
      }
      break;
    case CMDint:
      if (cmd->NumArgs == 0)   // If true then write the value
      {
        SendACKonly;
        if (!SerialMute) serial->println(*(cmd->pointers.intPtr));
        break;
      }
      if (cmd->NumArgs == 1)
      {
        *(cmd->pointers.intPtr) = arg1;
        SendACK;
        break;
      }
    case CMDfloat:
      if (cmd->NumArgs == 0)   // If true then write the value
      {
        SendACKonly;
        if (!SerialMute) serial->println(*(cmd->pointers.floatPtr));
      }
      if (cmd->NumArgs == 1)
      {
        *(cmd->pointers.floatPtr) = farg1;
        SendACK;
        break;
      }
      break;
    case CMDfunction:
      if (cmd->NumArgs == 0) cmd->pointers.funcVoid();
      if (cmd->NumArgs == 1) cmd->pointers.func1int(arg1);
      if (cmd->NumArgs == 2) cmd->pointers.func2int(arg1, arg2);
      break;
    case CMDfunctionStr:
      if (cmd->NumArgs == 0) cmd->pointers.funcVoid();
      if (cmd->NumArgs == 1) cmd->pointers.func1str(args1);
      if (cmd->NumArgs == 2) cmd->pointers.func2str(args1, args2);
      break;
    case CMDfun2int1flt:
      if (cmd->NumArgs == 3) cmd->pointers.func2int1flt(arg1, arg2, farg1);
      break;
    default:
      SendNAK;
      break;
  }
}

// This function processes serial commands.
// This function does not block and returns -1 if there was nothing to do.
int ProcessCommand(void)
{
  String sToken;
  char   *Token, ch;
  int    i;
  static int   arg1, arg2;
  static float farg1;
  static enum  PCstates state;
  static int   CmdNum;
  static char  delimiter = 0;
  // The following variables are used for the long string reading mode
  static char *lstrptr = NULL;
  static int  lstrindex;
  static bool lstrmode = false;
  static int lstrmax;

  // Wait for line in ringbuffer
  if (state == PCargLine)
  {
    if (RB.Commands <= 0) return -1;
    CmdArray[CmdNum].pointers.funcVoid();
    state = PCcmd;
    return 0;
  }
  if (lstrmode)
  {
    ch = RB_Get(&RB);
    if (ch == 0xFF) return (-1);
    if (ch == ',') return (0);
    if (ch == '\r') return (0);
    if (ch == '\n')
    {
      lstrptr[lstrindex++] = 0;
      lstrmode = false;
      return (0);
    }
    lstrptr[lstrindex++] = ch;
    return (0);
  }
  Token = GetToken(false);
  if (Token == NULL) return (-1);
  if (Token[0] == 0) return (-1);
  if ((echoMode) && (!SerialMute))
  {
    if (strcmp(Token, "\n") != 0)
    {
      if (delimiter != 0) serial->write(delimiter);
      serial->print(Token);
    }
    if (strcmp(Token, "\n") == 0) delimiter = 0;
    else delimiter = ',';
  }
  switch (state)
  {
    case PCcmd:
      if (strcmp(Token, ";") == 0) break;
      if (strcmp(Token, "\n") == 0) break;
      CmdNum = -1;
      // Look for command in command table
      for (i = 0; CmdArray[i].Cmd != 0; i++) if (strcmp(Token, CmdArray[i].Cmd) == 0)
        {
          CmdNum = i;
          break;
        }
      if (CmdNum == -1)
      {
        SetErrorCode(ERR_BADCMD);
        SendNAK;
        break;
      }
      // If the type CMDfunctionLine then we will wait for a full line in the ring buffer
      // before we call the function. Function has not args and must pull tokens from ring buffer.
      if (CmdArray[i].Type == CMDfunctionLine)
      {
        state = PCargLine;
        break;
      }
      // If this is a long string read command type then init the vaiable to support saving the
      // string directly to the provided pointer and exit. This function must not block
      if (CmdArray[i].Type == CMDlongStr)
      {
        lstrptr = CmdArray[i].pointers.charPtr;
        lstrindex = 0;
        lstrmax = CmdArray[i].NumArgs;
        lstrmode = true;
        break;
      }
      if (CmdArray[i].NumArgs > 0) state = PCarg1;
      else state = PCend;
      break;
    case PCarg1:
      Sarg1[0] = 0;
      sToken = Token;
      sToken.trim();
      arg1 = sToken.toInt();
      farg1 = sToken.toFloat();
      strcpy(Sarg1, sToken.c_str());
      if (CmdArray[CmdNum].NumArgs > 1) state = PCarg2;
      else state = PCend;
      break;
    case PCarg2:
      Sarg2[0] = 0;
      sToken = Token;
      sToken.trim();
      arg2 = sToken.toInt();
      strcpy(Sarg2, sToken.c_str());
      if (CmdArray[CmdNum].NumArgs > 2) state = PCarg3;
      else state = PCend;
      break;
    case PCarg3:
      sToken = Token;
      sToken.trim();
      farg1 = sToken.toFloat();
      state = PCend;
      break;
    case PCend:
      if ((strcmp(Token, "\n") != 0) && (strcmp(Token, ";") != 0))
      {
        state = PCcmd;
        SendNAK;
        break;
      }
      i = CmdNum;
      CmdNum = -1;
      state = PCcmd;
      ExecuteCommand(&CmdArray[i], arg1, arg2, Sarg1, Sarg2, farg1);
      break;
    default:
      state = PCcmd;
      break;
  }
  return (0);
}

void RB_Init(Ring_Buffer *rb)
{
  rb->Head = 0;
  rb->Tail = 0;
  rb->Count = 0;
  rb->Commands = 0;
}

int RB_Size(Ring_Buffer *rb)
{
  return (rb->Count);
}

int RB_Commands(Ring_Buffer *rb)
{
  return (rb->Commands);
}

// Put character in ring buffer, return 0xFF if buffer is full and can't take a character.
// Return 0 if character is processed.
char RB_Put(Ring_Buffer *rb, char ch)
{
  if (rb->Count >= RB_BUF_SIZE) return (0xFF);
  rb->Buffer[rb->Tail] = ch;
  if (rb->Tail++ >= RB_BUF_SIZE - 1) rb->Tail = 0;
  rb->Count++;
  if (ch == ';') rb->Commands++;
  if (ch == '\r') rb->Commands++;
  if (ch == '\n') rb->Commands++;
  return (0);
}

// Get character from ring buffer, return NULL if empty.
char RB_Get(Ring_Buffer *rb)
{
  char ch;

  if (rb->Count == 0)
  {
    rb->Commands = 0;  // This has to be true if the buffer is empty...
    return (0xFF);
  }
  ch = rb->Buffer[rb->Head];
  if (rb->Head++ >= RB_BUF_SIZE - 1) rb->Head = 0;
  rb->Count--;
  // Map \r to \n
  //  if(Recording) MacroFile.write(ch);
  if (ch == '\r') ch = '\n';
  if (ch == ';') rb->Commands--;
  if (ch == '\n') rb->Commands--;
  if (rb->Commands < 0) rb->Commands = 0;
  return (ch);
}

// Return the next character in the ring buffer but do not remove it, return NULL if empty.
char RB_Next(Ring_Buffer *rb)
{
  char ch;

  if (rb->Count == 0) return (0xFF);
  ch = rb->Buffer[rb->Head];
  if (ch == '\r') ch = '\n';
  return (ch);
}

void PutCh(char ch)
{
  RB_Put(&RB, ch);
}

// This function lists all the current threads and there current state.
void ListThreads(void)
{
  int    i = 0;
  Thread *t;

  // Loop through all the threads and report the Thread name, ID, Interval, enabled state, and last run time
  SendACKonly;
  serial->println("Thread name,ID,Interval,Enabled,Run time");
  while (1)
  {
    t = control.get(i++);
    if (t == NULL) break;
    serial->print(t->getName()); serial->print(", ");
    serial->print(t->getID()); serial->print(", ");
    serial->print(t->getInterval()); serial->print(", ");
    if (t->enabled) serial->print("Enabled,");
    else serial->print("Disabled,");
    serial->println(t->runTimeMs());
  }
}

void SetThreadEnable(char *name, char *state)
{
  Thread *t;

  if ((strcmp(state, "TRUE") !=0) && (strcmp(state, "FALSE") != 0))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  // Find thread by name
  t = control.get(name);
  if (t == NULL)
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  SendACKonly;
  if (strcmp(state, "TRUE") == 0) t->enabled = true;
  else t->enabled = false;
}
