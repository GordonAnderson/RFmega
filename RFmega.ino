#include <Arduino.h>
#include <variant.h>
#include <wiring_private.h>
#include "SERCOM.h"
#include <Thread.h>
#include <ThreadController.h>
#include <Adafruit_DotStar.h>
#include <arduino-timer.h>

#include <Wire.h>
#include <SPI.h>
#include "Hardware.h"
#include "RFmega.h"
#include "Errors.h"
#include "Serial.h"
#include "Button.h"
#include "ethernet.h"
#include <Adafruit_SPIFlash.h>
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>
#include <SerialBuffer.h>

//
// Gordon Anderson
//
// The RFmega hardware uses an Adafruit ItsyBitsy M4 processor.
//
// To do:
//  1.) Add LED control:
//      - RED = RF on
//      - RED = status
//              flash when tuning
//              fast flash when limit exceeded
//  2.) Add push button control
//      - Push once short, RF on or off
//      - Push and hold = tune
//  3.) Add external serial port interface (done)
//  4.) RF level lookup table (done)
//  5.) Add gate in support, RF on/off
//  6.) Update commands to match MIPS (no need)
//  7.) RF dive is inverted (fixed)
//  8.) Add name (done)
//  9.) Add power and drive limits (done)
//  10.) Correct the current calculation (done)
//  11.) Add enable logic (done)
//
//  Version history:
//    Version 1.0, April 25, 2022
//      - Orginal version
//    Version 1.1, May 5, 2022
//      - Lowered low freq limit to 400KHz
//    Version 1.3, Oct 3, 2022
//      - Updated the enet interface, initializtion issues would not allow it to start
//      - Added test of the 24 volt supply on init and in run loop. If 24 V is not valid then the 
//        system will halt
//    Version 1.4, Feb 3, 2023
//      - Fixed a startup issue that did not enable the serial port when the eternet interface 
//        was not found.
//    Version 1.5, Oct 28, 2023
//      - Added current sensor calibration function
//    Version 1.6, Feb 12, 2024
//      - Improved auto tune to make sure it searches full range 
//      - Changed filter from 0.1 to 0.025 to stabalize readbacks
//    Version 1.7, Sept 3, 2024
//      - Added the file system using circuit python setup to enable the arduino drivers
//        that support file IO. Added save and save calibration functions.
//   

// Power on ADC threshold, A4 input
#define PWR_THRESHOLD 1000

// PLL is not stable below 250,000Hz
#define MinFreq    400000
#define MaxFreq    5000000

const char   Version[] PROGMEM = "MIPS RF Mega Version 1.7, Sept 3, 2024";
RFmegaData   rfmegadata;

RFmegaData Rev_1_rfmegadata = {
                            sizeof(RFmegaData),"RFmega",2,
                            false,
                            0x58,
                            1728000,0,500,false,false,
                            0.005,
                            60,50,
                            1,.6184,-156.32,
                            2,.6184,-156.32,
                            0,118.29,0,
                            3,138.5,430,
                            true,
                            9,164,206,334,403,452,494,542,585,630,894,4,23,57,98,138,184,224,266,306,894,
                            9,164,206,334,403,452,494,542,585,630,894,4,23,57,98,138,184,224,266,306,894,
                            false,
                            SIGNATURE
                          };

// System variables
State state;
float RFvoltageP = 0;
float RFvoltageN = 0;
float DriveVolt = 0;
float DriveCurrent = 0;
float Power = 0;

auto timer = timer_create_default();

// Auto tune parameters
bool TuneRequest   = false;
bool RetuneRequest = false;
bool Tuning        = false;
bool TuneReport    = false;
// Tune states
#define TUNE_SCAN_DOWN 1
#define TUNE_SCAN_UP 2

#define MaxNumDown 5
#define MAXSTEP    100000

Button functionPin;

// Reserve a portion of flash memory to store configuration
// Note: the area of flash memory reserved is cleared every time
// the sketch is uploaded on the board.
FlashStorage(flash_rfmegadata, RFmegaData);

// ThreadController that will control all threads
ThreadController control = ThreadController();
//Threads
Thread SystemThread = Thread();

// for flashTransport definition
#include "flash_config.h"
Adafruit_SPIFlash flash(&flashTransport);
// file system object from SdFat
FatVolume fatfs;
File32 file;

void Control(void)
{
  if(rfmegadata.Mode)
  {
    // Here if in closed loop control mode.
    float error = rfmegadata.Setpoint - (RFvoltageP + RFvoltageN)/2;
    rfmegadata.Drive += error * rfmegadata.Gain;
    if(rfmegadata.Drive < 0) rfmegadata.Drive=0;
  }
  // Limit testing
  if(rfmegadata.Drive > rfmegadata.MaxDrive) rfmegadata.Drive = rfmegadata.MaxDrive;
  if(Power > rfmegadata.MaxPower) rfmegadata.Drive -= 0.1;
  if(rfmegadata.Drive < 0) rfmegadata.Drive = 0;
}

// This function is called at 40 Hz
void Update(void)
{
  float val;
  
// Update all changes
  if((state.Enable != rfmegadata.Enable) || (state.update))
  {
    state.Enable = rfmegadata.Enable;
    if(rfmegadata.Enable)
    {
      setFrequency(rfmegadata.Freq);
      delay(100);
      setDrive(rfmegadata.Drive);
    }
    else
    {
      setDrive(0);
      delay(100);
      FS7140setup(rfmegadata.TWIadd,rfmegadata.Freq,true);
    }
  }
  if((state.Freq != rfmegadata.Freq) || (state.update))
  {
    state.Freq = rfmegadata.Freq;
    if(rfmegadata.Enable) setFrequency(rfmegadata.Freq);
  }
  if((state.Drive != rfmegadata.Drive) || (state.update))
  {
    state.Drive = rfmegadata.Drive;
    if(rfmegadata.Enable) setDrive(rfmegadata.Drive);
  }
  if((state.Setpoint != rfmegadata.Setpoint) || (state.update))
  {
    state.Setpoint = rfmegadata.Setpoint;
  }
  if((state.Mode != rfmegadata.Mode) || (state.update))
  {
    state.Mode = rfmegadata.Mode;
  }
  if((state.Gate != rfmegadata.Gate) || (state.update))
  {
    state.Gate = rfmegadata.Gate;
  }
  state.update = false;
// Read the ADC values and convert
  if(!rfmegadata.UsePWL)
  {
    val = ReadADCchannel(rfmegadata.RFlevelP);
    RFvoltageP = (1.0 - FILTER) * RFvoltageP + FILTER * val;
    val = ReadADCchannel(rfmegadata.RFlevelN);
    RFvoltageN = (1.0 - FILTER) * RFvoltageN + FILTER * val;  
  }
  else
  {
    val = PWLlookup(0,GetADCvalue(rfmegadata.RFlevelP.Chan,20));
    RFvoltageP = (1.0 - FILTER) * RFvoltageP + FILTER * val;
    val = PWLlookup(1,GetADCvalue(rfmegadata.RFlevelN.Chan,20));
    RFvoltageN = (1.0 - FILTER) * RFvoltageN + FILTER * val;
  }
  val = ReadADCchannel(rfmegadata.DriveV);
  DriveVolt = (1.0 - FILTER) * DriveVolt + FILTER * val;
  val = ReadADCchannel(rfmegadata.DriveI);
  DriveCurrent = (1.0 - FILTER) * DriveCurrent + FILTER * val;
  if(DriveCurrent < 0) DriveCurrent = 0;
//  Power = DriveVolt * DriveCurrent;
  Power = 24 * DriveCurrent;
// Control functions
  RFdriver_tune();
  Control();
// Check limits and apply
  if(rfmegadata.Drive > rfmegadata.MaxDrive) rfmegadata.Drive = rfmegadata.MaxDrive;
}

bool RFonLED(void *)
{
  // If tuning then flash the LED
  // If RF on turn on the LED
  // If RF off tuen off the LED
  //timer.at(500, RFonLED);
  if(rfmegadata.Enable) digitalWrite(RFON,LOW);
  else digitalWrite(RFON,HIGH);
  return true;
}

bool StatusLED(void *)
{
  if(Tuning)
  {
    if(digitalRead(STATUS) == HIGH) digitalWrite(STATUS,LOW);
    else digitalWrite(STATUS,HIGH);
    return true;
  }
  digitalWrite(STATUS,HIGH);
  return true;
}

void setup() 
{
  delay(1000);
  // If this is rev 2.0 or later then make sure power is on before 
  // Starting up!
  if(Rev_1_rfmegadata.Rev >= 2)
  {
     analogReadResolution(12);
     analogWriteResolution(12);
     while(GetADCvalue(A4,20) < PWR_THRESHOLD) delay(500);
  }
  // Read the flash config contents and test the signature
  rfmegadata = flash_rfmegadata.read();
  if(rfmegadata.Signature != SIGNATURE) rfmegadata = Rev_1_rfmegadata;
  // Init serial communications
  SerialInit();
  Serial1.begin(115200); // external RS232 port
  // Init the TWI interface
  Wire.begin();
  // Init the driver timer
  setupD10pwm();
  // Init the DIO
  // This disables the  drive
  // This disables the RF
  analogReadResolution(12);
  analogWriteResolution(12);
  // Configure Threads
  SystemThread.setName((char *)"Update");
  SystemThread.onRun(Update);
  SystemThread.setInterval(25);
  // Add threads to the controller
  control.add(&SystemThread);
  // Set the frequency
  setFrequency(rfmegadata.Freq);
  // Set the drive
  setDrive(rfmegadata.Drive);
  state.update = true; // force full update.
  functionPin.begin(PB);
  
  pinMode(5,OUTPUT);
  digitalWrite(RFON,LOW);


  
  // Start connect status LED
  pinMode(PB,INPUT_PULLUP);
  pinMode(RFON,OUTPUT);
  digitalWrite(RFON,HIGH);
  pinMode(STATUS,OUTPUT);
  digitalWrite(STATUS,HIGH);
  timer.every(500, RFonLED);
  timer.every(500, StatusLED);
  rfmegadata.Enable=false;
  functionPin.released();
  delay(4000);
  Ethernet_init();
}

// This function process all the serial IO and commands
void ProcessSerial(bool scan)
{
  // Put serial received characters in the input ring buffer
  if (Serial.available() > 0)
  {
    serial = &Serial;
    PutCh(Serial.read());
  }
  if (Serial1.available() > 0)
  {
    serial = &Serial1;
    PutCh(Serial1.read());
  }
  if (!scan) return;
  // If there is a command in the input ring buffer, process it!
  if (RB_Commands(&RB) > 0) while (ProcessCommand() == 0); // Process until flag that there is nothing to do
}

// Auto tune algorithm, procedure is as follows:
// 1.) Set power tp 10%
// 2.) Set frequency to 1MHz
// 3.) Go down in frequency in 100KHz steps and record amplitude, stop when 5 steps in a row are all decreasing
// 4.) Go up in frequency from 1MHzin 100KHz steps and record amplitude, stop when 5 steps in a row are all decreasing
// 5.) Use the peak found in steps 3 and 4 and sweep in 10K steps using the same procedure in 3 and 4
// 6.) Use the peak found in step 5 and sweep in 1K steps using the same procedure in 3 and 4
// 7.) Done!
//
// Called from the main processing loop, this function does not block, uses a state machine to perform the logic
// States
//  TUNE_SCAN_DOWN
//  TUNE_SCAN_UP

void RFdriver_tune(void)
{
   static int    TuneStep = 100000, TuneState;
   static float  Max, Current, Last;
   static int    FreqMax, Freq;
   static int    NumDown,Nth;

   if(TuneRequest)
   {
     rfmegadata.Mode = false; // Make sure we are in open loop mode!
     // Set freq to 1MHz
     rfmegadata.Freq = 1000000;
     // Set drive to 10%
     rfmegadata.Enable = true;
     rfmegadata.Drive = 10;
     Tuning = true;
     TuneStep = MAXSTEP;
     Freq = 1000000;
     Last = Max = 0;
     NumDown = -MaxNumDown;
     TuneRequest = false;
     TuneState = TUNE_SCAN_DOWN;
     Nth = 20;
     TuneReport = true;
     if(TuneReport) serial->println("Tuning...");
     return;
   }
   if(RetuneRequest)
   {
     rfmegadata.Mode = false; // Make sure we are in open loop mode!
     // Set freq to current
     Freq = rfmegadata.Freq;
     Tuning = true;
     TuneStep = 1000;
     Last = Max = 0;
     NumDown = 0;
     RetuneRequest = false;
     TuneState = TUNE_SCAN_DOWN;
     Nth = 20;
     if(TuneReport) serial->println("Re-tuning...");
     return;
   }
   if(!Tuning) return;
   if(--Nth > 0) return;
   Nth = 20;
   // Here if the system is tuning
   Current = RFvoltageP + RFvoltageN;
   switch (TuneState)
   {
     case TUNE_SCAN_DOWN:
//        if(Current > Max)
//        {
//          Max = Current;
//          FreqMax = fragdata.Freq;
//        }
        if(Current <= (Last + 1)) NumDown++;
        else 
        {
          NumDown = 0;
          if(TuneStep == MAXSTEP) NumDown = -MaxNumDown;
        }
        rfmegadata.Freq -= TuneStep;
        if((NumDown >= MaxNumDown) || (rfmegadata.Freq < MinFreq))
        {
          TuneState = TUNE_SCAN_UP;
          rfmegadata.Freq = Freq;
          NumDown = 0;
          if(TuneStep == MAXSTEP) NumDown = -MaxNumDown;
        }
        if(Current > Max)  // Move this code here to force tune system to not use the limit value
        {
          Max = Current;
          FreqMax = rfmegadata.Freq;
        }
        if(Current <= (Last + 1)) NumDown++;
        break;
     case TUNE_SCAN_UP:
        if(Current > Max)
        {
          Max = Current;
          FreqMax = rfmegadata.Freq;
        }
        if(Current <= (Last +1)) NumDown++;
        else 
        {
          NumDown = 0;
          //if(TuneStep == MAXSTEP) NumDown = -MaxNumDown;
        }
        if(TuneStep == MAXSTEP) NumDown = -MaxNumDown;
        rfmegadata.Freq += TuneStep;
        if((NumDown >= MaxNumDown) || (rfmegadata.Freq > MaxFreq))
        {
          // Here we have found the peak for this step size, this
          // process repeats until step size is 1KHz
          Freq = FreqMax;
          if(Freq < MinFreq) Freq = MinFreq;
          if(Freq > MaxFreq) Freq = MaxFreq;
          rfmegadata.Freq = Freq;
          if(TuneStep == 1000)
          {
            // If here we are done!
            Tuning = false;
            if(TuneReport && !SerialMute)
            {
              serial->print("Auto tune complete, frequency = ");
              serial->println(Freq);
            }
            TuneReport = false;
            return;
          }
          else 
          {
            if(TuneStep == MAXSTEP) TuneStep = 10000;
            else TuneStep /= 10;
          }
          TuneState = TUNE_SCAN_DOWN;
          NumDown = 0;
        }
        break;
     default:
        break;
   }
   Last = Current;
}

void loop() 
{
  static uint32_t now = millis();
  static uint32_t downTime, timeHeld = 0;
  static bool isDown = false;

  if(Rev_1_rfmegadata.Rev >= 2)
  {
     if(GetADCvalue(A4,20) < PWR_THRESHOLD) Software_Reset();
  }
  ProcessSerial();
  control.run();
  timer.tick();
  if(now != millis())
  {
    now = millis();
    if(functionPin.down())
    {
      if(!isDown)
      {
        isDown = true;
        downTime = now;
      }
      // Record the length to time in mS the button is down.
      // Process when the button is released
      timeHeld = now - downTime;
    }
    else isDown = false;
    if(functionPin.released())
    {
      if(timeHeld >= 3000)
      {
        // Start a tune cycle
        TuneRequest = true;
      }
      else
      {
        // Cycle RF on/off state
        if(rfmegadata.Enable) rfmegadata.Enable = false;
        else rfmegadata.Enable = true;
      }
    }
  }
}

//
// Host command functions
//

void SaveSettings(void)
{
  rfmegadata.Signature = SIGNATURE;
  flash_rfmegadata.write(rfmegadata);
  SendACK;
}

void RestoreSettings(void)
{
  static RFmegaData fd;
  
  // Read the flash config contents and test the signature
  fd = flash_rfmegadata.read();
  if(fd.Signature == SIGNATURE) rfmegadata = fd;
  else
  {
    SetErrorCode(ERR_EEPROMWRITE);
    SendNAK;
    return;
  }
  SendACK;  
}

void Software_Reset(void)
{
  NVIC_SystemReset();  
}

void FormatFLASH(void)
{
  flash_rfmegadata.write(Rev_1_rfmegadata);  
  SendACK;
}

void ReadADC(int chan)
{
  SendACKonly;
  serial->println(GetADCvalue(chan, 20));
}

void Debug(int i)
{
}

//
// The following functions support the piece wise linear calibration model.
//

// These functions will generate the calibration table. The user will adjust the 
// drive to set a desired voltage and then enter the value. Up to ten points
// be definded. Enter an empty string to terminate data entry. It is assumed 
// that the channel has been tuned and is properly connected.
// channel is 1 through number of channels
// phase is RF+ or RF-
uint8_t PWLch;

void RFdriveAllowADJ(void)
{
  // Call the task system to allow all system processing
  control.run();
}

void genPWLcalTable(char *phase)
{
  String sToken;
  char   *res;
  int    brd,ph,i;
  float  Drive;
  
  sToken = phase;
  if((sToken != "RF+") && (sToken != "RF-")) BADARG;
  if(sToken == "RF+") ph = 0;
  else ph = 1;
  // Send the user instructions.
  serial->println("This function will generate a piecewise linear");
  serial->println("calibration table. Set the drive level to reach");
  serial->println("desired calibration points and then enter the");
  serial->println("measured value to the nearest volt. Press enter");
  serial->println("When finished. Voltage must be increasing!");
  // Loop to allow user to adjust drive and enter measured voltage
  rfmegadata.PWLcal[ph].num = 0;
  i=0;
  Drive = rfmegadata.Drive;
  while(true)
  {
     serial->print("\nPoint ");
     serial->println(i+1);
     res = UserInput((char *)"Set drive level, %: ", RFdriveAllowADJ);
     if( res == NULL) break;
     sToken = res;
     rfmegadata.Drive = sToken.toFloat();
     res = UserInput((char *)"\nEnter measured voltage: ", RFdriveAllowADJ);
     if( res == NULL) break;
     sToken = res;
     rfmegadata.PWLcal[ph].Value[i] = sToken.toInt();
     // Read the ADC raw counts
     if(ph == 0) rfmegadata.PWLcal[ph].ADCvalue[i] = GetADCvalue(rfmegadata.RFlevelP.Chan,20);
     else rfmegadata.PWLcal[ph].ADCvalue[i] = GetADCvalue(rfmegadata.RFlevelN.Chan,20);
     i++;
     rfmegadata.PWLcal[ph].num = i;
     if(i>=MAXPWL) break;
  }
  serial->println("");
  // Report the table
  serial->print("Number of table entries: ");
  serial->println(rfmegadata.PWLcal[ph].num);
  for(i=0;i<rfmegadata.PWLcal[ph].num;i++)
  {
    serial->print(rfmegadata.PWLcal[ph].Value[i]);
    serial->print(",");
    serial->println(rfmegadata.PWLcal[ph].ADCvalue[i]);
  }
  // Done!
  serial->println("\nData entry complete!");
  rfmegadata.Drive = Drive;
}

// This function will use the piecewise linear table to convert the 
// adcval to output voltage.
// ph = phase, 0 = RF+, 1 = RF-
float PWLlookup(int ph, int adcval)
{
  int            brd,i;
  PWLcalibration *pwl;
  
  pwl = &rfmegadata.PWLcal[ph];
  if(pwl->num < 2) return 0;
  for(i=0;i<pwl->num-1;i++)
  {
    if(adcval < pwl->ADCvalue[i]) break;
    if((adcval >= pwl->ADCvalue[i]) && (adcval <= pwl->ADCvalue[i+1])) break;
  }
  if(i == pwl->num-1) i--;
  // The points at i and i+1 will be used to calculate the output voltage
  // y = y1 + (x-x1) * (y2-y1)/(x2-x1)
  return (float)pwl->Value[i] + ((float)adcval - (float)pwl->ADCvalue[i]) * ((float)pwl->Value[i+1]-(float)pwl->Value[i])/((float)pwl->ADCvalue[i+1] -(float)pwl->ADCvalue[i]);
}

void calCurrent(void)
{
  serial->println("Calibrate current sensor.");
  // Set first drive level and ask for the current value
  rfmegadata.Drive  = UserInputFloat((char *)"Enter drive level 1 : ", RFdriveAllowADJ);
  float cur1 = UserInputFloat((char *)"Enter current, amps : ", RFdriveAllowADJ);
  int   adc1 = GetADCvalue(rfmegadata.DriveI.Chan,100);
  // Set second drive level and ask for the current value
  rfmegadata.Drive = UserInputFloat((char *)"Enter drive level 2 : ", RFdriveAllowADJ);
  float cur2 = UserInputFloat((char *)"Enter current, amps : ", RFdriveAllowADJ);
  int   adc2 = GetADCvalue(rfmegadata.DriveI.Chan,100);
  // Calculate the calibration parameters and apply.
  // counts = value * m + b
  // adc1 = cur1 * m + b
  // adc2 = cur2 * m + b
  // adc1 - adc2 = (cur1 - cur2) * m
  // m = (adc1 - adc2) / (cur1 - cur2)
  // b = adc2 - cur2 * m
  rfmegadata.Drive = 10;
  rfmegadata.DriveI.m = (float)(adc1 - adc2) / (cur1 - cur2);
  rfmegadata.DriveI.b = (float)adc2 - cur2 * rfmegadata.DriveI.m;
  serial->print("m = "); serial->println(rfmegadata.DriveI.m); 
  serial->print("b = "); serial->println(rfmegadata.DriveI.b); 
}

// Functions supporting Circuit Python file system (CPFS). This is used to save
// the configuration and calibration data to the SPI flash filesystem.
// Provides non volitial storage of setup data.
bool FSsetup(void)
{
  // Init external flash
  if (!flash.begin()) serial->println("Error, failed to initialize flash chip!");
  else
  {
    serial->println("Flash chip initalized!");
    if(!fatfs.begin(&flash)) serial->println("Failed to mount filesystem!");
    else
    {
      serial->println("Mounted filesystem!");
      return true;
    }
  }
  return false;
}
void saveDefaults(void)
{
  if(!FSsetup()) return;
  if((file = fatfs.open("default.dat",O_WRITE | O_CREAT))==0) serial->println("Can't create default.dat!");
  else
  {
    size_t num = file.write((void *)&rfmegadata,sizeof(RFmegaData));
    file.close();
    serial->print("default.dat written, number of bytes = ");
    serial->println(num);
  }
}
void loadDefaults(void)
{
  RFmegaData h;

  if(!FSsetup()) return;
  if((file = fatfs.open("default.dat",O_READ))==0) serial->println("Can't open default.dat!");
  else
  {
    size_t num = file.read((void *)&h,sizeof(RFmegaData));
    file.close();
    if((num != sizeof(RFmegaData)) || (h.Signature != SIGNATURE))
    {
      serial->println("Error reading default.dat file!");
      return;
    }
    rfmegadata = h;
    serial->print("default.dat read, number of bytes = ");
    serial->println(num);
  }
}
void saveCalibrations(void)
{
  if(!FSsetup()) return;
  if((file = fatfs.open("cal.dat",O_WRITE | O_CREAT))==0) serial->println("Can't create cal.dat!");
  else
  {
    size_t num = file.write((void *)&rfmegadata.RFlevelP,sizeof(ADCchan));
    num += file.write((void *)&rfmegadata.RFlevelN,sizeof(ADCchan));
    num += file.write((void *)&rfmegadata.DriveV,sizeof(ADCchan));
    num += file.write((void *)&rfmegadata.DriveI,sizeof(ADCchan));
    num += file.write((void *)rfmegadata.PWLcal,sizeof(PWLcalibration) * 2);
    file.close();
    serial->print("cal.dat written, number of bytes = ");
    serial->println(num);
  }
}
void loadCalibrations(void)
{
  if(!FSsetup()) return;
  if((file = fatfs.open("cal.dat",O_READ))==0) serial->println("Can't open cal.dat!");
  else
  {
    size_t num = file.read((void *)&rfmegadata.RFlevelP,sizeof(ADCchan));
    num += file.read((void *)&rfmegadata.RFlevelN,sizeof(ADCchan));
    num += file.read((void *)&rfmegadata.DriveV,sizeof(ADCchan));
    num += file.read((void *)&rfmegadata.DriveI,sizeof(ADCchan));
    num += file.read((void *)rfmegadata.PWLcal,sizeof(PWLcalibration) * 2);
    file.close();
    serial->print("cal.dat read, number of bytes = ");
    serial->println(num);
  }
}
// End of CPFS

void bootloader(void)
{
  __disable_irq();
 	//THESE MUST MATCH THE BOOTLOADER
	#define DOUBLE_TAP_MAGIC 			      0xf01669efUL
	#define BOOT_DOUBLE_TAP_ADDRESS     (HSRAM_ADDR + HSRAM_SIZE - 4)

	unsigned long *a = (unsigned long *)BOOT_DOUBLE_TAP_ADDRESS;
	*a = DOUBLE_TAP_MAGIC;
	//NVMCTRL->ADDR.reg  = APP_START;
	//NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMD_EB | NVMCTRL_CTRLB_CMDEX_KEY;
	
	// Reset the device
	NVIC_SystemReset() ;

	while (true);
}
