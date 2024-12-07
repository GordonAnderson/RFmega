#ifndef RFmega_h
#define RFmega_h
#include "Hardware.h"

#define FILTER   0.025

#define SIGNATURE  0xAA55A5A5

extern float RFvoltageP;
extern float RFvoltageN;
extern float DriveVolt;
extern float DriveCurrent;
extern float Power;

extern bool TuneRequest;

#define  MAXPWL 10
// This data structure is used for the piece wise linear calibration
// function.
typedef struct
{
  uint8_t   num;
  uint16_t  ADCvalue[MAXPWL];
  uint16_t  Value[MAXPWL];
} PWLcalibration;

typedef struct
{
  int16_t       Size;                   // This data structures size in bytes
  char          Name[20];               // Holds the board name, "RFmega"
  int8_t        Rev;                    // Holds the board revision number
  bool          Enable;
  int           TWIadd;
  // Fragmentor settings
  int           Freq;                   // Fragmentor operating frequency
  float         Drive;
  float         Setpoint;               // RF voltage setpoint in volts
  bool          Mode;                   // Mode = true for closed loop, false for open
  bool          Gate;                   // True to enable gate mode
  float         Gain;                   // Closed loop control gain
  // Limits
  float         MaxDrive;
  float         MaxPower;
  // ADC channels
  ADCchan       RFlevelP;
  ADCchan       RFlevelN;
  ADCchan       DriveV;
  ADCchan       DriveI;
  // Piece wise linear calibration data structures
  bool          UsePWL;                 // True to use PWL table
  PWLcalibration PWLcal[2];             // Piece wise linear data structures, 0 for RF+ and 1 of RF-
  //
  bool          EnetEnabled;
  unsigned int  Signature;              // Must be 0xAA55A5A5 for valid data
} RFmegaData;

typedef struct
{
  bool          update;
  // RFmega settings
  bool          Enable;
  int           Freq;                   // RFmega operating frequency
  float         Drive;
  float         Setpoint;               // RF voltage setpoint in volts
  bool          Mode;                   // Mode = true for closed loop, false for open
  bool          Gate;                   // True to enable gate mode
} State;


// Prototypes...
void ProcessSerial(bool scan = true);
void SaveSettings(void);
void RestoreSettings(void);
void Software_Reset(void);
void FormatFLASH(void);
void Debug(int i);
void ReadADC(int chan);
void RFdriver_tune(void);

void bootloader(void);
void saveDefaults(void);
void loadDefaults(void);
void saveCalibrations(void);
void loadCalibrations(void);

void genPWLcalTable(char *phase);
void calCurrent(void);

#endif
