#ifndef Hardware_h
#define Hardware_h

// DIO lines
#define RFON      7
#define STATUS    9
#define PB        11
#define TRIG      2

// M4 pin assigenments

#define   MAXDRIVE 999

typedef struct
{
  int8_t  Chan;                   // ADC channel number 0 through max channels for chip.
                                  // If MSB is set then this is a M0 ADC channel number
  float   m;                      // Calibration parameters to convert channel to engineering units
  float   b;                      // ADCcounts = m * value + b, value = (ADCcounts - b) / m
} ADCchan;

typedef struct
{
  int8_t  Chan;                   // DAC channel number 0 through max channels for chip
  float   m;                      // Calibration parameters to convert engineering to DAC counts
  float   b;                      // DACcounts = m * value + b, value = (DACcounts - b) / m
} DACchan;

// Function prototypes
int   GetADCvalue(int chan, int num);
float Counts2Value(int Counts, DACchan *DC);
float Counts2Value(int Counts, ADCchan *ad);
int   Value2Counts(float Value, DACchan *DC);
int   Value2Counts(float Value, ADCchan *ac);
float ReadADCchannel(ADCchan adch);

void ProgramFLASH(char * Faddress,char *Fsize);

void setupD10pwm();
void setD10pwm(int value);
void setFrequency(int);
void setDrive(float percent);


#endif
