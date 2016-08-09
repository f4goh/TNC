/********************************************************************************************
 * VHF TNC DRA818 APRS Arduino library
 * Created 7/8/2016
 * Anthony LE CREN f4goh@orange.fr 
 * Modified
 * Use this library freely
 * AX25 RX routines from  Robert Marshall, KI4MCW 
 * Instance :
 *
 * Functions :
 *
 *******************************************************************************************/
 
 
#ifndef TNC_H
#define TNC_H
#include <Arduino.h>


//#include <SoftwareSerial.h>

class TNC
{
  public:
    TNC();
   
   
   
   void begin(int p_bf, int p_led, int f1,int f2, unsigned char *ptr);
   void sendpacket(unsigned char buffer[], unsigned char size_array);
   void gpsnmea(char byteGPS);
   void sinus();
	
   unsigned long computeDdsWord(double freq);
   
   double freq;
   int sync;
   int led;
   int bf;
   double refclk;
   byte flip_freq;
   
   
   
   volatile unsigned long ddsAccu;   // phase accumulator
   volatile unsigned long ddsWord;
   volatile unsigned long ddsWord0;  // dds tuning word 0
   volatile unsigned long ddsWord1;  // dds tuning word 1
   
	
   typedef struct  {
  char hour[6+1];
  char Latitude[9+1];
  char NS;
  char Longitude[10+1];
  char EO;
  byte fix;
  char sat[2+1];
  char altitude[7+1];
  byte secondes;     //secondes in byte from hour
  byte minutes;     //minutes in byte from hour
  unsigned int pperiod;      //second order
  byte sync;     //flag to send (matching for secondes%pperiod==0)
  int32_t time;
  boolean debug;
  boolean dumpNmea;
  byte mode;
  char neo;
  byte Ndelay;
  int nbSat;
  long altidudeMeters;
  long altidudeFeet;
  char feet[15];
    } GGAstruct;
  GGAstruct GPGGA;    //declare la structure
   
  volatile int sinusPtr;
  volatile int countPtr;
  volatile int shift;
  
    byte ptrStartNmea;
  
   void Decode(void);
  unsigned char digiTX;
unsigned char taille;

  
  private:
 
  int32_t parseDecimal(const char *term);
  int32_t timePrec;
  unsigned int timeElapsed;
  
  void send_bit(int tempo);
  unsigned char flip;

  
  byte validNmea(char byteGPS);
  void sendbyte (unsigned char inbyte);
  void fcsbit(unsigned short tbyte);
  void flipout(void);
  void clearGps(void);
  
  
  unsigned char stuff,flag,fcsflag;
 
  unsigned short crc;

  
  int sentence_status;      //0: recherche $, 1:recherche GPxxx, 2:GPGGA trouvé
  char sentenceType[5+1];     //GPxxx
  int ptr;                  //ptr for cahr arrays
  int comma_count;          //count , into sentences 
  
  
  // ==== vars RX routine
  
#define MIN_PACKET_LEN   10
#define PACKET_SIZE      200
#define AX25_MARK        0
#define AX25_SPACE       1
#define MAX_SYNC_ERRS    5
  
void decode_ax25(void) ;
 
  
signed char     adcval ;                   // zero-biased ADC input
                //last_phase_err;           
				
int16_t         mult_cb[7],                // circular buffer for adc*delay values
                mult_sum,                  // sum of mult_cb values
                bias_sum ;                 // sum of last 128 ADC readings
				
unsigned char   rawadc,                    // value read directly from ADCH register
		since_last_chg,            // samples since the last MARK/SPACE symbol change
                phase_err,                 // symbol transition timing error, in samples
                current_symbol,            // MARK or SPACE
                last_symbol,               // MARK or SPACE from previous reading
		last_symbol_inaframe,      // MARK or SPACE from one bit-duration ago
                inaframe,                  // rec'd start-of-frame marker
                bittimes,                  // bit durations that have elapsed
                bitqlen,                   // number of rec'd bits in bitq
                popbits,                   // number of bits to pop (drop) off the end
                byteval,                   // rec'd byte, read from bitq
                cb_pos,                    // position within the circular buffer
                msg[PACKET_SIZE + 1],      // rec'd data
                msg_pos,                   // bytes rec'd, next array element to fill
                x,                         // misc counter
                test,                      // temp variable
                decode_state,              // section of rec'd data being parsed
                bias_cnt,                  // number of ADC samples collected (toward goal of 128)
                adc_bias,                  // center-value for ADC, self-adjusting
                hb12 ;                     // heartbeat (1 or 0)

unsigned char   sync_err_cnt,              // number of sync errors in this frame (so far)
		bit_timer,                 // countdown one bit duration
		thesebits ;                // number of elapsed bit durations
				
signed char     adc_delay[6] ;             // delay line for adc readings
				
uint32_t        bitq ;                     // rec'd bits awaiting grouping into bytes
unsigned char *track;

  
};

extern TNC Beacon;

#endif
