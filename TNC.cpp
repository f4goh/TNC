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
 

#include <TNC.h>



TNC Beacon;


TNC::TNC(){
 sentence_status=0;      //0: recherche $, 1:recherche GPxxx, 2:GPGGA trouvé
 ptr=0;                  //ptr for char arrays
 comma_count=0;          //count , into sentences 
 GPGGA.nbSat=0;
 GPGGA.debug=false;
 GPGGA.dumpNmea=false;
 GPGGA.fix=0;
 sinusPtr=0;
 countPtr=0;
 timeElapsed=0;
 GPGGA.time=0;
 timePrec=0;
 ptrStartNmea=0;
 digiTX=0;
}

void TNC::begin(int p_bf, int p_led, int f1,int f2, unsigned char *ptr) {
 bf=p_bf;
 led=p_led;
 
 pinMode(led, OUTPUT);
 pinMode(bf, OUTPUT);
  
 analogWrite(bf, 0);      //dds off
 digitalWrite(led, LOW);
 refclk=13157.9;      // measured 
 //refclk=13160;      // measured 
 track=ptr;
 
 ddsWord0=TNC::computeDdsWord(f1);
 ddsWord1=TNC::computeDdsWord(f2);
 
}



/********************************************************
 * Send a bit into an FM modulation
 ********************************************************/
 
void TNC::send_bit(int tempo)
{
countPtr=0;
while(countPtr<tempo){
}
digitalWrite(led,digitalRead(led)^1);
}



void TNC::sinus()
{

const static byte sinusTable[512] PROGMEM = {128,129,131,132,134,135,137,138,140,141,143,145,146,148,149,151,152,154,155,157,158,160,161,163,164,166,167,169,170,172,173,175,176,178,179,180,182,183,185,186,
                                             187,189,190,191,193,194,195,197,198,199,201,202,203,204,206,207,208,209,210,212,213,214,215,216,217,218,219,221,222,223,224,225,226,227,228,229,230,230,231,232,
                                             233,234,235,236,236,237,238,239,240,240,241,242,242,243,244,244,245,245,246,247,247,248,248,249,249,249,250,250,251,251,251,252,252,252,253,253,253,253,254,254,
                                             254,254,254,254,254,254,254,254,255,254,254,254,254,254,254,254,254,254,254,253,253,253,253,252,252,252,251,251,251,250,250,249,249,249,248,248,247,247,246,245,
                                             245,244,244,243,242,242,241,240,240,239,238,237,236,236,235,234,233,232,231,230,230,229,228,227,226,225,224,223,222,221,219,218,217,216,215,214,213,212,210,209,
                                             208,207,206,204,203,202,201,199,198,197,195,194,193,191,190,189,187,186,185,183,182,180,179,178,176,175,173,172,170,169,167,166,164,163,161,160,158,157,155,154,
                                             152,151,149,148,146,145,143,141,140,138,137,135,134,132,131,129,127,126,124,123,121,120,118,117,115,114,112,110,109,107,106,104,103,101,100,98,97,95,94,92,91,89,
                                             88,86,85,83,82,80,79,77,76,75,73,72,70,69,68,66,65,64,62,61,60,58,57,56,54,53,52,51,49,48,47,46,45,43,42,41,40,39,38,37,36,34,33,32,31,30,29,28,27,26,25,25,24,23,
                                             22,21,20,19,19,18,17,16,15,15,14,13,13,12,11,11,10,10,9,8,8,7,7,6,6,6,5,5,4,4,4,3,3,3,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,3,3,3,4,4,4,5,5,6,
                                             6,6,7,7,8,8,9,10,10,11,11,12,13,13,14,15,15,16,17,18,19,19,20,21,22,23,24,25,25,26,27,28,29,30,31,32,33,34,36,37,38,39,40,41,42,43,45,46,47,48,49,51,52,53,54,56,57,
                                             58,60,61,62,64,65,66,68,69,70,72,73,75,76,77,79,80,82,83,85,86,88,89,91,92,94,95,97,98,100,101,103,104,106,107,109,110,112,114,115,117,118,120,121,123,124,126};
											 

 ddsAccu=ddsAccu+ddsWord; // soft DDS, phase accu with 32 bits
 sinusPtr=ddsAccu >> 23;
analogWrite(bf,pgm_read_byte(&(sinusTable[sinusPtr])));
countPtr++;
//digitalWrite(led,digitalRead(led)^1);
}



/********************************************************
 * AX25 TX routines
 ********************************************************/

void TNC::flipout(void)
 {		       
	stuff = 0;     //since this is a 0, reset the stuff counter
    flip_freq^=1;
	if (flip_freq==0) ddsWord=ddsWord1; else ddsWord=ddsWord0;
	
}

void TNC::fcsbit(unsigned short tbyte)
{
  crc ^= tbyte;
  if (crc & 1)
    crc = (crc >> 1) ^ 0x8408;  // X-modem CRC poly
  else
    crc = crc >> 1;
     
 }
 
 
void TNC::sendbyte (unsigned char inbyte)
{
   unsigned char k, bt;
 
   for (k=0;k<8;k++)
	{                                                         //do the following for each of the 8 bits in the byte
     bt = inbyte & 0x01;                                          //strip off the rightmost bit of the byte to be sent (inbyte)
     if ((fcsflag==0) & (flag==0)) (TNC::fcsbit(bt));                 //do FCS calc, but only if this is not a flag or fcs byte
     if (bt==0) (TNC::flipout());  			                // if this bit is a zero, flip the output state
        else {                          			//otherwise if it is a 1, do the following:
           stuff++;    				                //increment the count of consequtive 1's 
            if ((flag==0) & (stuff==5))
				{   	                        //stuff an extra 0, if 5 1's in a row
                                       		                 //flip the output state to stuff a 0
                               TNC::send_bit(11);
                                TNC::flipout();  
                        }                                      
        }
     inbyte = inbyte>>1;          		                //go to the next bit in the byte
TNC::send_bit(11);
  }//fin pour
}


unsigned long TNC::computeDdsWord(double freq)
{
return pow(2,32)*freq/refclk;
}



void TNC::sendpacket(unsigned char buffer[], unsigned char size_array)
{
	unsigned char i;
    crc=0xffff;
	stuff=0;
   
	shift=6;      //init
	flip_freq=1;
	ddsWord=ddsWord1;
	
    TNC::send_bit(11);
   
   flag = 1;             //The variable flag is true if you are transmitted flags (7E's) false otherwise.
   fcsflag = 0;       //The variable fcsflag is true if you are transmitting FCS bytes, false otherwise.

   for (i=0;i<200;i++) TNC::sendbyte(0x7E);	        //Sends 100 flag bytes.
   flag = 0;          			        //done sending flags
   for(i=0;i<size_array;i++) TNC::sendbyte(buffer[i]);       //send the packet bytes
   fcsflag = 1;       		//about to send the FCS bytes
   TNC::sendbyte((crc ^ 0xff));	// Send the CRC
   crc >>= 8;
   TNC::sendbyte((crc ^ 0xff));
   fcsflag = 0;		//done sending FCS
   flag = 1;  	//about to send flags
  for (i=0;i<100;i++) TNC::sendbyte(0x7E);	        //Sends 100 flag bytes.
 
  }
 


byte TNC::validNmea(char byteGPS)
{
  if (ptrStartNmea<5) {
	if (byteGPS=='$') ptrStartNmea++;
	Serial.write('S');
	return 0;
  }
  else return 1;
}  
  
  
  
 void TNC::clearGps(void)
 {
 	GPGGA.sync=1; 
	sentence_status=0;
	comma_count=0;
	GPGGA.nbSat=0;
	ptr=0;
	GPGGA.fix=0;
	timeElapsed=0;
	timePrec=GPGGA.time;
	ptrStartNmea=0;
 }
 
 
 
  
 void TNC::gpsnmea(char byteGPS)
{
 if( GPGGA.dumpNmea==true) Serial.print((char)byteGPS);
 
   if (validNmea(byteGPS)==0) return;
  
 switch (byteGPS)
{
  case '$' : if (sentence_status==0) {
                                      sentence_status=1;
                                      ptr=0;
                                      if (GPGGA.debug==true) Serial.print('$');
                                     }
              break;
  case ',' : if (sentence_status==1) {
                                         sentenceType[ptr++]=0;
                                         if (strcmp(sentenceType,"GPGGA")==0) 
                                                  {
                                                   if (GPGGA.debug==true) Serial.println(F("gga found"));
                                                   sentence_status=2;    //can extend sentence_status to 3,4 etc for another GPxxx sentences
                                                  }
                                                  else sentence_status=0;
                                      } 
              if (sentence_status==2) {
                                       switch (comma_count)
                                       {
                                      case 1 :   GPGGA.hour[ptr]=0;
                                                 if (GPGGA.fix>0) {
																	GPGGA.secondes=(GPGGA.hour[4]-'0')*10+GPGGA.hour[5]-'0';
																	GPGGA.minutes=(GPGGA.hour[2]-'0')*10+GPGGA.hour[3]-'0';
																	GPGGA.time=parseDecimal(GPGGA.hour);
																	timeElapsed=GPGGA.time-timePrec;
																	if (GPGGA.debug==true) {Serial.print(F("Hour :")); Serial.println(GPGGA.hour);}
																	//Serial.println(timeElapsed);
																	if (Beacon.GPGGA.mode==0){
																	 if ((timeElapsed/100)>=GPGGA.pperiod){
																			clearGps();
																	}
																	}
																	if (Beacon.GPGGA.mode==1){
																	  if (GPGGA.neo=='N') {
																	     if ((GPGGA.secondes>GPGGA.Ndelay) && (GPGGA.secondes<(GPGGA.Ndelay+5)))  {
																			clearGps();
																	      }
																		 }
																	  if (GPGGA.neo=='E') {
																	     if ((GPGGA.secondes>GPGGA.Ndelay) && (GPGGA.secondes<(GPGGA.Ndelay+5)) && ((GPGGA.minutes%2)==0)) {
																			clearGps();
																	      }
																		 }
																	  if (GPGGA.neo=='O') {
																	     if ((GPGGA.secondes>GPGGA.Ndelay) && (GPGGA.secondes<(GPGGA.Ndelay+5)) && ((GPGGA.minutes%2)==1)) {
																			clearGps();
																	      }
																		 }
																	}
																	//if ((GPGGA.secondes>0) && (GPGGA.secondes<3)) GPGGA.sync=1; else GPGGA.sync=0;
																	}
												break; 
                                      case 2 :   GPGGA.Latitude[ptr]=0;
												 break; 
                                      case 4 :   GPGGA.Longitude[ptr]=0;
												 break; 
                                      case 6 :  if (GPGGA.fix>0) digitalWrite(led, HIGH); else {
																									digitalWrite(led, digitalRead(led)^1);
																								    if (GPGGA.debug==true)		{
																																	Serial.print(F("Nb sat :"));
																																	Serial.println(GPGGA.sat);
																																	}
																								}
												break; 
									  case 7 : GPGGA.sat[ptr]=0;
											   GPGGA.nbSat=atoi(GPGGA.sat);
                                               break;            
                                      case 9 :   GPGGA.altitude[ptr]=0;
									          GPGGA.altidudeMeters=atol(GPGGA.altitude);
									          GPGGA.altidudeFeet=(long) GPGGA.altidudeMeters*328/100;
											  ltoa(GPGGA.altidudeFeet,GPGGA.feet,10);
											  break; 
                                      }
                                    }
              ptr=0;
              comma_count++;
              break;
  case '*' :  sentence_status=0;
              comma_count=0;
			  break;
  default:
   if (sentence_status==1) sentenceType[ptr++]=byteGPS;
   if (sentence_status==2) {
                             switch (comma_count)
                             {
                              case 1 :   if (ptr<6) GPGGA.hour[ptr++]=byteGPS;
                                         break;
                              case 2 :   GPGGA.Latitude[ptr++]=byteGPS;
                                         break;            
                              case 3 :   GPGGA.NS=byteGPS;
                                         break;            
                              case 4 :   GPGGA. Longitude[ptr++]=byteGPS;
                                         break;            
                              case 5 :   GPGGA.EO=byteGPS;
                                         break;            
                              case 6 :   GPGGA.fix=byteGPS-'0';
                                         break;            
                              case 7 :   GPGGA.sat[ptr++]=byteGPS;
                                         break;            
							  case 9 :   GPGGA.altitude[ptr++]=byteGPS;
                                         break;            
                             }
                       }
 }
}
 
 
 int32_t TNC::parseDecimal(const char *term)
{
  bool negative = *term == '-';
  if (negative) ++term;
  int32_t ret = 100 * (int32_t)atol(term);
  while (isdigit(*term)) ++term;
  if (*term == '.' && isdigit(term[1]))
  {
    ret += 10 * (term[1] - '0');
    if (isdigit(term[2]))
      ret += term[2] - '0';
  }
  return negative ? -ret : ret;
}

 /********************************************************
 * AX25 RX routines from  Robert Marshall, KI4MCW 
 ********************************************************/

 void TNC::Decode()

{
    // heartbeat on led
    digitalWrite(led,digitalRead(led)^1);
    // calculate ADC bias (average of last 128 ADC samples)
    // this input is decoulped from the receiver with a capacitor, 
    // and is re-biased to half of the Arduino regulated +3.3V bus
    // with a voltage divider. therefore the net bias should equal
    // (more or less) the settle point of the voltage divider.
    // doing this in software also means that the calculated bias
    // will re-center automatically if the resistors are not 
    // perfectly matched, etc.
    rawadc = ADCH ;
    bias_sum += rawadc ;
    if ( ++bias_cnt == 128 )
    {
        adc_bias = bias_sum >> 7 ;
        bias_cnt = 0 ;
        bias_sum = 0 ;
    }
    
   
    //=========================================================
    // Seguine math
    //    for details, see http://www.cypress.com/?docID=2328
    //=========================================================


    adcval = rawadc - adc_bias ;
	
    // circle buffer is just 7 elements (about half of a bit duration)
    if (++cb_pos == 7) { cb_pos = 0 ; }
	
    // back out old value from sum
    mult_sum -= mult_cb[ cb_pos ] ;
	
    // multiply the latest ADC value by the value ~1/2 lo-freq duration ago. if the 
    // incoming audio is the 1200 Hz MARK tone, the two samples will (usually) have 
    // opposite phases/signs, so multiplying the two values will give a negative result. 
    // If the incoming audio is 2200 Hz, the two samples usu. have the same phase/sign, 
    // so multiplying them will give a positve result.
    // subscript 5 = six samples ago-------v
    mult_cb[ cb_pos ] = adcval * adc_delay[5] ;
	
    // add this result to get the sum of the last 7 multipliers (1st LPF)
    mult_sum += mult_cb[ cb_pos ] ;
	
    // force a definitive answer with a hysteresis zone around zero (2nd LPF)
    if      ( mult_sum >=  100 ) { current_symbol = AX25_SPACE ; }
    else if ( mult_sum <= -100 ) { current_symbol = AX25_MARK ;  }
    else                         { ; } // inconclusive - dont change
		
    // increment # of samples since last symbol change, enforce a max
    if ( ++since_last_chg > 200 ) { since_last_chg = 200 ; }
    thesebits = 0 ;
	
    
    // rotate the delay
    for ( x=5 ; x>=1 ; x-- ) 
    {
        adc_delay[x] = adc_delay[x-1] ;
    }	
    adc_delay[0] = adcval ;

	
    //=============================
    //   clock and bit recovery
    //=============================

	
    // the in-a-frame and seeking-frame-start modes require different strategies
    // let's split them up here

    if ( inaframe ) 
    {
        //================================
	// clock recovery within a frame
	//================================
		
	// check symbol state only once per bit time (3rd LPF)
	bit_timer-- ;
		
	if ( current_symbol != last_symbol ) 
	{
	    // save the new symbol
	    last_symbol = current_symbol ;
	    // reset counter
	    since_last_chg = 0 ;
			
	    // Ideally, frequency transitions will occur on a since-last-change 
	    // count that is an exact bit duration at the 1200 Hz signaling 
	    // rate - that is, an exact multiple of 11 samples (11 samples = 1 bit, 
	    // 22 = 2 bits, 33 = 3, etc). To give some extra settle time, we 
	    // don't attempt to read the symbol value at the exact moment of 
	    // transition; instead, we give it 4 extra beats. Thus as bit_timer 
	    // is counting down to the next symbol check, its value should ideally 
	    // be 4 when the symbol change actually takes place. If the symbol 
	    // change is a little early or late, we can tweak the bit_timer to
	    // tolerate some drift and still keep sync.
	    // By those rules, an SLC of 4 is perfect, 2 through 6 are fine and  
	    // need no adjustment, 7,8 and 0,1 need adjustment, 9,10 are timing 
            // errors - we can accept a certain number of those before aborting.
			
	    if ( ( bit_timer == 7 ) || ( bit_timer == 8 ) )
	    {
	        // other station is slow or we're fast. nudge timer back.
		bit_timer -= 1 ;
		
	    }
	    else if ( ( bit_timer == 0 ) || ( bit_timer == 1 ) )
	    {
	        // they're fast or we're slow - nudge timer forward
		bit_timer += 1 ;
		  }	
	    else if ( ( bit_timer == 9 ) || ( bit_timer == 10 ) )
	    {
	        // too much error
		if ( ++sync_err_cnt > MAX_SYNC_ERRS ) 
		{
		    sync_err_cnt = 0 ;
		    msg_pos = 0 ;
		    inaframe = 0 ;
		    bitq = 0 ;
		    bitqlen = 0 ;
		    bit_timer = 0 ;
		    bittimes = 0 ;
		    // turn off DCD light
		   // DCD_OFF ;
		    return ;
		}
	//	else
	//	{
		    		
	//	}
	    } // end bit_timer cases
	} // end if symbol change
		
        //=============================
	// bit recovery within a frame
	//=============================
		
	if ( bit_timer == 0 )
	{
            // time to check for new bits
	    
	  		
	    // reset timer for the next bit
	    bit_timer = 11 ;
	    // another bit time has elapsed
	    bittimes++ ;
			
	    // wait for a symbol change decide what bits to clock in,
            // and how many
            if ( current_symbol != last_symbol_inaframe ) 
	    { 
	        // add one as ready-to-decode flag
		thesebits = bittimes + 1 ; 
		bittimes = 0 ;
		last_symbol_inaframe = current_symbol ;
            }
            
	} // end if bit_timer==0 

    } // end if inaframe

    else
    {
        //=================
	// not in a frame
	//=================
		
	// housekeeping
	// phase_err = since_last_change MOD 11, except that the "%" operator is =very slow=
	phase_err = since_last_chg ;
	while ( phase_err >= 11 ) { phase_err -= 11 ; }
	
	// elapsed bittimes = round (since_last_chg / 11)
	bittimes = 0 ;
	test = since_last_chg + 5 ;
	while ( test > 11 ) { test -= 11 ; bittimes++ ; }
	thesebits = 0 ;
	
	//====================================
	// clock recovery NOT within a frame
	//====================================
		
	// our bit clock is not synced yet, so we will need a symbol transition 
        // to either establish sync or to clock in bits (no transition? exit ISR)
	if ( current_symbol == last_symbol ) 
	{ return ; }
	
        // symbol change 

	// save the new symbol, reset counter
	last_symbol = current_symbol ;
	since_last_chg = 0 ;
	
	// check bit sync
	if ( ( phase_err >= 4 ) && ( phase_err <= 7 ) )
	{
	    // too much error
	    bitq = 0 ;
	    bitqlen = 0 ;
            // turn off the DCD light
           // DCD_OFF ;
	}	
		
	// save these bits
	thesebits = bittimes + 1 ;
			
    } // end else ( = not inaframe)
	
	
    //========================================
    //   bit recovery, in or out of a frame
    //========================================
	

    // if no bit times have elapsed, nothing to do
    if ( thesebits == 0 ) { return ; }
    else                  { thesebits-- ; }  // remove the "ready" flag
	
    // determine incoming bit values based on how many bit times have elapsed.
    // whatever the count was, the last bit involved a symbol change, so must be zero.
    // all other elapsed bits must be ones. AX.25 is transmitted LSB first, so in
    // adding bits to the incoming bit queue, we add them right-to-left (ie, new bits
    // go on the left). this lets us ready incoming bytes directly from the lowest
    // eight bits of the bit queue (once we have that many bits).
    
    // the act of adding bits to the queue is in two parts - (a) OR in any one bits,
    // shifting them to the left as required prior to the OR operation, and (b) update
    // the number of bits stored in the queue. with zero bits, there's nothing to
    // OR into place, so they are taken care of when we update the queue length, and 
    // when we shift the queue to the right as bytes are popped off the end later on.
    
    switch ( thesebits )
    {
    case 1: break ;    // no ones to add ------> binary       "0"
                            
    case 2: bitq |= ( 0x01 << bitqlen ) ;     // binary      "01"
            break ;
                           
    case 3: bitq |= ( 0x03 << bitqlen ) ;     // binary     "011"
            break ;

    case 4: bitq |= ( 0x07 << bitqlen ) ;     // binary    "0111"
            break ;
                           
    case 5: bitq |= ( 0x0F << bitqlen ) ;     // binary   "01111"
            break ;
                           
    // "six" is a special case ("bitstuff"): drop the zero, and only add the 5 one bits
    case 6: bitq |= ( 0x1F << bitqlen ) ;     // binary   "11111"
            thesebits = 5 ;
            break ;
                  
    // "seven" is another special case - only legal for an HDLC byte			  
    case 7:                                   // binary "0111111"
            if ( ( bitqlen == 1 ) && ( bitq == 0 ) )
	    {
	        // only one bit pending, and it's a zero 
		// this is the ideal situation to receive a "seven".
		// complete the HDLC byte
		bitq = 0x7E ;
		bitqlen = 8 ;
            }
            else if ( bitqlen < 4 )
            {
                // 0-3 bits still pending, but not the ideal single-zero.
		// this is kinda ugly. let's dump whatever is pending, 
		// and close the frame with a tidy right-justified HDLC.
		bitq = 0x7E ;
		bitqlen = 8 ;
            }				
            else if ( bitqlen >= 4 )
            {
                // also kinda ugly - half or more of an unfinished byte.
                // lets hang onto the pending bits by fill out this 
                // unfinished byte with zeros, and append the full HDLC 
                // char, so that we can close the frame neatly.
		bitq = ( bitq & 0xFF ) | 0x7E00 ;
		bitqlen = 16 ;
            }
            else 
            {
                // huh?!? ok, well, let's clean up
		bitq = 0x7E ;
		bitqlen = 8 ;
            }
            
            // we've already made the necessary adjustments to bitqlen,
            // so do not add anything else (below)
            thesebits = 0 ;
	    break ;
  
    default: 
            // less than a full bit, or more than seven have elapsed
            // clear buffers
            
            msg_pos = 0 ;
            inaframe = 0 ;
            bitq = 0 ;
            bitqlen = 0 ;
            // do not add to bitqlen (below)
            thesebits = 0 ;
            // turn off DCD light
          //  DCD_OFF ;
            break ;
                   
    } // end switch

    // how many bits did we add?
    bitqlen += thesebits ;
              
	
    //===================
    //   byte recovery
    //===================


    // got our bits in a row. now let's talk about bytes.

    // check the bit queue, more than once if necessary
    while ( bitqlen >= 8 )
    {
        // take the bottom 8 bits to make a byte
        byteval = bitq & 0xFF ;

        // special case - HDLC frame marker
        if ( byteval == 0x7E )
        {
            if ( inaframe == 0 ) 
            {
                // marks start of a new frame               
                inaframe = 1 ;
                last_symbol_inaframe = current_symbol ;
		sync_err_cnt = 0 ;
		bit_timer = 15 ;
		bittimes = 0 ;
                // pop entire byte (later)
                popbits = 8 ;
            }	

            else if ( msg_pos < MIN_PACKET_LEN )
            {
                // We are already in a frame, but have not rec'd any/enough data yet.
                // AX.25 preamble is sometimes a series of HDLCs in a row, so 
                // let's assume that's what this is, and just drop this byte.
		
                popbits = 8 ;
            }    
           
            else     
            {
                // in a frame, and have some data, so this HDLC is probably 
                // a frame-ender (and maybe also a starter)
				
                if ( msg_pos > 0 )
                {   /*
                    printf( "Message was:" ) ;
                    for ( x=0 ; x < msg_pos ; x++ )
                    {
                        printf( " %02X", msg[x] ) ;
                    }    
                    printf( "\n" ) ; 
                    printf( "Which decodes as:\n" ) ;
                    */
                    // send frame data out the serial port
                    decode_ax25() ;
                }

                // stay in inaframe-mode, in case a new frame is starting
		msg_pos = 0 ;
		sync_err_cnt = 0 ;
		bittimes = 0 ;
		// pop entire byte (later)
                popbits = 8 ;
              
            }  // end else for "if not inaframe"

        }  // end if byteval = 0x7E

        else if ( inaframe == 1 ) 
        {
            // not an HDLC frame marker, but =is= a data character
            

            // add it to the incoming message
            msg[ msg_pos ] = byteval ;
            msg_pos++ ;
            
            // is this good enough of a KISS frame to turn on the carrier-detect light?
            // we know we have an HDLC (because we're in a frame); 
            // check for end-of-header marker
          //  if ( byteval == 0x03 ) { DCD_ON ; }
            
            // pop entire byte (later)
            popbits = 8 ;
        }    

        else
        { 
            // not already in a frame, and this byte is not a frame marker.
            // It is possible (likely) that when an HDLC byte arrives, its 8 bits will not 
            // align perfectly with the 8 we just checked. So instead of dropping all 8 of 
            // these random bits, let's just drop one, and re-check the rest again later. 
            // This increases our chances of seeing the HDLC byte in the incoming bitstream
            // amid noise (esp. if we're running open squelch).
            popbits = 1 ; 
        }

        // pop the used bits off the end 
        // (i know, not really a "pop", more of a "drop")
        bitq = ( bitq >> popbits ) ;
        bitqlen -= popbits ;

    } // end while bitqueue >= 8

    // debug: check timing
    //end_time = TCNT3 ;

    sei() ;
    return ;
}  // end timerX interrupt




void TNC::decode_ax25 (void)
{
  // Take the data in the msg array, and send it out the serial port.

  x = 0 ;
  decode_state = 0 ;     // 0=just starting, 1=header, 2=got 0x03, 3=got 0xF0 (payload data)

  //*************solution actuelle
  // lop off last 2 bytes (FCS checksum, which we're not sending to the PC)
  for ( x = 0 ; x < (msg_pos - 2) ; x++ )
  {
    switch ( decode_state )  
    {
      // note the goofy order!!
    case 0:  
      // just starting

      Serial.write(0xC0);                 // frame start/end marker

      Serial.write(0x00);                 // data on port 0
      Serial.write(msg[x]);

      decode_state = 1 ;
      break ;

    case 2: 
      // got the 0x03, waiting for the 0xF0
      if ( msg[x] == 0xF0 ) 
      { 

        Serial.write(msg[x]);

        decode_state = 3 ;
      }
      else 
      {
        // wow - corrupt packet? abort

        // Serial.write(13); 
        // Serial.write(10); 
        return ;
      } 
      break ;

    case 1: 
      // in the header
      if ( msg[x] == 0x03 ) 
      { 

        Serial.write(msg[x]);

        decode_state = 2 ; 
        break ;
      }
      // else fall through

    default:
      // payload or header
      if ( msg[x] == 0xC0 ) { 
        Serial.write(0xDB) ;
      }
      Serial.write(msg[x]);
      if ( msg[x] == 0xDB ) { 
        Serial.write(0xDD) ;
      }
      break ;
    }  
    track[x]=msg[x];
  } // end for  
  Serial.write(0xC0);  // end of frame
  digiTX=1;
  taille=msg_pos-2;
}

 
 
 
 /*
//sinus table, generate with processing.org
//fe=13200,	13157.9 mesured
//te=1/13200 = 75.7µs (interrupt timer1 = 76)

int sinus[]=new int[512];

void setup()
{
int n;
  for (n=0;n<512;n++){
                    sinus[n]=(int) (128+127*sin(TWO_PI*(float)n/512));
                    print(sinus[n]+","); 
  }
}

void draw()
{
 
}



*/


