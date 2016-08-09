/*
    TNC APRS with DRA818
 
 Copyright (C) 2016 Anthony LE CREN <f4goh@orange.fr>
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

this program decode and repeat ax25 sentence with DRA818
miss digi callsign for routing (update soon)
also use kiss protocol on serial at 9600 bauds
 */

#include <SoftwareSerial.h>
#include <TNC.h>
#include <TimerOne.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


#define SETDIGI

#define PTT     3    // PTT pin. This is active low.
#define PwDw    2  // Power Down pin. This need to start low, then be set high before programming.


#define DRA_RXD 7   // The Arduino IO line that is connected to the DRA818's TXD pin. 3.3V only
// We *receive* TTL 3.3V Serial from the DRA818 on this pin.

#define DRA_TXD 6   // The Arduino IO line connected to the DRA818's RXD pin.
// We *send* TTL serial to the DRA818 on this pin.

#define bfPin 5     //afsk output must be PD5 
#define ledPin A2     //led for GPS fix


#define txPin 9      //tx pin into RX GPS connection
#define rxPin 8      //rx pin into TX GPS connection

#define gpsBaudrate 9600      //gps baud rate



SoftwareSerial dra_serial(DRA_RXD, DRA_TXD);    //for DRA818

SoftwareSerial gps(rxPin, txPin);  // RX, TX for GPS


LiquidCrystal_I2C lcd(0x27, 20, 4);    //4 lines *20 columns lcd char


// track char array

#define trackPHGSize 72
unsigned  char     trackPHG[trackPHGSize]= {
  'F'<<1, '4'<<1, 'G'<<1, 'O'<<1, 'H'<<1, ' '<<1, 0x60, //avant APTT4 7 octets (0-6)
  'F'<<1, '4'<<1, 'G'<<1, 'O'<<1, 'H'<<1, ' '<<1, ('0' + 2) << 1, //F4GOH-11 7 octets (7-13)
  'W'<<1, 'I'<<1, 'D'<<1, 'E'<<1, '1'<<1, ' '<<1, ('0' + 1) << 1, //WIDE1-1 7 octets (14-20)
  'W'<<1, 'I'<<1, 'D'<<1, 'E'<<1, '2'<<1, ' '<<1, ('0' + 1) << 1 | 1, //WIDE2-1   fin ssid lsb =1 7 octets (21-27)
  0x03, 0xf0, //ctrl, pid 2 octets (28-29)
  '/', '1', '5', '0', '4', '5', '2', 'h', //heure 8 (30-37)
  '4', '8', '5', '1', '.', '2', '0', 'N', '/', '0', '0', '2', '2', '0', '.', '9', '2', 'E', //lat, long 18 octets (38-55)
  '#', 'P', 'H', 'G', '7', '1', '6', '0', '4', '/', ' ', ' ', 'D', 'i', 'g', 'i'   // Calculate  your PHG at http://www.aprsfl.net/phgr.php 
};           



char gpsDataIn;


long previousMillisSerial = 0;
long currentMillisSerial;
long EcratMillisSerial;

unsigned char track[200];
unsigned char digi[200];
volatile int trackSize;
unsigned char memoriseState=0;
unsigned char kissEnable=0;
byte save_TIMSK0;

void setup() {
  Serial.begin(9600);

  
  lcd.begin();  //4 lines *20 columns lcd char
  lcd.setBacklight(HIGH);    
  lcd.clear();
  lcd.print(F("  APRS digi v1.0"));    //intro
  lcd.setCursor(0, 2);
  lcd.print(F("     F4GOH 2016"));
  
  pinMode(PwDw, OUTPUT);        //config dra818 ctrl lines
  pinMode(bfPin, OUTPUT);
  digitalWrite(PwDw, HIGH);

  pinMode(PTT, OUTPUT);
  digitalWrite(PTT, LOW);

  // start menu
  
if  (detectMenu()==1) ConfigMenu();


    ADMUX   = 0x43 ;                    // channel3, ref to internal input (vcc)
    ADMUX  |= (1<<ADLAR) ;              // left-justified (only need 8 bits)
    ADCSRA  = (1<<ADPS2) ;              // pre-scale 16
    ADCSRA |= (1<<ADATE) ;              // auto-trigger (free-run)
    ADCSRB  = 0x00 ;                    // free-running
    DIDR0  |= (1<<ADC0D) ;              // disable digital driver on ADC0 
    ADCSRA |= (1<<ADEN) ;               // enable ADC
    ADCSRA |= (1<<ADSC) ;               // trigger first conversion  

    Beacon.begin(bfPin, ledPin, 1200, 2200, digi);   //analog pin, led pin, freq1, freq2, ptr
#ifdef SETDIGI
  positionGps();          //enable if you use GPS on set Digi postion
#endif
 Timer1.initialize(76);    //µs  fe=13200 hz so TE=76µs 13157.9 mesured
 Timer1.attachInterrupt(decode);
 delay(2000);
 save_TIMSK0=TIMSK0;                  //save Timer 0 register
 TIMSK0 =0;    //disable Timer 0 and périphéral irq
 PCICR = 0;
#ifdef SETDIGI
   txing(3);    //send digi position
#endif   
//  Serial.print("ready");
}


void loop (void)
{
  unsigned  char a;
  if (Serial.available()>0) {
    a=Serial.read();
   // Serial.write(a);    //debug
    memoriseTrack(a);
  }

  // sleep_mode() ;
  if ((kissEnable==1) || (Beacon.digiTX==1)) { 
    delay500();
    delay500();
    Serial.flush();
    if (kissEnable==1) {
      lcd_data(track,trackSize);
      txing(1);
      kissEnable=0;
      // print_data(track,trackSize); //debug
    }
    if (Beacon.digiTX==1) {
      lcd_data(digi, Beacon.taille);
      txing(2);
      Beacon.digiTX=0;
      //print_data(digi, Beacon.taille);  //debug      
    }
  }
}


void lcd_data(unsigned char array[], int size_buffer) {
  TIMSK0 = save_TIMSK0;   //restore timer0 irq to use delay into i2c lcd library
  lcd.clear();
  int n;
  for (int n =0; n < 6; n ++) {
    lcd.write(array[n]>>1);
  }
  lcd.write(',');
  for (int n =7; n < 7+6; n ++) {    //print call
    lcd.write(array[n]>>1);
  }
  lcd.print(((array[13]/2)-'0'));   //print ssid
  lcd.write('-');
  int s;
  for (int n =0; n < size_buffer; n ++) if (array[n]==0xf0) s=n+1;    //find 0xf0
  n=1;
  int col=0;
  lcd.setCursor(0, n);
  while (s<size_buffer) { //print comments
    lcd.write(array[s++]);
    col++;
    if (col==20) {
      n++;
      col=0;
      lcd.setCursor(0, n);
    }
  }
  TIMSK0 = 0;
}

void print_data(unsigned char array[], int size_buffer) {
  for (int n =0; n < size_buffer; n ++) {
    Serial.write(array[n]);
    //Serial.print(",");
  }
  Serial.println(trackSize);
}

void memoriseTrack(unsigned char car)
{
  // if ( kissEnable==0){
  if (memoriseState==0) if (car==0xC0) {
    memoriseState=1;
    trackSize=0;
    return;
  }
  if (memoriseState==1) if (car==0x00) {
    memoriseState=2;
    return;
  }
  if (memoriseState==2) if (car!=0xC0) {
    track[trackSize++]=car;
  }
  else
  {
    memoriseState=0;
    kissEnable=1;
  }

  // }
}

void positionGps()
{
  gps.begin(gpsBaudrate);
  while (Beacon.GPGGA.fix==0) {
    if (gps.available()) {
      gpsDataIn=gps.read();
      Serial.write(gpsDataIn);
      Beacon.gpsnmea(gpsDataIn);
    }
  }
  memcpy(trackPHG +31, Beacon.GPGGA.hour, 6);      //prepare APRS char array track to send
  memcpy(trackPHG +38, Beacon.GPGGA.Latitude, 7);     //beware index number from char array
  trackPHG[45] = Beacon.GPGGA.NS;
  memcpy(trackPHG +47, Beacon.GPGGA.Longitude, 8);
  trackPHG[55] = Beacon.GPGGA.EO;
  gps.end();
}


void delay500()   //use this delay delay(500) when timer0 irq is disabled
{
  unsigned long count;
  for (count=0;count<18500;count++) {
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
  };
}

void txing(byte sentenceType)
{
  digitalWrite(PTT, HIGH);            //ptt on
  delay500();
  delay500();
  TCCR0B = TCCR0B & 0b11111000 | 1;    //switch to 62500 HZ PWM frequency
  Timer1.attachInterrupt(sinus_irq);   //warp interrupt in library
  switch (sentenceType)
  {
  case 1 : 
    Beacon.sendpacket(track, trackSize);  //send packet
    break;
  case 2 : 
    Beacon.sendpacket(digi, Beacon.taille);  //send packet
    break;
  case 3 : 
    Beacon.sendpacket(trackPHG, trackPHGSize);  //send packet
    break;
  }
  digitalWrite(PTT, LOW);              //PTT off
  Timer1.detachInterrupt();            //disable timer1 interrupt
  Timer1.attachInterrupt(decode);
  analogWrite(bfPin, 0);                //PWM at 0
  TCCR0B = TCCR0B & 0b11111000 | 3;    //register return to normal
}



void decode()    //warp timer1 irq into TNC lib to decode ax25
{
  Beacon.Decode();
}


void sinus_irq()    //warp timer1 irq into TNC lib to send ax25
{
  Beacon.sinus();
}

byte detectMenu()
{
  // lcd.print(F("m for boot menu"));
  Serial.println(F("m for boot menu"));
  // lcd.setCursor(0,1);
  char car=0;
  char countDown=-1;
  digitalWrite(ledPin, HIGH);
  previousMillisSerial = millis();
  do {
    currentMillisSerial = millis();  
    EcratMillisSerial=currentMillisSerial - previousMillisSerial;
    if (Serial.available()>0) {
      if (Serial.read()=='m') {     
        return 1;
      }
    }
    if ((EcratMillisSerial/1000)!=countDown) {
      countDown++;
      //  lcd.write(countDown+0x30);
      Serial.write(countDown+0x30);
    }
  }
  while (EcratMillisSerial<5000);
  digitalWrite(ledPin, LOW);
  Serial.println();
  return 0;
}

void ConfigMenu()
{
  char carMenu;
  //lcd.clear();
  //lcd.print(F("Boot menu"));
  do {
    carMenu=0;
    Serial.println(F("-----------"));
    Serial.println(F("Config menu"));
    Serial.println(F("1 set DRA 818 to 144.8000 Mhz"));
    Serial.println(F("2 set DRA 818 to volume max 8"));
    Serial.println(F("3 set DRA 818 to normal filter"));
    Serial.println(F("4 Test GPS (for later software upgrade)"));
    Serial.println(F("0 Quit menu"));
    Serial.println(F("-----------"));
    while (carMenu==0) if (Serial.available()>0) carMenu=Serial.read();
    switch(carMenu) {
    case '1' : 
      configDra818("AT+DMOSETGROUP=0,144.8000,144.8000,0000,4,0000");
      break;
    case '2' : 
      configDra818("AT+DMOSETVOLUME=8");
      break;
    case '3' : 
      configDra818("AT+SETFILTER=0,0,0");
      break;
    case '4' : 
      testGps();
      break;
    case '0' :
      Serial.println(F("ok quit"));
      break;
    default : 
      Serial.println(F("error"));
    }
    //updateAll();
  } 
  while (carMenu!='0');
}

void configDra818(char *cmd)
{
  char ack[3];
  dra_serial.begin(9600);
  dra_serial.println(cmd);
  ack[2]=0;
  previousMillisSerial = millis();
  do
  {
    if (dra_serial.available()>0) {
      ack[0]=ack[1];
      ack[1]=ack[2];
      ack[2]=dra_serial.read();
    }
    currentMillisSerial = millis();  
    EcratMillisSerial=currentMillisSerial - previousMillisSerial;
  }
  while ( (ack[2]!=0xa)&&(EcratMillisSerial<2000));
  Serial.println(cmd);
  dra_serial.end();
  if (ack[0]==0x30) Serial.println(F("DRA update done")); 
  else Serial.println(F("DRA update error"));
}

/*
+DMOSETGROUP:0  //command sucessfull 0x30,0xd,0xa
 +DMOSETGROUP:1  //command failed 0x31,0xd,x0a
 so read 3 last bytes into shifted buffer until 0xa found
 then test ack[0] 0x30 or 0x31
 */



void testGps()
{  
  gps.begin(gpsBaudrate);
  char s=0;
  Serial.println(F("GPS testing...s to Stop"));
  do
  {  
    if (Serial.available()) {
      s=Serial.read();
    }
    if (gps.available()) {
      gpsDataIn=gps.read();
      Serial.write(gpsDataIn);
    }
  }
  while (s!='s');
  Serial.println(F("Stop GPS test"));
  s=Serial.read();
  gps.end();
}


/*
//DRA818 INFOS
 //Handshake  +crlf
 // AT+DMOCONNECT
 
 // Format˖AT+DMOSETGROUP=BWˈTX_FˈRX_FˈTx_subaudioˈSQˈRx_subaudio
 // AT+DMOSETGROUP=0,144.8000,144.8000,0000,4,0000
 // SQ 0 à 8
 
 //Scan fréquency
 //S+144.8000
 //Réponse OK -> S=0 0=found 1=no signal
 
 //Volume configuration 1 a 8
 //AT+DMOSETVOLUME=4
 
 //AT+SETFILTER=PRE/DE-EMPH, HIGHPASS, LOWPASS
 //AT+SETFILTER=0,0,0
 
 */


/*
//debug dra command and txing with $ car (bridge between 2 serial port)
/*  
 char car;
 if (Serial.available()>0){
 car=Serial.read();
 if (car=='$') {
 txing();
 }
 else
 if (car=='*') digitalWrite(PTT, LOW);
 else                          
 dra_serial.write(car);  
 }
 
 if (dra_serial.available()>0){
 Serial.write(dra_serial.read());  
 }
 */

 /*
test sentence
C0 00 8C 68 8E 9E 90 40 60 8C 68 8E 9E 90 40 74 AE 92 88 8A 62 40 62 AE 92 88 8A 64 40 63 03 F0 2F 31 35 30 34 35 32 68 34 38 35 31 2E 32 30 4E 2F 30 30 32 32 30 2E 39 32 45 3E 37 33 20 41 6E 74 C0 
 */


