#ARDUINO APRS and DRA818 TNC module library #
F4GOH Anthony f4goh@orange.fr <br>

Aout 2016

Use this library freely with Arduino 1.6.5

## Update protocols ##
- APRS TNC with repeater
- Using DRA818 for transmit and decode

## Installation ##
To use the TNC library:  
- Go to https://github.com/f4goh/TNC, click the [Download ZIP](https://github.com/f4goh/TNC/archive/master.zip) button and save the ZIP file to a convenient location on your PC.
- Uncompress the downloaded file.  This will result in a folder containing all the files for the library, that has a name that includes the branch name, usually TNC-master.
- Rename the folder to  TNC.
- Copy the renamed folder to the Arduino sketchbook\libraries folder.

- you must add Arduino TimerOne library : <br>
  Go to http://playground.arduino.cc/Code/Timer1

## Usage notes ##


To use 4x20 characters LCD Display, the LiquidCrystal_I2C and WIRE libraries must also be included.


```c++
#include <SoftwareSerial.h>
#include <TNC.h>
#include <TimerOne.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
```

Change your callsign into unsigned char array to set digi position and PHG config

```c++
#define trackPHGSize 72
unsigned  char     trackPHG[trackPHGSize]= {
  'F'<<1, '4'<<1, 'G'<<1, 'O'<<1, 'H'<<1, ' '<<1, 0x60, //avant APTT4 7 octets (0-6)
  'F'<<1, '4'<<1, 'G'<<1, 'O'<<1, 'H'<<1, ' '<<1, ('0' + 2) << 1, //F4GOH-2 7 octets (7-13)
  'W'<<1, 'I'<<1, 'D'<<1, 'E'<<1, '1'<<1, ' '<<1, ('0' + 1) << 1, //WIDE1-1 7 octets (14-20)
  'W'<<1, 'I'<<1, 'D'<<1, 'E'<<1, '2'<<1, ' '<<1, ('0' + 1) << 1 | 1, //WIDE2-1   fin ssid lsb =1 7 octets (21-27)
  0x03, 0xf0, //ctrl, pid 2 octets (28-29)
  '/', '1', '5', '0', '4', '5', '2', 'h', //heure 8 (30-37)
  '4', '8', '5', '1', '.', '2', '0', 'N', '/', '0', '0', '2', '2', '0', '.', '9', '2', 'E', //lat, long 18 octets (38-55)
  '#', 'P', 'H', 'G', '7', '1', '6', '0', '4', '/', ' ', ' ', 'D', 'i', 'g', 'i'   // Calculate  your PHG at http://www.aprsfl.net/phgr.php 
}; 
```
Enable or disable SETDIGI

```c++
//#define SETDIGI
```
if SETDIGI is uncomment : GPS module needs to be connected to send digipeater position
if SETDIGI is comment : no GPS module needs, no digipeater position is sent

ADC converter is configure as free run ACQ

```c++
    ADMUX   = 0x43 ;                    // channel3, ref to internal input (vcc)
    ADMUX  |= (1<<ADLAR) ;              // left-justified (only need 8 bits)
    ADCSRA  = (1<<ADPS2) ;              // pre-scale 16
    ADCSRA |= (1<<ADATE) ;              // auto-trigger (free-run)
    ADCSRB  = 0x00 ;                    // free-running
    DIDR0  |= (1<<ADC0D) ;              // disable digital driver on ADC0 
    ADCSRA |= (1<<ADEN) ;               // enable ADC
    ADCSRA |= (1<<ADSC) ;               // trigger first conversion  
```

DRA818 speed must be 9600 bauds

```c++
dra_serial.begin(9600);
```

Sometimes TIMSK0 be clear to have only timer1 IRQ running

```c++
 save_TIMSK0=TIMSK0;                  //save Timer 0 register
 TIMSK0 =0;    //disable Timer 0 and périphéral irq
```
