

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#define SAMPLE_RATE 8000


#include "sounddata.h"
#include "sounddata2.h"
#include "sounddata3.h"
#include "Arduino.h"

int ledSum = 0;
int ledPins[] = {1, 2, 4, 5, 6 , 7};
long startTime = 0;
int sensorPins[] = {A0, A1, A2, A3, A4 ,A5};
int sensorValues[] = {0,0,0,0,0,0};
int prevSensors[] = {0,0,0,0,0,0};
int ledOn[] = {0,0,0,0,0,0};

int speakerPin = 3; // Can be either 3 or 11, two PWM outputs connected to Timer 2
volatile uint16_t sample;
byte lastSample;
long randO = 1;
int switcheroo = 0;
int hitSensor = 0;


float sensorThreshV = 2.2;
int sensorThresh = (int) (sensorThreshV  * 1023/5) ; 
int starterShot=0;


void stopPlayback()
{
  // Disable playback per-sample interrupt.
  TIMSK1 &= ~_BV(OCIE1A);

  // Disable the per-sample timer completely.
  TCCR1B &= ~_BV(CS10);

  // Disable the PWM timer.
  TCCR2B &= ~_BV(CS10);

  digitalWrite(speakerPin, LOW);

}

// This is called at 8000 Hz to load the next sample.
ISR(TIMER1_COMPA_vect) {
  if (sample >= sounddata_length) {
    if (sample == sounddata_length + lastSample) {
      stopPlayback();
    }
    else {
      if(speakerPin==11){
        // Ramp down to zero to reduce the click at the end of playback.
        OCR2A = sounddata_length + lastSample - sample;
      } 
      else {
        OCR2B = sounddata_length + lastSample - sample;                
      }
    }
  }
  else {
    switch(switcheroo){
    case 1: 
      if(speakerPin==11){

        OCR2A = pgm_read_byte(&sounddata_data[sample]);
      } 
      else {
        OCR2B = pgm_read_byte(&sounddata_data[sample]);            
      }
      break;
    case 2:
      if(speakerPin==11){

        OCR2A = pgm_read_byte(&sounddata2_data[sample]);
      } 
      else {
        OCR2B = pgm_read_byte(&sounddata2_data[sample]);            
      }
      break;
    default:
      if(speakerPin==11){

        OCR2A = pgm_read_byte(&sounddata3_data[sample]);
      } 
      else {
        OCR2B = pgm_read_byte(&sounddata3_data[sample]);            
      }
      break;
    }

  }

  ++sample;
}

void startPlayback()
{
  pinMode(speakerPin, OUTPUT);
  


  // Set up Timer 2 to do pulse width modulation on the speaker
  // pin.

  // Use internal clock (datasheet p.160)
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));

  // Set fast PWM mode  (p.157)
  TCCR2A |= _BV(WGM21) | _BV(WGM20);
  TCCR2B &= ~_BV(WGM22);

  if(speakerPin==11){
    // Do non-inverting PWM on pin OC2A (p.155)
    // On the Arduino this is pin 11.
    TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
    TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
    // No prescaler (p.158)
    TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

    // Set initial pulse width to the first sample.
    switch(switcheroo)
    {
    case 1:
      OCR2A = pgm_read_byte(&sounddata_data[0]);
      break;
    case 2:
      OCR2A = pgm_read_byte(&sounddata2_data[0]);
      break;
    default:
      OCR2A = pgm_read_byte(&sounddata3_data[0]);
      break;
    }
  } 
  else {
    // Do non-inverting PWM on pin OC2B (p.155)
    // On the Arduino this is pin 3.
    TCCR2A = (TCCR2A | _BV(COM2B1)) & ~_BV(COM2B0);
    TCCR2A &= ~(_BV(COM2A1) | _BV(COM2A0));
    // No prescaler (p.158)
    TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

    // Set initial pulse width to the first sample.
    switch(switcheroo) {

    case 1:
      OCR2B = pgm_read_byte(&sounddata_data[0]);
      break;
    case 2:
      OCR2B = pgm_read_byte(&sounddata2_data[0]);
      break;
    default:
      OCR2B = pgm_read_byte(&sounddata3_data[0]);
      break;
    }
  }





  // Set up Timer 1 to send a sample every interrupt.

  cli();

  // Set CTC mode (Clear Timer on Compare Match) (p.133)
  // Have to set OCR1A *after*, otherwise it gets reset to 0!
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));

  // No prescaler (p.134)
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

  // Set the compare register (OCR1A).
  // OCR1A is a 16-bit register, so we have to do this with
  // interrupts disabled to be safe.
  OCR1A = F_CPU / SAMPLE_RATE;    // 16e6 / 8000 = 2000

  // Enable interrupt when TCNT1 == OCR1A (p.136)
  TIMSK1 |= _BV(OCIE1A);
  switch(switcheroo) 
  {
  case 1:
    lastSample = pgm_read_byte(&sounddata_data[sounddata_length-1]); 
    break;
  case 2:
    lastSample = pgm_read_byte(&sounddata2_data[sounddata_length-1]);
    break;
  default:
    lastSample = pgm_read_byte(&sounddata3_data[sounddata_length-1]);
    break;
  }
  sample = 0;
  sei();
}



void setup()
{
  for( int i = 0; i<6 ; i++)
  { pinMode(sensorPins[i], INPUT);   
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
   // digitalWrite(sensorPins[i],HIGH);
  }

}

void loop()
{
  unsigned long currentTime = millis();
 
  for( int i =0 ; i<6 ; i++)
  {  
      
      sensorValues[i] = analogRead(sensorPins[i]);
          
     if (sensorValues[i] < sensorThresh && prevSensors[i] > sensorThresh && (currentTime-startTime) > 1000 && ledOn[i] == 0 ) 

        {
      
          if(starterShot <= 2)
          {
            randomSeed(currentTime % 1024);
            switcheroo=random(1,3);
            starterShot++;
          }
          else
          {
      
            randO=random(1,11);
            switcheroo = floor(randO/5.0)+1;
          }
      
          startPlayback();
          startTime=currentTime;
          digitalWrite(ledPins[i], HIGH);
          ledOn[i]=1;
          ledSum++;
      
        }
      
 
      prevSensors[i] = sensorValues[i];
    
    
  }
  
  if(ledSum >= 6)
    {
      int toggler = 0;
      for(int x = 0; x< 5; x++)
      {
            switcheroo = random(1,3);
            startPlayback();
            for(int i =0 ; i<6 ; i++)
            {  digitalWrite(ledPins[i],toggler) ;}
            delay(500);
            toggler = !toggler;
        
      }
      
      switcheroo = random(1,3);
      startPlayback();
      for(int i =0 ; i<6 ; i++)
      { digitalWrite(ledPins[i],HIGH); }
      delay(1000);
      
       for(int i =0 ; i<6 ; i++)
      { digitalWrite(ledPins[i],LOW); 
         prevSensors[i]=0;
         ledOn[i]=0;}
      ledSum=0;
      
    }
  
 
}

