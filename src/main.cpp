// Arduino timer CTC interrupt example
// www.engblaze.com

// avr-libc library includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Arduino.h"

#define LEDPIN 13
#define RELAY_PIN 4
#define ABORT_PIN 2
#define MANUAL_WATER_PIN 3
#define ON 0
#define OFF 1

volatile byte seconds;
volatile byte water_duration_timer;
volatile int water_now = 0;
volatile int relay_state = 1;
boolean off = 1;
boolean on = 0;
boolean debug = 0;

void emergency_abort()
{
	water_now = 0;

	if ( debug == 1 ) {
		Serial.println("Giessvorgang per Notaus abgebrochen");
	}

	detachInterrupt(digitalPinToInterrupt(ABORT_PIN));
}

void water_manually()
{
	water_now = 1;

	if ( debug == 1 ) {
		Serial.println("Manueller Giessvorgang initiiert");
	}

	detachInterrupt(digitalPinToInterrupt(MANUAL_WATER_PIN));
}

void setup()
{
  if (debug == 1) {
	  Serial.begin(9600);
	  delay(5000);
  }

  pinMode(LEDPIN, OUTPUT); 
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(ABORT_PIN, INPUT_PULLUP);
  pinMode(MANUAL_WATER_PIN, INPUT_PULLUP);
  digitalWrite( RELAY_PIN, off );
  relay_state = off;
  attachInterrupt( digitalPinToInterrupt(ABORT_PIN), emergency_abort, LOW );
  attachInterrupt( digitalPinToInterrupt(MANUAL_WATER_PIN), water_manually, LOW );

  cli();      // disable global interrupts
  
  // initialize Pin Change Interrupt for abort button
/*  PCICR = 0;

  PCICR |= ( 1 << PCIE0); // activate pin change interrupt on pin 10
  PCMSK0 |= ( 1 << PCINT2); //
*/

  // initialize Timer1
  TCCR1A = 0;   // set entire TCCR1A register to 0
  TCCR1B = 0;   // same for TCCR1B

  // set compare match register to desired timer count:
  OCR1A = 15624;
  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  // enable global interrupts:
  sei();
}

void loop()
{
    // Wenn relay aus und es soll gegossen werden:
    // Relays einschalten und angeschalteten Zustand merken
    if ( water_now == 1 && relay_state == off  ) {
      digitalWrite( RELAY_PIN, on );
      relay_state = on;
    } 

    // Wenn relay an und es soll nicht gegossen werden:
    // Relays ausschalten und ausgeschalteten Zustand merken
    if ( water_now == 0 && relay_state == on ) {
      digitalWrite( RELAY_PIN, off );
      relay_state = off;
    }
   
}

ISR(TIMER1_COMPA_vect)
{
    seconds++;
    if ( seconds == 5 ) {
      seconds = 0;
      int sensorValue0 = analogRead(A0);
      int sensorValue1 = analogRead(A1);
      int delta = ( sensorValue0 + sensorValue1 ) / 2;
      int poti = analogRead(A2);
      int giessdauer=((poti/17)+3);

	  if (debug == 1){
		  Serial.print("Feuchte: ");
		  Serial.print(delta);
		  Serial.print("  --  Poti: ");
		  Serial.print(poti);
		  Serial.print("  --  Giessdauer(s): ");
		  Serial.print(giessdauer);
		  Serial.print("  --  Relaystatus [0=on|1=off]: ");
		  Serial.print(relay_state);
		  Serial.print("  -- wir giessen grade [0=nein|1=ja]: ");
		  Serial.println(water_now);
	  }
      

      if ( delta < 300 ) {
        water_now = 1;
      }
    }

    if ( water_now == 1 ){
      int poti = analogRead(A2);
      int giessdauer=((poti/17)+3); // Giessdauer in s
      water_duration_timer++;

	  if (debug == 1) {
		  Serial.print("Timer: ");
		  Serial.print(water_duration_timer);
		  Serial.print(" / ");
		  Serial.println(giessdauer);
	  }

      if ( water_duration_timer == giessdauer ) {
        water_duration_timer = 0;
        water_now = 0;
      }
    }
    
  digitalWrite(LEDPIN, !digitalRead(LEDPIN));
  attachInterrupt( digitalPinToInterrupt(ABORT_PIN), emergency_abort, LOW );
  attachInterrupt( digitalPinToInterrupt(MANUAL_WATER_PIN), water_manually, LOW );
}

