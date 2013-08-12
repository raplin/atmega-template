/* $CSK: lesson2.c,v 1.3 2009/05/17 06:22:44 ckuethe Exp $ */
/*
 * Copyright (c) 2008 Chris Kuethe <chris.kuethe@gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

//for code 0120 (ABEX TV)
//fairly fast, gap modulated

#define F_CPU 16000000

#include <stdint.h>
#include <stdio.h>

#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/interrupt.h>

typedef unsigned char uint8;
typedef unsigned int uint16;

// ---------------------------------------------
// HW


//----------------------------------------------
void print(char *str);
void got_char(uint8 c);


#define IRPIN (13-8)  //PCINT15 / PC5


// ---------------------------------------------
// GLOBALS
uint8 gTicks=0;
uint8 LEDSTATE=0;
volatile uint8 hackval=0,hackval2=0;

uint8 gServoEnable=1;
#define NUM_SERVOS 4
uint8 gServoPos[ NUM_SERVOS ];


#define LED(n) {if (n) LEDSTATE|=0x20; else LEDSTATE&=~0x20; }

// ---------------------------------------------
// Timer IRQ

ISR(TIMER0_COMPA_vect) 
{
    gTicks++;
}

uint8 gNextPORTB=0;
uint8 gServoIndex=0;
uint16 gNextPulseLength=10000;

ISR(TIMER1_COMPA_vect) 
{
    PORTB=gNextPORTB;
    OCR1A=gNextPulseLength;
    gServoIndex=(gServoIndex+1) & (NUM_SERVOS-1);
    gNextPORTB=(gServoEnable?1:0)<<gServoIndex;
    gNextPulseLength=(800/4)+((uint16)gServoPos[gServoIndex]<<1);
    if (gServoIndex==0 && gServoEnable) gServoEnable--;
}

 //#define NORMAL_TCCR1B (1<<WGM12) | (1<<CS11)
 //#define NORMAL_TCCR1A ((0<<COM1A1) | (1<<COM1A0))  //<<toggle oc1a (pb1) (servo shifter /latch) on compare match
 
void init_irqs()
{
  TIMSK0=0;
  TCCR0A=(1<<WGM01); //| NORMAL_TCCR1A;
  OCR0A=0xff;  //1024 prescale from xtal = 16Mhz / 1024 = 15625hz or 64us, so each overflow at 255 = about 60hz
  //on 20mhz xtal / 1024 = 19531.25khz (51.2us) , so / 2= about 10khz, 200=500hz

  TCCR0B=(1<<CS02)|(1<<CS00);   ///1024
  TCNT0=0;  //reset count
  TIMSK0=(1<<OCIE0A);  //int on ocr A
  
  //16mhz=62.5ns / clk. Prescale/64 = 4.00us
  //timer1 for servos
  TCCR1A=(0<<WGM11)|(0<<WGM10); //CTC mode
  TCCR1B=(1<<WGM12)|(0<<WGM13)    |   (1<<CS11)|   (1<<CS10);  //011=clk/64=4us per clock
  TCNT1=0;
  OCR1A=10000;  //counter  resets on ocr1a. ocr1a can be set any time (not buffered)
  TIMSK1=(1<<OCIE1A);  //int on ocr A
  //TIMSK1=(1<<TOIE1); //overflow irq enable


  //pin change on ir input (pc) pc13
  PCMSK1=1<<(IRPIN);
  PCICR|=1<<PCIE1; //pins 14..8

  sei();
  
}

void init_ports()
{
    DDRB = 0xFF;  //d0..n are servos
    DDRC = 0x00;  //PC5 / PCINT13 is ir input
    DDRD = 0x00;

}


// ---------------------------
#define RXLEN (16*2)

#define RX_PACKETS 0
#if RX_PACKETS
uint8 rxpos=0;
volatile uint8 rxpackets=0;
uint8 rxdata[RXLEN];
#endif


ISR(USART_RX_vect) 
{
  uint8 c=UDR0;
  got_char( c);
#if RX_PACKETS
    rxdata[rxpos++]=c;
    if (rxpos==RXLEN){
      rxpos=0;
      rxpackets=1;
    }
#endif

}

ISR(USART_TX_vect) 
{
}

#define IR_ZERO 0x11
#define IR_ONE 0x21
#define IR_MAX 0x30

#define IR_BYTE1_CODE 0xaa

uint8 gLastIRCount=0,gLastIRTicks=0;
uint8 gShifter=0;
uint8 gRXCount=0,gRXByte=0, gIRRemote=0;

ISR(PCINT1_vect) 
{
  //pin change 1 (IR receiver)
  uint8 now=TCNT0;
  if (PINC&(1<<IRPIN)){ 
      //end of carrier pulse (rising edge, pin now 1)
      uint8 delta=(now-gLastIRCount) & 0x7f;
      uint8 shiftCopy=gShifter;
      uint8 err=0;

      #if 1
      if ((gTicks - gLastIRTicks>1) || (delta>  IR_MAX)){
        err=1;
      }else{
        if (delta< IR_ZERO+ ((IR_ONE-IR_ZERO)/2) ){
          gShifter>>=1;
          //UDR0='0';
        }else {
          gShifter=(gShifter>>1)|0x80;
          //UDR0='1';
        }
        
        if (shiftCopy&1){
          //UDR0=gShifter;
          //read 8 bits
          if (!(gRXCount&1)){
            gRXByte=gShifter; //every odd byte is inverse of previous
          }else{
            gShifter^=0xff;
            if (gRXByte == gShifter){
              if (gRXCount&2){
                gIRRemote=gShifter; //done
                got_char('0'+gIRRemote);
                //UDR0=gIRRemote;
              }else{
                if (gShifter!=IR_BYTE1_CODE) err=1;
              }
            }else{
              err=1;
              //error
            }
          }
          gRXCount+=1;
          gShifter=0x80;
        }
      }
      if (err)
      {
          //gap too long, reset
        //UDR0='\n';
        gShifter=0x80;
        gRXCount=0;
      }
    #else
      UDR0=delta;
    #endif
    

    gLastIRTicks=gTicks;

    gLastIRCount=now;
  }else{
    //carrier starting  (falling edge)
  }
  
}


void Term_Send( int data )
{
    UCSR0A=(1<<TXC0);  //clear (not reqd if irq enabled)
    UDR0=(uint8)data;
    while(!(UCSR0A & (1<<TXC0)));
}

int Term_Get()  //dummy don't care about getch
{
    return 0;
}


void print(char *str)
{ //dumb;blocks
  char c;
  while ((c=*str++)!=0){
    UCSR0A=(1<<TXC0);  //clear (not reqd if irq enabled)
    UDR0=c;
    while(!(UCSR0A & (1<<TXC0)));

  }
}

void init_print()
{
  FILE *ser;
  ser=fdevopen(Term_Send, Term_Get);

  UCSR0A = (1<<RXC0) | (1<<TXC0);
  UCSR0C=(1<<UCSZ01) | (1<<UCSZ00);
  UBRR0 = 8; //21; //@16mhz: 16=57600 , 8=115200 , 3=250k  //115.2k (-3.5%) at 16mhz (at 20mhz use 10)
               // @20mhz: 21:57600 10=115200,, 
  UCSR0B=(1<<RXCIE0) |/* (1<<TXCIE0) |*/ (1<<RXEN0) | (1<<TXEN0);
//  UCSR0B=(1<<TXCIE0) | (1<<RXEN0) | (1<<TXEN0);
//  UCSR0B=(1<<RXEN0) | (1<<TXEN0);
  _delay_ms(2);
  print("\n\n\n\nMoballs " __DATE__ " : " __TIME__ "\r\n(C) 2013 Richard Aplin\r\n");
}


// -------------------------------------

void do_task(void)
{
//  gServoPos[0]=hackval<<0;
//  gServoPos[1]=hackval2<<0;
//  gServoPos[2]=hackval2<<0;
}

#define POS_MAX 0xb0
#define POS_HALFWAY (POS_MAX/2)

void got_char(uint8 c)
{
  uint8 t;
  uint8 mc=c;
  if (c==0x4c){
    gServoEnable=0;
    return;
  }
  mc-='1';
  for(t=0;t<3;t++){
    if (mc<3){
      uint8 hackval=gServoPos[t];
      uint8 copy=hackval;
      if (mc==0 && hackval) hackval-=2;
      if (mc==1)hackval=POS_HALFWAY;
      if (mc==2 && hackval!=POS_MAX) hackval+=2;
      if (copy!=hackval){
        gServoEnable=250;
        gServoPos[t]=hackval;
      }
      return;
    }
    mc-=3;
  }  
  UDR0=c;

}

       
int main (void)
{

#define WATCHDOG_TIMEOUT WDTO_120MS
    init_ports();
    init_irqs();
    init_print();
    wdt_enable (WATCHDOG_TIMEOUT);

    while (1) {
        do_task();
        wdt_reset ();
    }
    return 0;
}

