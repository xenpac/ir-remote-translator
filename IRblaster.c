/*
 AVR IR Blaster. Infrared Remote Control Translater and Blaster.
 This code uses the AVR Atmega48PA microcontroller.
 It has 4096 Bytes of flash, 512Bytes of RAM, 256Bytes of eeprom.
 The SFH5110 IR receiver is not working below 4V !! so we use 5V Supply, maybe 4.3V Lion accu may work.
Remote controls usual sleep current is around 1 uA .!

Programing instructions see "learncode".

 Also contained is a working write flash routine.
Flash programming:
In ATmega 48/48PA there is no Read-While-Write support, and no separate Boot Loader Section. The SPM
instruction can be executed from the entire Flash.
This means that the CPU is halted while the flash programming takes place, which is fine.
Here the SPM instruction performs a complex flash programming algorithm and while doing so it disconnects the clock to the CPU
which is then halted. (to not access the flash meanwhile!)

 Copyright Thomas Krueger, Hofgeismar, Germany
 December 2019
 */

#include <io.h>
#include <wdt.h>
#include <sleep.h>
#include <interrupt.h>
#include <stdlib.h>
#include <eeprom.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <string.h>


// ++++++++++++++++++++++++ DEFINES +++++++++++++++++++++++++++++++++++
#define BYTE unsigned char
#define WORD unsigned short
#define ULONG unsigned long

// Macros: Bit operations on byte/word/long variables. Format: bset(Bit,variable). works also on SFRs(creates sbi,cbi)
#define bset(x,y) (y |= (1 << x))
#define bclr(x,y) (y &= (~(1 << x)))
#define btst(x,y) (y & (1 << x))

//SPM_PAGESIZE the processors pagesize is defined in iom48pa.h, the processor file. 64Bytes
#define MINPAGE 42   //0xA80 startpage of ircode-table in flash after code, check that out !!!
#define MAXPAGE 63   // 0xFC0 start of last page in flash


//#define DBPRINT   // to get debug of frames on serial. saves a lot of space if off!!
#ifdef DBPRINT
char sbuf[80];
void printdb(void);
void putcc(char c);
void putss(char *ps);
char getcc(void);
#endif

//protos:
void flash_read_page (uint32_t page, uint8_t *buf);
void flash_write_page (uint32_t page, uint8_t *buf);
void set_receiver(void);
void set_transmitter(void);
BYTE decodebuf(void);
void setuptxbuf(void);
BYTE findcode( void);
BYTE learncode(void);
void blink(BYTE cnt);
void wait(void);
void waitlong(void);


#define IOSIZE 70
BYTE iobuf[IOSIZE]; // used for Rx and Tx
BYTE flashbuf[SPM_PAGESIZE]; // used for flash io
WORD Lastcap;
BYTE Capcnt=0;
BYTE Errors=0;
BYTE Learnbut=0; // flag, if set, we pressed the "learnbutton"
BYTE Gotcode=0; // flag, if set, we received a valid ir code in CS.
BYTE Page; // flashpage of data in flashbuf, set by findcode()
BYTE Debug; // if set, toggles the LED each time a vilad code is received.

// SPM_PAGESIZE must be divisable by the size of this struct!
struct ircode
{
    ULONG comparecode; // the code we received from remote control or 0xffffffff as end of table. Or 0 for invalid/followon code,
    ULONG sendcode;  // the code we send out as translation, the spec follows below
    BYTE sync1;		// synclength1. we always assume to have sync1 and sync2
    BYTE sync2;		// synclength2
    BYTE stoplen; // if 0, no stoplen
    BYTE timshort; //puls duration 0 Bit. timshort + timshort = 0
    BYTE timlong;  //puls duration 1 Bit. timshort + timlong = 1
    BYTE coding;   //1Bit-coding scheme: 0=short/long;1=long/short
    BYTE bits; // number of Bits in code to transmit
    BYTE next; // followon code. if 0xAA, then the next record in the table will be send also.(fe. to power multible devices on/off).
} CS, *PCS; // PCS global pointer to current record in flashbuf, set by findcode(). CS is used for reception.


int main(void)
{

    // INIT:

    // After Reset, all port-pins are input/tri-state. Output Data is all 0. watchdog is disabled.
    // To enable pullup of an input, write 1 to port-data register-bit.
    // Port Registers B,C,D:PORTB=Data; DDRB=direction(1=output); PINB=Bittoggle on write 1.

    // set 8MHz with internal RC oscillator. Note: default after Reset normally is 1Mhz!
    CLKPR = _BV(CLKPCE); // enable clock prescale change
    CLKPR = 0; // set 8 MHz

    // status LED output on PD2
    bset(2,DDRD);

    // IR LED output on PD6
    bset(6,DDRD);

    // IR Receiver digital input on PB0
    bclr(0,DDRB);
    bset(0,PORTD); // enable pullup

    // Learn-Button input on PD3 INT1
    bclr(3,DDRD);
    bset(3,PORTD); // enable pullup

    // init serial:
    UBRR0H = 0;
    UBRR0L = 103; // 12=9600 Baud at 1mhz; 103=9600 Baud at 8mhz
    UCSR0A = 0x02; //double speed
    UCSR0B = 0x18; // RxTx enable
    UCSR0C = 0x06; // 8N1


    // TIMER 0: (8Bit)
    /* Timer0 is the 38kHz oscillator on output OC0A Pin PD6, the transmit output to IR LED.
    timer0 runs in CTC mode (Clear Timer on Compare) and toggle OC0A Pin.
    Thus the value of the compare register determines the frequency of the symetric squarewave output.
    Note: the CTC togglepin function introduces another divide by 2. So the formular for the comparevalue is:
    Fcpu/(divider*frequency*2) or (8Mhz/(1*38000*2)
    The oscillator is outputed on Pin PD6 only, if COM0A0 in TCCR0A is set to 1, else it is isolated from PD6
    and the general portpin output data is active (see below, its 0).
    So: COM0A0 1 = Mark(38khz), COM0A0 = 0, Space (0)
    */
    TCCR0A = 0x02; // CTC-mode
    TCCR0B = 0x01; // clocksource = systemclock
    OCR0A = 106;  // set output compare register to desired frequency!!! 8Mhz/(1*2*38000)=105.

    bset(6,DDRD); // set portpin pd6 to output. Thats the OC0A frequency 38KHZ. control with COM0A0 in TCCR0A
    bclr(6,PORTD); // clear output for Spacelevel 0 = LED off.

    // TIMER1: (16Bit)
    set_receiver();

    //Timer2(8Bit) Timeout generator for receive.
    TCCR2A = 0; // normal up counter
    TCCR2B = 0x07; // input clock is 8Mhz/1024. 128ns/tick. 7812 Hz. times out after 32ms at count 255.

    // learncode-button input INT1 enable
    EICRA=0x08; // falling edge
    bset(INT1,EIMSK);


    // power down not needed peripherals. disable on debugprint!
    //PRR = 0x87; // disable twi, SPI,UART,ADC

    sei(); // enable interrupts

    //set_sleep_mode( _BV(SM0) | _BV(SM1) ); // power save sleep mode(SM1 +SM0), just SM1 = powerdown.

    // set the sleep enable bit in SMCR. sleep mode is IDLE by default SMCR setting.
	// powerconsumption@8Mhz: ON=13mA; IDLE=8.5mA; PowerSave/PowerDown=1.8mA (only IR Receiver draws current!)
    sleep_enable();

    while(1)
    {
        // keep entering sleep mode
        sleep_cpu(); // execute the sleep instruction and goto IDLE sleep

        // an interrupt occured : T1capture or button int1
        //This is the only code that does not execute in interrupt!
        if (Learnbut) learncode();
        Learnbut=0;
    }
}




/* Timer1-Rx  is used to receive incoming IR codes.
Anytime the input pin toggles, a capture interrupt is generated and the current timer value
is written into the capture register ICR1.
We use it to measure the time in clocks of each pulse or pause.
So we can record the complete incoming waveform as a stream of samples. logic analyzer mode.
Samples are written in the capture interrupt routine. EOT is detected by a Timer2 overflow interrupt.
Note: the sfh8110 inverts the signal, so "no transmission" will give a 1. 38khz-on will result in a 0.
So first capture is done at the first falling edge, starting with the sync pulse.

Measurement-Range: Sync-pulse can be max 10msecs, the shortest puls is about 0.5msecs.
We take 1Mhz Timer1 clock which gives capturevalues in 1us increments, so max 65535 us = 65ms.
To convert to a Bytevalue we divide it by 40 to arrive at range 250(10ms) to 12 (0.5ms)
Timing:
Code packets are about 25 to 70ms long, Repeatcodes are send at about 100 to 150ms interval.

Valid frames are received if:
- Frequency is 38Khz
- one snyc cycle at start detected
- Bit duration less than 10.2ms (We cannot detect in between sync bits which messup the reception as timing is averaged)
- Bit duration > 0
- Puls- or Pause- Duration Modulation detected
- min 10 Bits received
- max 32 Bits received
- optional Stop Puls detected

*/

// set timer1 to receiver mode from ICF1 pin:
void set_receiver(void)
{
    bclr(COM0A0,TCCR0A); // turn off 38khz
    Capcnt=Errors=0;

    TCCR1A = 0; // normal 16bit mode up counter.
    TCCR1B = 0x82; // noice-canceller, falling-edge, clocksource = systemclock/8
    TIFR1  = 0xff; // clear all int flags
    TIMSK1 = 0x20; // enable capture interrupt

}

/* Timer1-Tx is used for transmission. playback recorded IR codes.
It receives the duration of each pulse or pause from the recorded/constructed sample stream in iobuf.
A stream has a maximum of 67 Byte-duration-samples. End-indicator=0 or max 67 items, ->  32 databits + syncs +stop
It is assumed, that either pulse duration or pause duration is used. ie:NEC,....others to get a hexvalue to compare with.
The first sample defines the active sync-pulse.
Next sample defines the following pause....and so on. The last could be a stoppulse.
The timer1 is started in CTC mode with the first duration programmed.(in timer1 OVL-int routine)
If timer1 reaches the duration value, a compare int is generated.
In its int-routine, the transmitter output is inverted (Mark/Space) and the next duration is loaded.
*/
// set timer1 to transmitter mode and start transmission of data in iobuf.
void set_transmitter(void)
{
    bclr(COM0A0,TCCR0A); // turn off 38khz
    Capcnt=0;
    TCCR1A = 0; // 16bit mode up counter, CTC.
    TCCR1B = 0x0A; // CTC, clocksource = systemclock/8
    TCNT1=0;
    OCR1A=0xffff; // start Tx-cycle after 66ms on TIMER1_COMPA_vect interrupt. (actually we need 100ms between frames)
    TIFR1  = 0xff; // clear all int flags
    TIMSK1 = 0x02; // enable OCIE1A compare match interrupt
}




/* when the "learnbutton" is pressed we enter here.

A remotecontrol code S1 maybe translated into 1,2 or 3 other ir codes D1,D2 or D3

Menue:
Blink 1 = learn a single code:  S1 -> D1  (update possible)
Blink 2 = learn a 2 followon codes, like power for 2 devices: S1 ->D1 ->D2     (only append possible else error)
Blink 3 = learn a 3 followon codes, like power for 3 devices: S1 ->D1 ->D2->D3 (only append possible else error)
Blink 4 = erase flashtable, press 2 different IRcodes to activate. (all codes are erased!!)
-- set Debug LED toggle modes: (Disable by PowerOff)
Blink 5 = LED is toggled each time a valid code was reveied. to test if IR is 38khz and receivable.
Blink 6 = same as 5, but toggles LED only on code compare match. find same code on different controls, test code recognize.		  
Blink 7 = toggle LED on 32bit code received. to look for modern codes like NEC!
Blink 8 = toggle LED on 16bit code received. 

Menue selection:
The LED will blink the number of times the "learnbutton" was pressed ie. shows menue item.
to enter that menue item, press any key on the remote control
-
Code entering:
- blink 1 time = prompt to press the remotecontrol code S
- blink 2 time = prompt to press the replacementcode D1
if menue 2 
- blink 3 time = prompt to press the replacementcode D2
if menue 3
- blink 4 time = prompt to press the replacementcode D3

on error (flash table full or entered S-code in D1 or D2 or D3)
	- blink 10 times  return error

- blink 1  = OK
return success

If you pressed the button more than 4 times just press any remote key to exit.
DONT press remote keys too fast, this may corrupt the data entry.

This routine may NOT be called from interrupt!!
returns: 0=OK; 1=error
*/
BYTE learncode(void)
{
    ULONG codeS;
    BYTE i,j,menue;

    // Blink Menue. press the "learnbutton" so many times as desired menueitem. then press any key on remote.
    Gotcode=0;
    while (!Gotcode) blink(Learnbut>>1); // wait for a remotecode to acknowledge menue selection
    menue=Learnbut>>1; // divide by 2, each press causes 2 triggers
	
//Debug functions:	
	
	if (menue == 5) // any valid code
	{
		Debug = 1;
		goto retok;
	}

	if (menue == 6) //compare match
	{
		Debug = 2;
		goto retok;
	}
	
	if (menue == 7) // 32bit codes
	{
		Debug = 3;
		goto retok;
	}
	
	if (menue == 8) //16 Bit codes
	{
		Debug = 4;
		goto retok;
	}
	
	if (menue > 8) goto reterr; // invalid menue item.
	
// Code functions:
	
    codeS=CS.sendcode;
	
	
    if (menue==4) // erase flashtable
    {
		Gotcode=0;
        while (!Gotcode) blink(4);// press different remote key as before to acknowledge erase operation.
        if (codeS==CS.sendcode) goto reterr;; //you pressed the same key so abort.
		cli();
        for (i=MINPAGE; i<=MAXPAGE; i++)
        {
            boot_page_erase ((ULONG)(SPM_PAGESIZE*i)); // erase the destination page
            boot_spm_busy_wait ();      // Wait until page is erased.
        }
		sei();
        goto retok;
    }

    // enter  the remote control code S1
    Gotcode=0;
    while (!Gotcode) blink(1); // wait for first remotecode S
    codeS=CS.sendcode;

    for (j=0; j<menue; j++) // enterloop for replacement codes and storage
    {
        // enter  the replacement code  D
        Gotcode=0;
        while (!Gotcode) blink(j+2); // wait for replacement code D

        if (codeS==CS.sendcode) goto reterr; //you entered codeS again, that makes no sense

		//setup the new record
        if (!j) CS.comparecode = codeS; // set the comparecode to S for first entry
        else CS.comparecode = 0; // multicode indication
		if ((menue==1)||((j+1)==menue)) CS.next=0;
		else CS.next=0xAA; // multicode indicator

        // now we have the complete translatecode in CS, lets store it in flash


        // find a possible existing entry for S1 in flashpage for update or find end of table
		switch (findcode())
		{
			case 0: // found existing entry of code asto CS.comparecode>0
			if (menue>1) goto reterr; //there was an existing entry for codeS, we cannot place multicodes for this. delete table first
			case 1: // found end of table with free space left.
			break;
			case 2: // the tablespace is full
			goto reterr;
			
		}


        memcpy(PCS,&CS,sizeof(struct ircode)); // copy new entry to flashbuf 
        flash_write_page(Page,flashbuf);

    } //endfor

retok:
    blink(1); // OK
    return 0;



reterr:
    blink(10);
    return 1;
}



/* find translation code im memory and copy to CS .
- compare CS.comparecode with the tableentrys-comparecode
	- on match copy it to CS and return success
	- if table.comparecode is 0xffffffff thats end of table, return error
		global var "Page" is set to flashpage of data in flashbuf for possible update,
		so we can use flashbuf to create new entry and save it under "Page".
returns: 
 - 0=OK found; flashbuf,Page and PCS valid, containing found record; record copied to CS
 - 1=not found but end of table has room for additional entry; flashbuf,Page and PCS valid, containing found record
 - 2=not found, the table is full. flashbuf,Page and PCS invalid == 0
 Page contains the flash pagenumber of last accessed page.
 flashbuf contains the pagedata of the last accessed page!
 PCS contains the "struct ircode" index of current record in flashbuf
 */
BYTE findcode(void)
{
    BYTE i,flag;


// find code in CS in the flash table
    flag=0;
    for (Page=MINPAGE; Page<=MAXPAGE; Page++)
    {
        PCS = (void*)flashbuf;
        flash_read_page (Page, flashbuf);
        for (i=0; i<(SPM_PAGESIZE / sizeof(struct ircode)); i++)
        {
            if (PCS->comparecode==0xffffffffL) 
			{
				return 1; // end of table found, there is room for another entry
			}
            if (PCS->comparecode && (PCS->comparecode == CS.comparecode)) // skip comparecode==0 
            {
                flag=1;
                break; // found it
            }
            PCS++;
        }
        if (flag) break;
    }

    if (!flag) 
	{
		PCS = 0;
		return 2; // we searched the whole table space and did not find the code or 0xffffffff, so table is full
	}

// PCS now points to the matching  ircode struct or next free entry

    return 0;
}




/*
Decode the received ircode in iobuf and fill the global CS struct so we can compare it with table entrys.
NOTE: a 1-bit can be double or trible the 0-bit length!
Modulation may have not a 50% duty cycle, so time-values for high/low may be different!
Most common the Puls-duration is longer asto achieve a stronger illumination ie.distance,
but its better to increase the Pulscycle of the 38khz oscillator to get better illumination power!!
We are workig on a 50% duty cyle so: Bittime0=(t1+t2)/2; Bittime 1=Bittime0/2 + ((t1+t2) - Bittime0/2)
returns 0=OK;1=error
*/
BYTE decodebuf(void)
{
    BYTE w1,w2;
    ULONG l=0; // assembled code
    BYTE i,b,flag;
    WORD al=0; //averge short
    WORD ah=0; //average long
    BYTE lc=0; //average counters
    BYTE hc=0;
    BYTE cod0=0; // detect coding order
    BYTE cod1=0;
    BYTE ret=0;
    BYTE *ps;


    ps=&iobuf[1]; // first sync. iobuf[0] is not used due to program flow!
    CS.sync1=*ps++;
    CS.sync2=*ps++;


// we always skip the first two periods as being sync!!
    for (i=3,b=0; i<IOSIZE; i+=2,b++)
    {
        flag=0;
        w1=*ps++; // get next 2 time value = 1 bit-cycle
        w2=*ps++;
		
        // end detection by value of 0
        if (!w1) // no stoplen
        {
            CS.stoplen=0;
            break;
        }
        if (!w2) // have stoplen
        {
            CS.stoplen=w1;
            break;
        }
		
        if (b==32) // we are after bit32 but not end of data-> error, unsupported longer code
        {
            ret++;
            break;
        }


        // if w1 > w2*2   or w2 > w1*2, its a 1
        if (w1 > (w2<<1))
        {
            flag=1; //its a 1 bit
            cod0++; //count coding long-short
        }
        else if (w2 > (w1<<1))
        {
            flag=1; //its a 1 bit
            cod1++; //count coding short-long
        }
        else // all lows, its a 0
        {
            al+=w1;  // add both values to calc average bit 0 duration-time
            al+=w2;
            lc+=1;   // inc 0-bit count

        }

        if (flag)
        {
            ah+=w2; // add both values to calc average bit 1 duration-time
            ah+=w1;
            hc+=1;  // inc 1-bit count
            l|=1L<<b;  //set 1-bit
        }

    } //endfor

    if (cod0&&cod1) ret++; // error, both codings are present
    if (cod0>1) CS.coding=0; // at least 2 1-Bit-codings must be present, to filter in between syns,if RC5.
    else if (cod1>1) CS.coding=1;
    else ret++; // error no pulslength coding detected

    CS.sendcode=CS.comparecode=l; // set both codes to the found one for findcode to work
    CS.bits = b;
	

    // calc bittimes based on averages
    al /=lc;  // divide total 0-Bit durations by the 0-Bit-count = average 0-Bit Time
    ah /=hc;  // divide total 1-Bit durations by the 1-Bit-count = average 1-Bit Time
    if (al&1) al++; // we need even values, round up, there are always truncations on divisions
    if (ah&1) ah++;
    al >>=1; // /2 = average Half-0-Bit-Time
    ah -= al; //(t1+t2)-time0/2. average 1-Bit-Time - average 0-Bit-Time/2 = average 1-Bit-Puls-Time

    CS.timshort=al;
    CS.timlong = ah;


    return ret;
}




/* Create the sample stream for Tx from PCS in iobuf. 
It expects that PCS points to the record to get the ir-data from
added multicode support.
*/
void setuptxbuf(void)
{
    BYTE *pd, i;
    ULONG l;
// prepare iobuf from PCS:
    pd=iobuf;
    l=PCS->sendcode;
    *pd++=PCS->sync1;
    *pd++=PCS->sync2;

    for (i=0; i<PCS->bits; i++)
    {
        if (l&1) // send a 1
        {
            if (PCS->coding) // short/long
            {
                *pd++=PCS->timshort;
                *pd++=PCS->timlong;
            }
            else  //long/short
            {
                *pd++=PCS->timlong;
                *pd++=PCS->timshort;
            }
        }
        else // send 0
        {
            *pd++=PCS->timshort;
            *pd++=PCS->timshort;
        }

        l>>=1;
    } // endfor

    if (PCS->stoplen) *pd++=PCS->stoplen;
    *pd=0; //EOT

// check for multi-records: set PCS to the next record if there or PCS=0
	if (PCS->next == 0xAA) 
	{
		PCS++;

		struct ircode *pcs = (void*)(flashbuf+SPM_PAGESIZE);
		if (PCS >= pcs)  // if PCS is at the last record of flashpage, read in the next flashpage
		{
			Page++;
	        flash_read_page (Page, flashbuf);
			PCS = (void*)flashbuf;
		}
	}
	else
		PCS = 0; //indicate there are now further records
}



// wait a little 400ms??
void wait(void)
{
    volatile WORD w=0;
    while (--w);
}

void waitlong(void)
{
    BYTE i;
    for (i=0; i<4; i++)
        wait();
}

// blink LED cnt times ,with pause at end
void blink(BYTE cnt)
{

    while (cnt--)
    {
        bset(2,PORTD);
        wait();
        bclr(2,PORTD);
        wait();
        wait();
    }
    waitlong();
}




/* IR-Receive:
A level-change (as programmed in the ICES1-Bit of register TCCR1B) has occured on input pin ICP1.
INT-flag is cleared automaticly by entering int-routine.
DO:
- toggle the edge select bit ICES1 in TCCR1B.
- clear timer2 overflow counter
If this was the first int (ie. start of reception):
- enable Timer2 overflow interrupt to detect EOT.
else
- calc and store the Timer1 Counter difference to get the duration (This works even if Timer1 overflowed!)
endif
- save the timer1-cap value (ICR1) for next occurence
- inc capture count=index to receive buffer.

Format of the receive buffer:
Byte0=Errorcounter. If there where receive errors or repeatframes, we discard the reception
Bytes1 to 80 = the time durations in the form: Marktime,Spacetime,Marktime,Spacetime,......stoplen-time?
Marktime=Mark(38khz-on)
Spacetime=Space (0,silence)
So there are always Mark/Space valuepairs.....except the last single value (if present) and is the EOT-indication
Receptions longer than 68 Bytes are discarded. 
*/
ISR(TIMER1_CAPT_vect)
{
    if (btst(ICES1,TCCR1B)) //toggle edge select CapInt
        bclr(ICES1,TCCR1B);
    else
        bset(ICES1,TCCR1B);

    TCNT2 = 135; // re-set Timer2 counter so it not overflows. 135=15ms

    WORD cnt = ICR1; // get capture value timer1
    WORD diff = cnt - Lastcap; // calc time difference. works even if T1 overflowed. uint16 math includes modulo


    if (!Capcnt) // if this is the first transition.ie. start of transmission
    {
        bset(TOV2,TIFR2); // clear Tim2 Overflow IntFlag
        bset(TOIE2,TIMSK2); // enable overflow interrupt Timer2

    }
    else
    {
        if (diff > 10200) //> 10.2 ms
        {
            Errors++; //duration longer than 10.2 ms.some protocols have upto 9.5ms , or have in between sync bits of about 4.5ms
        }
        diff /= 40; // convert to Bytevalue


        if (!diff) Errors++; // duration is 0

        if (Capcnt<(IOSIZE-1)) iobuf[Capcnt]= diff; // store duration in buffer
        else Errors++; // too long reception
    }

    Lastcap = cnt; // save the timer1-cap value (ICR1) for next interrupt
    Capcnt++; // inc cap counter

}



/* Timer1 compare_match interrupt:
IR code Transmitter:
The duration of a period has expired.

if the next duration in the table is not zero, and not IOSIZE items reached
- load it into the compare register
- set Tx State:
	- Capcnt even = MARK (38khz on) COM0A0 in TCCR0A =1
	- Capcnt odd  = Space (0)		COM0A0 in TCCR0A =0
else
- terminate transmission and reset to known state.
*/
ISR(TIMER1_COMPA_vect)
{
    WORD cnt = iobuf[Capcnt]; // get next timevalue or 0 as EOT

    if  (cnt) 
    {
        cnt *= 40; // revert the byte compression
        OCR1A = cnt; // set new period time
        if (!(Capcnt&1))
            bset(COM0A0,TCCR0A); // Mark 38khz
        else
            bclr(COM0A0,TCCR0A); //Space 0
        Capcnt++;
    }
    else if (PCS) // if there is another record to transmit
	{
		setuptxbuf();
		set_transmitter();
	}
	else
    {
        set_receiver(); // terminate transmission
    }
	
// you can optionally enlarge the transmission gap by calling set_transmitter() several times with global countdown variable!

}

/* Timer2 Receive Timeout generator = end of reception after 15ms no transition on ICT1.
Its clocked by 8Mhz/1024 so times out at TOP(255) after 15ms.
During each capture interrupt its count value is preset again.
At end of transmission (no more capture  ints) it will overflow and generate an ovl interrupt to end the reception here.

- decode received stream into CS
if notOK,
	- init receiver; return
else reception OK
	- find translate code in table
	if found
		- init transmitter; return;
	else goto NotOK;

NOTE: once in an interrupt routine, all other interrupts are disabled!
Interrupt priority defined by vector-number, so CapInt has higher prio than Overflow
*/
ISR(TIMER2_OVF_vect)
{
    bclr(TOIE2,TIMSK2); // disable overflow interrupt Timer2
    bclr(ICIE1,TIMSK1); // disable capture int
    iobuf[Capcnt]=0; // EOF, terminate receive buffer

    if (Capcnt < 20) Errors++; // repeat frame or invalid frame ie. less than 20 halfbits received

    if (!Errors&&!decodebuf())  // if valid frame was received...ie.valid data in CS
    {
		if (Debug==1) bset(2,PIND); // toggle LED on every code received

#ifdef DBPRINT
        printdb();
#endif

        if (!Learnbut) // dont translate in learnmode!
        {
			if ((Debug==3)&&(CS.bits==32)) bset(2,PIND); // toggle LED on 32bit code received
			if ((Debug==4)&&(CS.bits==16)) bset(2,PIND); // toggle LED on 32bit code received
	
			
            if (!findcode()) // if found translate code
            {
				if (Debug==2) bset(2,PIND); // toggle LED on code compare match

                setuptxbuf();
                set_transmitter();
                return;
            }
        }
        Gotcode++; // flag reception OK, valid code in CS. used for learncode
    }

    //reset capture system after timeout
    set_receiver();


}

// the "learncode" key was pressed
ISR(INT1_vect)
{

    waitlong(); // debounce time
    Learnbut++; // signal button pressed
//bset(INTF1,EIFR); // clear int flag if double set
}


/* write SPM_PAGESIZE Bytes to flash (one page!):

NOTE: The SELFPRGEN Fuse Bit must be programmed, otherwise you cannot write to flash !!!!
	  BLB0,1,2 Fuses may need to be set to "SPM no restriction" on some AVR processors.

The addressable flash region shall be after the code!! Set MAXPAGE,MINPAGE accordingly in define above.
entry:
- page = pagenumber from MINPAGE to MAXPAGE(inclusive)
- *buf = pointer to sourcebuffer(RAM) containing the SPM_PAGESIZE-Bytes to write.

The addressed page is erased, then SPM_PAGESIZE Bytes are written to the flash.

For Atmega48: flashsize=2048-words or 4096-Bytes. SPM_PAGESIZE is 32Words or 64Bytes, Number of pages=64 or 0 - 63
No special Bootsection is present, so SPM can be executed anywhere by default.
If Processor has a Boot-section, check BLB0,1,2 Fuses.
*/
void flash_write_page (uint32_t page, uint8_t *buf)
{
    uint16_t i;
    uint8_t sreg;

    if ((page > MAXPAGE) || (page < MINPAGE)) return; // page number out of range
    page *= SPM_PAGESIZE; // convert pagenumber to actual start-address of the page in flash !!!!

    sreg = SREG; // save status register for later restore (simulate an interrupt)
    cli(); // Disable interrupts.
    eeprom_busy_wait (); // wait possible eeprom action, this will corrupt flashing.

    boot_page_erase (page); // erase the destination page
    boot_spm_busy_wait ();      // Wait until page is erased.

    for (i=0; i<SPM_PAGESIZE; i+=2) // write your data to the special register-buffer,Z-pointer(this is NOT normal RAM!)
    {
        // the registerbuffer is WORD addressed and takes WORDs, so even amounts only.
        uint16_t w = *buf++;
        w += (*buf++) << 8; // little endian

        boot_page_fill (page + i, w); // write to register buffer one word
    }

    boot_page_write (page);     // flash the page from the registerbuffer.(internal algorithm)
    boot_spm_busy_wait();       // Wait until flashing is finished.

//   boot_rww_enable (); // Reenable RWW-section. only when RWW-section is present!!

    SREG = sreg; // restore the status register with possible int-enable flags
}

/* read SPM_PAGESIZE Bytes from flash(one page!):
entry:
- page = pagenumber from MINPAGE to MAXPAGE(inclusive)
- *buf = pointer to destinationbuffer(RAM) receiving the SPM_PAGESIZE-Bytes.

No Bootsection restrictions!

*/
void flash_read_page (uint32_t page, uint8_t *buf)
{

    uint8_t sreg;

    if ((page > MAXPAGE) || (page < MINPAGE)) return; // page number out of range
    page *= SPM_PAGESIZE;

    sreg = SREG; // save status register for later restore
    cli(); // Disable interrupts.

    //void * memcpy_P( void *dest, const void *src, size_t n )
    //The memcpy_P() function returns a pointer to dest or 0 on error
    memcpy_P(buf, (const void*)(uintptr_t)(page), SPM_PAGESIZE); // copy data from flash to buffer using the clib function in pgmspace.h

    SREG = sreg; // restore the status register with possible int-enable flags
}


#ifdef DBPRINT
// serial io: (uses a lot of codespace!)


void printdb(void)
{
    //debug
    sprintf(sbuf,"\n\nCapcnt:%u Code:0x%08lx s1:%u s2:%u ",Capcnt,CS.sendcode,CS.sync1,CS.sync2);
    putss(sbuf);
    sprintf(sbuf,"stoplen:%u timshort:%u timlong:%u cod:%u bits:%u\n",CS.stoplen,CS.timshort,CS.timlong,CS.coding,CS.bits);
    putss(sbuf);
    BYTE i;
    for (i=0; i<Capcnt; i++)
    {
        sprintf(sbuf," %u",iobuf[i]);
        putss(sbuf);
    }
}



void putcc(char c)
{
    while(!btst(UDRE0,UCSR0A)); // wait tx empty
    UDR0 = c;
}

void putss(char *ps)
{
    while (*ps) putcc(*ps++);
}


char getcc(void)
{
    if (btst(RXC0,UCSR0A))
        return(UDR0);
    else return(0);
}

#endif