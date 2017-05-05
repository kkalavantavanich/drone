#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Serial.h"
#include "Protocol.h"
#include "MultiWii.h"
#include "Alarms.h"

/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/


//RAW RC values will be store here
#if defined(SBUS)
  volatile uint16_t rcValue[RC_CHANS] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // interval [1000;2000]
#elif defined(SERIAL_SUM_PPM)
  volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000; 2000]
#else
  volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#endif

#if defined(SERIAL_SUM_PPM) //Channel order for PPM SUM RX Configs
  static uint8_t rcChannel[RC_CHANS] = {SERIAL_SUM_PPM};
#elif defined(SBUS) //Channel order for SBUS RX Configs
  // for 16 + 2 Channels SBUS. The 10 extra channels 8->17 are not used by MultiWii, but it should be easy to integrate them.
  static uint8_t rcChannel[RC_CHANS] = {SBUS};
#elif defined(SUMD)
  static uint8_t rcChannel[RC_CHANS] = {PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4};
#else // Standard Channel order
  static uint8_t rcChannel[RC_CHANS]  = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,AUX3PIN,AUX4PIN};
  static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS}; // if this slowes the PCINT readings we can switch to a define for each pcint bit
#endif

void rxInt(void);

/**************************************************************************************/
/***************                   RX Pin Setup                    ********************/
/**************************************************************************************/
void configureReceiver() {
  /******************    Configure each rc pin for PCINT    ***************************/
  #if defined(STANDARD_RX)
    #if defined(MEGA)
      DDRK = 0;  // defined PORTK as a digital port ([A8-A15] are considered as digital PINs and not analogical)
    #endif
    // PCINT activation
    for(uint8_t i = 0; i < PCINT_PIN_COUNT; i++){ // i think a for loop is ok for the init.
      PCINT_RX_PORT |= PCInt_RX_Pins[i];
      PCINT_RX_MASK |= PCInt_RX_Pins[i];
    }
    PCICR = PCIR_PORT_BIT;
    
    /*************    atmega328P's Specific Aux2 Pin Setup    *********************/
    #if defined(PROMINI)
     #if defined(RCAUXPIN)
        PCICR  |= (1 << 0) ; // PCINT activated also for PINS [D8-D13] on port B
        #if defined(RCAUXPIN8)
          PCMSK0 = (1 << 0);
        #endif
        #if defined(RCAUXPIN12)
          PCMSK0 = (1 << 4);
        #endif
      #endif
    #endif
    
    /***************   atmega32u4's Specific RX Pin Setup   **********************/
    #if defined(PROMICRO)
      //Trottle on pin 7
      DDRE &= ~(1 << 6); // pin 7 to input
      PORTE |= (1 << 6); // enable pullups
      EICRB |= (1 << ISC60);
      EIMSK |= (1 << INT6); // enable interuppt
      // Aux2 pin on PBO (D17/RXLED)
      #if defined(RCAUX2PIND17)
        DDRB &= ~(1 << 0); // set D17 to input 
      #endif
      // Aux2 pin on PD2 (RX0)
      #if defined(RCAUX2PINRXO)
        DDRD &= ~(1 << 2); // RX to input
        PORTD |= (1 << 2); // enable pullups
        EICRA |= (1 << ISC20);
        EIMSK |= (1 << INT2); // enable interuppt
      #endif
    #endif
    
  /*************************   Special RX Setup   ********************************/
  #endif
  #if defined(SERIAL_SUM_PPM)
    PPM_PIN_INTERRUPT; 
  #endif
  #if defined(SUMD)
    SerialOpen(RX_SERIAL_PORT,115200);
  #endif
  #if defined(SBUS)
    SerialOpen(RX_SERIAL_PORT,100000);
    switch (RX_SERIAL_PORT) { //parity
      #if defined(MEGA)
        case 0: UCSR0C |= (1<<UPM01)|(1<<USBS0); break;
        case 1: UCSR1C |= (1<<UPM11)|(1<<USBS1); break;
        case 2: UCSR2C |= (1<<UPM21)|(1<<USBS2); break;
        case 3: UCSR3C |= (1<<UPM31)|(1<<USBS3); break;
      #endif
    }
  #endif
}

/**************************************************************************************/
/***************               Standard RX Pins reading            ********************/
/**************************************************************************************/
#if defined(STANDARD_RX)

#if defined(FAILSAFE) && !defined(PROMICRO)
   // predefined PC pin block (thanks to lianj)  - Version with failsafe
  #define RX_PIN_CHECK(pin_pos, rc_value_pos)                        \
    if (mask & PCInt_RX_Pins[pin_pos]) {                             \
      if (!(pin & PCInt_RX_Pins[pin_pos])) {                         \
        dTime = cTime-edgeTime[pin_pos];                             \
        if (900<dTime && dTime<2200) {                               \
          rcValue[rc_value_pos] = dTime;                             \
          if((rc_value_pos==THROTTLEPIN || rc_value_pos==YAWPIN ||   \
              rc_value_pos==PITCHPIN || rc_value_pos==ROLLPIN)       \
              && dTime>FAILSAFE_DETECT_TRESHOLD)                     \
                GoodPulses |= (1<<rc_value_pos);                     \
        }                                                            \
      } else edgeTime[pin_pos] = cTime;                              \
    }
#else
  /** Checks whether pin has changed and sets cTime & dTime. Called in ISR(RX_PC_INT.).
   *  dTime = difference in time between 900 and 2200 us. Translated into rcValue.
   *  cTime = current time (us).
   *
   *            ______dTime______
   *  _________|                 |__________
   *         eTime             cTime
   */
  #define RX_PIN_CHECK(pin_pos, rc_value_pos)                    \
if (mask & PCInt_RX_Pins[pin_pos]) {                             \
  if (!(pin & PCInt_RX_Pins[pin_pos])) {                         \
	dTime = cTime - edgeTime[pin_pos];                           \
	if (900 < dTime && dTime < 2200) {                           \
	  rcValue[rc_value_pos] = dTime;                             \
	}                                                            \
  } else edgeTime[pin_pos] = cTime;                              \
}
#endif

  /** Port Change Interrupt. Called every time a change state occurs on an RX input pin. */
  ISR(RX_PC_INTERRUPT) { //this ISR is common to every receiver channel, it is call every time a change state occurs on a RX input pin
    uint8_t mask; // is 1 on pin that have changed
    uint8_t pin;
    uint16_t cTime,dTime; // cTime = actual currentTime, dTime = signal pulse length (differenceTime)
    static uint16_t edgeTime[8]; // time when it last changed
    static uint8_t PCintLast; // Pin Change Interrupt Last
  #if defined(FAILSAFE) && !defined(PROMICRO)
    static uint8_t GoodPulses;
  #endif
  
    pin = RX_PCINT_PIN_PORT; // RX_PCINT_PIN_PORT indicates the state of each PIN for the Arduino port dealing with Ports digital pins
   
    mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates which pin changed
    cTime = micros();         // micros() return a uint32_t, but it is not useful to keep the whole bits => we keep only 16 bits
    sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
    PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]
  
    #if (PCINT_PIN_COUNT > 0)
      RX_PIN_CHECK(0,2);
    #endif
    #if (PCINT_PIN_COUNT > 1)
      RX_PIN_CHECK(1,4);
    #endif
    #if (PCINT_PIN_COUNT > 2)
      RX_PIN_CHECK(2,5);
    #endif
    #if (PCINT_PIN_COUNT > 3)
      RX_PIN_CHECK(3,6);
    #endif
    #if (PCINT_PIN_COUNT > 4)
      RX_PIN_CHECK(4,7);
    #endif
    #if (PCINT_PIN_COUNT > 5)
      RX_PIN_CHECK(5,0);
    #endif
    #if (PCINT_PIN_COUNT > 6)
      RX_PIN_CHECK(6,1);
    #endif
    #if (PCINT_PIN_COUNT > 7)
      RX_PIN_CHECK(7,3);
    #endif
    
    #if defined(FAILSAFE) && !defined(PROMICRO)
      if (GoodPulses==(1<<THROTTLEPIN)+(1<<YAWPIN)+(1<<ROLLPIN)+(1<<PITCHPIN)) {  // If all main four chanells have good pulses, clear FailSafe counter
        GoodPulses = 0;
        if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0; 
      }
    #endif
  }
  /*********************      atmega328P's Aux2 Pins      *************************/
  #if defined(PROMINI)
    #if defined(RCAUXPIN)
    /* this ISR is a simplification of the previous one for PROMINI on port D
       it's simplier because we know the interruption deals only with one PIN:
       bit 0 of PORT B, ie Arduino PIN 8
       or bit 4 of PORTB, ie Arduino PIN 12
     => no need to check which PIN has changed */
    ISR(PCINT0_vect) {
      uint8_t pin;
      uint16_t cTime,dTime;
      static uint16_t edgeTime;
    
      pin = PINB;
      cTime = micros();
      sei();
      #if defined(RCAUXPIN8)
       if (!(pin & 1<<0)) {     //indicates if the bit 0 of the arduino port [B0-B7] is not at a high state (so that we match here only descending PPM pulse)
      #endif
      #if defined(RCAUXPIN12)
       if (!(pin & 1<<4)) {     //indicates if the bit 4 of the arduino port [B0-B7] is not at a high state (so that we match here only descending PPM pulse)
      #endif
        dTime = cTime-edgeTime; if (900<dTime && dTime<2200) rcValue[0] = dTime; // just a verification: the value must be in the range [1000;2000] + some margin
      } else edgeTime = cTime;    // if the bit 2 is at a high state (ascending PPM pulse), we memorize the time
    }
    #endif
  #endif
  
  /****************      atmega32u4's Throttle & Aux2 Pin      *******************/
  #if defined(PROMICRO)
    // throttle
    ISR(INT6_vect){ 
      static uint16_t now,diff;
      static uint16_t last = 0;
      now = micros();  
      if(!(PINE & (1<<6))){
        diff = now - last;
        if(900<diff && diff<2200){
          rcValue[3] = diff;
          #if defined(FAILSAFE)
           if(diff>FAILSAFE_DETECT_TRESHOLD) {        // if Throttle value is higher than FAILSAFE_DETECT_TRESHOLD
             if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;   // If pulse present on THROTTLE pin (independent from ardu version), clear FailSafe counter  - added by MIS
           }
          #endif 
        }
      }else last = now; 
    }
    // Aux 2
    #if defined(RCAUX2PINRXO)
      ISR(INT2_vect){
        static uint16_t now,diff;
        static uint16_t last = 0; 
        now = micros();  
        if(!(PIND & (1<<2))){
          diff = now - last;
          if(900<diff && diff<2200) rcValue[7] = diff;
        }else last = now;
      }
    #endif  
  #endif
#endif


/**************************************************************************************/
/***************                PPM SUM RX Pin reading             ********************/
/**************************************************************************************/
// attachInterrupt fix for promicro
#if defined(PROMICRO) && defined(SERIAL_SUM_PPM)
  ISR(INT6_vect){rxInt();}
#endif

// PPM_SUM at THROTTLE PIN on MEGA boards
#if defined(PPM_ON_THROTTLE) && defined(MEGA) && defined(SERIAL_SUM_PPM)
  ISR(PCINT2_vect) { if(PINK & (1<<0)) rxInt(); }
#endif

// Read PPM SUM RX Data
#if defined(SERIAL_SUM_PPM)
  void rxInt(void) {
    uint16_t now,diff;
    static uint16_t last = 0;
    static uint8_t chan = 0;
  #if defined(FAILSAFE)
    static uint8_t GoodPulses;
  #endif
  
    now = micros();
    sei();
    diff = now - last;
    last = now;
    if(diff>3000) chan = 0;
    else {
      if(900<diff && diff<2200 && chan<RC_CHANS ) {   //Only if the signal is between these values it is valid, otherwise the failsafe counter should move up
        rcValue[chan] = diff;
        #if defined(FAILSAFE)
          if(chan<4 && diff>FAILSAFE_DETECT_TRESHOLD) GoodPulses |= (1<<chan); // if signal is valid - mark channel as OK
          if(GoodPulses==0x0F) {                                               // If first four chanells have good pulses, clear FailSafe counter
            GoodPulses = 0;
            if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;
          }
        #endif
      }
    chan++;
  }
}
#endif

/**************************************************************************************/
/***************                   SBUS RX Data                    ********************/
/**************************************************************************************/
#if defined(SBUS)

#define SBUS_SYNCBYTE 0x0F // Not 100% sure: at the beginning of coding it was 0xF0 !!!
static uint16_t sbusIndex=0;
static uint16_t sbus[25]={0};

void readSerial_RX(){
  while(SerialAvailable(RX_SERIAL_PORT)){
    int val = SerialRead(RX_SERIAL_PORT);
    if(sbusIndex==0 && val != SBUS_SYNCBYTE)
      continue;
    sbus[sbusIndex++] = val;
    if(sbusIndex==25){
      sbusIndex=0;
      spekFrameFlags = 0x00;
      rcValue[0]  = ((sbus[1]|sbus[2]<< 8) & 0x07FF)/2+SBUS_MID_OFFSET;
      rcValue[1]  = ((sbus[2]>>3|sbus[3]<<5) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[2]  = ((sbus[3]>>6|sbus[4]<<2|sbus[5]<<10) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[3]  = ((sbus[5]>>1|sbus[6]<<7) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[4]  = ((sbus[6]>>4|sbus[7]<<4) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[5]  = ((sbus[7]>>7|sbus[8]<<1|sbus[9]<<9) & 0x07FF)/2+SBUS_MID_OFFSET;
      rcValue[6]  = ((sbus[9]>>2|sbus[10]<<6) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[7]  = ((sbus[10]>>5|sbus[11]<<3) & 0x07FF)/2+SBUS_MID_OFFSET; // & the other 8 + 2 channels if you need them
      //The following lines: If you need more than 8 channels, max 16 analog + 2 digital. Must comment the not needed channels!
      rcValue[8]  = ((sbus[12]|sbus[13]<< 8) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[9]  = ((sbus[13]>>3|sbus[14]<<5) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[10] = ((sbus[14]>>6|sbus[15]<<2|sbus[16]<<10) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[11] = ((sbus[16]>>1|sbus[17]<<7) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[12] = ((sbus[17]>>4|sbus[18]<<4) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[13] = ((sbus[18]>>7|sbus[19]<<1|sbus[20]<<9) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[14] = ((sbus[20]>>2|sbus[21]<<6) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[15] = ((sbus[21]>>5|sbus[22]<<3) & 0x07FF)/2+SBUS_MID_OFFSET; 
      // now the two Digital-Channels
      if ((sbus[23]) & 0x0001)       rcValue[16] = 2000; else rcValue[16] = 1000;
      if ((sbus[23] >> 1) & 0x0001)  rcValue[17] = 2000; else rcValue[17] = 1000;
      spekFrameDone = 0x01;

      // Failsafe: there is one Bit in the SBUS-protocol (Byte 25, Bit 4) whitch is the failsafe-indicator-bit
      #if defined(FAILSAFE)
      if (!((sbus[23] >> 3) & 0x0001))
        {if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;}   // clear FailSafe counter
      #endif

      // For some reason the SBUS data provides only about 75% of the actual RX output pulse width
      // Adjust the actual value by +/-25%.  Sign determined by pulse width above or below center
      uint8_t adj_index;
      for(adj_index=0; adj_index<16; adj_index++) {
        if (rcValue[adj_index] < MIDRC)
          rcValue[adj_index] -= (MIDRC - rcValue[adj_index]) >> 2;
        else
          rcValue[adj_index] += (rcValue[adj_index] - MIDRC) >> 2;
      }
    }
  }        
}
#endif

/**************************************************************************************/
/*************** SUMD ********************/
/**************************************************************************************/

#if defined(SUMD)
#define SUMD_SYNCBYTE 0xA8
#define SUMD_MAXCHAN 8
#define SUMD_BUFFSIZE SUMD_MAXCHAN*2 + 5 // 6 channels + 5 -> 17 bytes for 6 channels
static uint8_t sumdIndex=0;
static uint8_t sumdSize=0;
static uint8_t sumd[SUMD_BUFFSIZE]={0};

void readSerial_RX(void) {
  while (SerialAvailable(RX_SERIAL_PORT)) {
    int val = SerialRead(RX_SERIAL_PORT);
    if(sumdIndex == 0 && val != SUMD_SYNCBYTE) continue;
    if(sumdIndex == 2) sumdSize = val;
    if(sumdIndex < SUMD_BUFFSIZE) sumd[sumdIndex] = val;
    sumdIndex++;

    if(sumdIndex == sumdSize*2+5) {
      sumdIndex = 0;
      spekFrameFlags = 0x00;
      debug[1] = sumd[1];
      if (sumdSize > SUMD_MAXCHAN) sumdSize = SUMD_MAXCHAN;
      for (uint8_t b = 0; b < sumdSize; b++)
        rcValue[b] = ((sumd[2*b+3]<<8) | sumd[2*b+4])>>3;
      spekFrameDone = 0x01; // havent checked crc at all

      #if defined(FAILSAFE)
      if (sumd[1] == 0x01)
        {if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;} // clear FailSafe counter
      #endif
    }
  }
}
#endif

/**************************************************************************************/
/***************          combine and sort the RX Datas            ********************/
/**************************************************************************************/

uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  #if defined(SBUS) || defined(SUMD)
    if (chan < RC_CHANS) {
      data = rcValue[rcChannel[chan]];
    } else data = 1500;
  #else
    uint8_t oldSREG;
    oldSREG = SREG; cli(); // Let's disable interrupts
    data = rcValue[rcChannel[chan]]; // Let's copy the data Atomically
    SREG = oldSREG;        // Let's restore interrupt state
  #endif
  return data; // We return the value correctly copied when the IRQ's where disabled
}

/**************************************************************************************/
/***************          compute and Filter the RX data           ********************/
/**************************************************************************************/
#define AVERAGING_ARRAY_LENGTH 4
void computeRC() {
  static uint16_t rcData4Values[RC_CHANS][AVERAGING_ARRAY_LENGTH-1];
  uint16_t rcDataMean,rcDataTmp;
  static uint8_t rc4ValuesIndex = 0;
  /** Channel in {}.*/
  uint8_t chan;
  uint8_t a;
  uint8_t failsafeGoodCondition = 1;


	rc4ValuesIndex++;
	if (rc4ValuesIndex == AVERAGING_ARRAY_LENGTH-1) rc4ValuesIndex = 0;
	for (chan = 0; chan < RC_CHANS; chan++) {
	  rcDataTmp = readRawRC(chan);
	  #if defined(FAILSAFE)
		failsafeGoodCondition = rcDataTmp>FAILSAFE_DETECT_TRESHOLD || chan > 3 || !f.ARMED; // update controls channel only if pulse is above FAILSAFE_DETECT_TRESHOLD
	  #endif                                                                                // In disarmed state allow always update for easer configuration.
	  #if defined(SBUS) || defined(SUMD) // no averaging for SBUS & SUMD signal
		if(failsafeGoodCondition)  rcData[chan] = rcDataTmp;
	  #else
		if(failsafeGoodCondition) {
		  rcDataMean = rcDataTmp;
		  for (a=0;a<AVERAGING_ARRAY_LENGTH-1;a++) rcDataMean += rcData4Values[chan][a];
		  rcDataMean = (rcDataMean+(AVERAGING_ARRAY_LENGTH/2))/AVERAGING_ARRAY_LENGTH;
		  if ( rcDataMean < (uint16_t)rcData[chan] -3)  rcData[chan] = rcDataMean+2;
		  if ( rcDataMean > (uint16_t)rcData[chan] +3)  rcData[chan] = rcDataMean-2;
		  rcData4Values[chan][rc4ValuesIndex] = rcDataTmp;
		}
	  #endif
	  if (chan<8 && rcSerialCount > 0) { // rcData comes from MSP and overrides RX Data until rcSerialCount reaches 0
		rcSerialCount --;
		#if defined(FAILSAFE)
		  failsafeCnt = 0;
		#endif
		if (rcSerial[chan] >900) {rcData[chan] = rcSerial[chan];} // only relevant channels are overridden
	  }
	}
}
