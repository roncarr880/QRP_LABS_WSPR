
// QRP_LABS_WSPR
//   Arduino, QRP Labs Arduino shield, SI5351 clock, QRP Labs RX module, QRP Labs relay board.
//   NOTE:  The tx bias pot works in reverse, fully clockwise is off.
//   Added a CANADAUINO  WWVB interface to keep time.

//  common startup commands with WWVB logging
//  enter 1 CAT command, ?V for Rx only or #0 to stay in FRAME TX mode

//   The CAT emulation is TenTec Argonaut V at 1200 baud.

// New encoder switch commands:
//   Tap : change tuning step 10hz to 1Meg
//   DTap: toggle mode WSPR TX Frame to CAT control - RX mode
//   Long: change band

//   To set a new operation frequency for stand alone Frame Mode, restart the Arduino, start wsjt-x or HRD.
//   Tune to one of the magic WSPR frequencies and toggle TX (tune in wsjt will work).
//   The new frequency will be stored in EEPROM.
//   If using the band hopping feature of WSJT, make sure the first transmit is on the band you wish for
//   the default.  Only one save is performed per Arduino reset. 
//
//   A 4:1 frequency relationship between the tx freq and the rx clock is maintained using the
//   R dividers in the SI5351.  Dividers 1 - rx and 4 - tx will cover 1mhz to 30mhz
//   Dividers 16 - rx and 64 - tx will cover 40 khz to 2 mhz

#include <FreqCount.h>   // one UNO I have does not work correctly with this library, another one does
#include <EEPROM.h>
#include <OLED1306_Basic.h>

#define ROW0 0          // text based rows for the 128x64 OLED
#define ROW1 8
#define ROW2  16
#define ROW3  24
#define ROW4  32
#define ROW5  40
#define ROW6  48
#define ROW7  56



#define SI5351   0x60    // i2c address
#define PLLA 26          // register address offsets for PLL's
#define PLLB 34
#define CLK0_EN   1
#define CLK1_EN   2
#define CLK2_EN   4
// the starting frequency will be read out of EEPROM except the 1st time when EEPROM is blank
#define FREQ  7038600   // starting freq when EEPROM is blank
#define DIV   28        // starting divider for 4*freq*RDIV
#define RDIV   1        // starting sub divider.  1 will cover less than 1mhz to > 30mhz.
                        // 16 will cover 37khz to 2.3 mhz ( with the 4x factor it is 64 when transmitting )
#define CAT_MODE  0     // computer control of TX
#define FRAME_MODE 1    // self timed frame (stand alone mode)
#define MUTE  A1        // receiver module T/R switch pin
#define START_CLOCK_FREQ   2700449800LL   // *100 for setting fractional frequency  ( 446600 ) 
//#define START_CLOCK_FREQ   2700466600   // test too high
//#define START_CLOCK_FREQ   2700426600   // test too low

//#define CLK_UPDATE_THRESHOLD  59    // errors allowed per minute to consider valid sync to WWVB
#define CLK_UPDATE_THRESHOLD2 48

#define DEADBAND 25                 // wwvb signal timing +-deadband 

#define stage(c) Serial.write(c)

OLED1306 LCD;            // a modified version of LCD_BASIC by Rinky-Dink Electronics

extern unsigned char SmallFont[];
extern unsigned char MediumNumbers[];
extern unsigned char BigNumbers[];


int last_error_count = 60;
char val_print = ' ';


  // using even dividers between 6 and 254 for lower jitter
  // freq range 2 to 150 without using the post dividers
  // we are using the post dividers and can receive down to 40khz
  // vco 600 to 900
  
uint64_t clock_freq = START_CLOCK_FREQ;
uint64_t drift;                      // calibrate freq result used as a measure of temperature and mapped to correct
                                     // 27 master clock drift
uint32_t freq = FREQ;                // ssb vfo freq
const uint32_t cal_freq = 3000000;   // calibrate frequency
long cal_result;
const uint32_t cal_divider = 200;
uint32_t divider = DIV;
uint32_t audio_freq = 1466;          // wspr 1400 to 1600 offset from base vfo freq 
uint8_t  Rdiv = RDIV; 
uint8_t operate_mode = FRAME_MODE;     //FRAME_MODE  CAT_MODE // start in stand alone timing mode
uint8_t wspr_tx_enable;              // transmit enable
uint8_t wspr_tx_cancel;              // CAT control RX command cancels tx
uint8_t cal_enable;
long tm_correct_count = 60000;       // add or sub one ms for time correction per this many ms
int8_t tm_correction = 0;            // 0, 1 or -1 time correction
int8_t tm_correction2;               // frame sync to wwvb signal falling edge

//      Download WSPRcode.exe from  http://physics.princeton.edu/pulsar/K1JT/WSPRcode.exe   and run it in a dos window 
//      Type (for example):   WSPRcode "K1ABC FN33 37"    37 is 5 watts, 30 is 1 watt, 33 is 2 watts, 27 is 1/2 watt
//      ( Use capital letters in your call and locator when typing in the message string.  No extra spaces )
//      Using the editing features of the dos window, mark and copy the last group of numbers
//      Paste into notepad and replace all 3 with "3,"  all 2 with "2," all 1 with "1," all 0 with "0,"
//      Remove the comma on the end
//  the current message is   "K1URC FN54 23"
const char wspr_msg[] = { 
 3, 3, 2, 2, 2, 0, 0, 2, 1, 2, 0, 2, 1, 1, 1, 2, 2, 2, 3, 0, 0, 1, 0, 1, 1, 3, 3, 2, 2, 0,
 2, 2, 0, 0, 3, 2, 0, 3, 0, 1, 2, 0, 2, 0, 0, 2, 3, 0, 1, 3, 2, 0, 1, 1, 2, 1, 0, 2, 0, 3,
 3, 2, 3, 2, 2, 2, 2, 1, 3, 2, 3, 0, 3, 0, 3, 0, 3, 2, 0, 3, 2, 0, 3, 0, 3, 1, 0, 2, 0, 1,
 1, 2, 1, 2, 3, 0, 2, 2, 3, 2, 2, 0, 2, 2, 1, 0, 2, 1, 2, 0, 3, 3, 1, 2, 3, 1, 2, 2, 3, 1,
 2, 1, 2, 0, 0, 1, 1, 3, 2, 2, 2, 2, 0, 1, 0, 1, 2, 0, 3, 1, 0, 2, 0, 0, 2, 2, 0, 1, 3, 0,
 1, 2, 3, 1, 0, 2, 2, 1, 3, 0, 2, 2
 };


uint8_t band;            // current band
struct BAND {
   int pin;          // band relay switching
   uint32_t  low;    // low frequency limit
   uint32_t high;    // high frewquency limit
};
uint8_t wband = 3;           // wspr band switching via encoder long press
uint8_t sstate[1];           // switch state, one switch

//  relay board was jumpered to NOT have filter 1 always in line and antenna connects to the bnc 
//  on the arduino shield.  ( otherwise highest freq would need to be in position 1 and output would 
//  be from the relay board )
struct BAND band_info[6] = {    // filter selected
  {  7,   40000,   600000 },    // 630m
  { A0,  600000,  2500000 },    // 160m
  { 10, 2500000,  5000000 },    // 80m
  { 11, 5000000, 11500000 },    // 30m
  { 12,11500000, 20000000 },    // 17m
  { A3,20000000, 30000000 }     // 10m
};  

// wspr frequencies for eeprom save routine.  Only these frequencies will be saved.
const uint32_t magic_freq[10] = {
  474200, 1836600, 3568600, 7038600, 10138700, 14095600, 18104600, 21094600, 24924600, 28124600
};

#define WWVB_OUT 9
#define WWVB_PWDN 8      // was the low enable.  Rewired the WWVB receiver to get power from this I/O pin.
                         // this reverses the logic so it is now high to enable.  With only two wires in the
                         // cable, this seems like the best way to remove the need for the 9 volt battery
                         // that was powering the wwvb receiver and to avoid taking the WSPR unit apart to 
                         // switch the wiring to +5 volts instead of an I/O pin.
uint64_t wwvb_data, wwvb_sync, wwvb_errors;

uint8_t wwvb_quiet = 0;  // wwvb debug print flag, set to 1 for printing
                         // or enter 1 CAT command( ?V for Rx only or #0 to stay in FRAME mode with logging )
uint8_t wwvb_stats[8];   // bit distribution over 60 seconds
uint8_t wwvb_last_err;   // display last error character received ( will show what causes just one error )
uint8_t DST;             // daylight savings bit

uint8_t frame_sec;    // frame timer counts 0 to 120
int frame_msec;
// uint8_t tick;         // start each minute,  what was this for ? to match displayed time with computer time
                         // but disturbs the trending bit display

int FF = 5;        // fixed part of fudge factor for frequency counter result ( counting 3 mhz signal )
int ff = 0;        // fractional part of the fudge factor ( floats not useful as limited in significant figures )

uint8_t dbug_print_state;   // print messages at 1200 baud without blocking

// date, time keeping
int gmon = 1,gday = 1,gyr = 1,ghr,gmin;
int tot_days = 1;
uint16_t leap = 1;

// #define TK 4        // keep time has been run
#define TS 2        // time was set from WWVB decode
// #define TP 1        // print decode indicator
uint8_t time_flags; // WWVB encodes the previous minute, flags to print the correct time

uint8_t trends[60];
uint8_t clr_trends;
unsigned int decodes;
uint8_t report_i;      // see if a single trend shows the lsb of minutes position in time.

long int stp = 1000;

int debug_i;
/***************************************************************************/

void ee_save(){ 
uint8_t i;
static uint8_t last_i = 255;

  if( last_i != 255 ) return; // save only the freq of the 1st time transmitting after reset
                
  for( i = 0; i < 10; ++i ){
    if( freq == magic_freq[i] ) break;
  }
  if( i == 10 ) return;      // not a wspr frequency
  // if( i == last_i ) return;  // already wrote this one. ( redundant - only saving 1st power up tx freq now
                                // instead of the last transmit freq. This allows wsjt band hopping tx
                                // without wearing out the eeprom

  last_i = i;
  EEPROM.put(0,Rdiv);    // put does not write if data matches
  EEPROM.put(1,freq);    // and hopefully this will not take long when there is a match
  EEPROM.put(5,divider);
}

void ee_restore(){

  if( EEPROM[0] == 255 ) return;   // blank eeprom
  EEPROM.get(0,Rdiv);
  EEPROM.get(1,freq);
  EEPROM.get(5,divider);
  //Serial.println(Rdiv);
  //Serial.println(freq);
  //Serial.println(divider);
}

void setup() {
uint8_t i;
  
  Serial.begin(1200);      // TenTec Argo V baud rate 1200
  i2init();

  ee_restore();            // get default freq for frame mode from eeprom

  pinMode(MUTE,OUTPUT);    // receiver t/r switch
  digitalWrite(MUTE,LOW);  // enable the receiver
  
  pinMode(WWVB_OUT, INPUT);     // sample wwvb receiver signal
  pinMode(WWVB_PWDN, OUTPUT);
  // digitalWrite(WWVB_PWDN,LOW);  // enable wwvb receiver
  digitalWrite(WWVB_PWDN,HIGH);    // power the wwvb receiver with the I/O pin

  // set up the relay pins, exercise the relays, delay is 1.2 seconds, so reset at 59 seconds odd minute to be on time
  for( i = 0; i < 6; ++i ){
    pinMode(band_info[i].pin,OUTPUT);
    digitalWrite( band_info[i].pin,LOW );
    delay(200);
    digitalWrite( band_info[i].pin,HIGH );
  }

  i2cd(SI5351,16,0x4f);   // clock 0, PLLA
  i2cd(SI5351,17,0x4f);   // clock 1, PLLA
  i2cd(SI5351,18,0x6f);   // clock 2, PLLB

  // set some divider registers that will never change
  for(i = 0; i < 3; ++i ){
    i2cd(SI5351,42+8*i,0);
    i2cd(SI5351,43+8*i,1);
    i2cd(SI5351,47+8*i,0);
    i2cd(SI5351,48+8*i,0);
    i2cd(SI5351,49+8*i,0);
  }

  si_pll_x(PLLB,cal_freq,cal_divider,0);   // calibrate frequency on clock 2
  si_load_divider(cal_divider,2,0,1);

  si_pll_x(PLLA,Rdiv*4*freq,divider,0); // receiver 4x clock
  si_load_divider(divider,0,0,Rdiv*4);  // TX clock 1/4th of the RX clock
  si_load_divider(divider,1,1,Rdiv);    // load divider for clock 1 and reset pll's
  
  i2cd(SI5351,3,0xff ^ (CLK1_EN + CLK2_EN) );   // turn on clocks, receiver and calibrate
  //  i2cd(SI5351,3,0xff ^ (CLK0_EN + CLK1_EN + CLK2_EN));   // testing only all on, remove tx PWR

  digitalWrite(band_info[band].pin,LOW);   // in case this turns out to be the correct relay
  band_change();  // select the correct relay
  //Serial.println(F("Starting..."));
  pinMode(13,OUTPUT);

  LCD.InitLCD();                        // using a modified Nokia library for the OLED
  LCD.setFont(SmallFont);
  LCD.clrScr();
  LCD.print("QRP-LABS WSPR SDR",0,ROW0);
  i2flush();
  delay(2000);
 // LCD.clrRow(0);   // ? clear to end from current position 
  LCD.clrScr();
  
  freq_display();
  mode_display();
}

int8_t encoder(){   /* read encoder, return 1, 0, or -1 */
  
static int8_t mod;     /* encoder is divided by 4 because it has detents */
static int8_t dir;     /* need same direction as last time, effective debounce */
static int8_t last;    /* save the previous reading */
int8_t new_;     /* this reading */
int8_t b;

   new_ = (PIND >> 2) & 3;  
   if( new_ == last ) return 0;       /* no change */

   b = ( (last << 1) ^ new_ ) & 2;  /* direction 2 or 0 from xor of last shifted and new data */
   last = new_;
   if( b != dir ){
      dir = b;
      return 0;      /* require two in the same direction serves as debounce */
   }
   mod = (mod + 1) & 3;       /* divide by 4 for encoder with detents */
   if( mod != 2 ) return 0;

   if( dir == 2 ) return 1;   /* swap return values if it works backwards */
   else return -1;
}

  /* switch states */
#define IDLE_ 0
#define ARM  1
#define DARM 2
#define DONE 3
#define TAP  4
#define DTAP 5
#define LONGP 6
      /* run the switch state machine, generic code for multiple switches even though have only one here */
int8_t switches(){
static uint8_t press_, nopress;
static uint32_t tm;
int  i,j;
int8_t sw;
int8_t s;

   if( tm == millis() ) return 0;      // run once per millisecond
   tm = millis();
   
   /* get the switch readings, low active but invert bits */
   sw = ((PIND & 0x10) >> 4) ^ 0x01;                 
   
   if( sw ) ++press_, nopress = 0;       /* only acting on one switch at a time */
   else ++nopress, press_ = 0;           /* so these simple vars work for all of them */

   /* run the state machine for all switches in a loop */
   for( i = 0, j = 1; i < 1; ++i ){
      s = sstate[i];
      switch(s){
         case DONE:  if( nopress >= 100 ) s = IDLE_;  break;
         case IDLE_:  if( ( j & sw ) && press_ >= 30 ) s = ARM;  break; /* pressed */
         case ARM:
            if( nopress >= 30 ) s = DARM;                      /* it will be a tap or double tap */
            if( press_ >= 240 ) s = LONGP;                     // long press
         break;
         case DARM:
            if( nopress >= 240 )  s = TAP;
            if( press_ >= 30 )    s = DTAP;
         break;
      }      
      sstate[i] = s; 
      j <<= 1;
   }
   
   return sstate[0];      // only one switch implemented so can return its value
}


uint8_t  band_change(){

   if( freq > band_info[band].low && freq <= band_info[band].high ) return 0;

   // band change needed
   digitalWrite(band_info[band].pin,HIGH);
   for( band = 0; band < 6; ++band ){
      if( freq > band_info[band].low && freq <= band_info[band].high ) break;
   }
   if( band == 6 ) band = 5;   // default band
   digitalWrite( band_info[band].pin,LOW);
   return 1;
}

void qsy(uint32_t new_freq){         // change frequency
unsigned char divf;
uint32_t f4;
static uint32_t old_freq = 0;

   divf = 0;   // flag if we need to reset the PLL's
   if( (abs((int32_t)old_freq - (int32_t)new_freq) > 500000)  || old_freq == 0){
       divf = 1;    // large qsy from our current dividers
       old_freq = new_freq;
   }
   freq = new_freq;
   if( band_change() ) divf = 1;    // check the proper relay is selected

   // force freq above a lower limit
   if( freq < 40000 ) freq = 40000;
   
   if( freq > 2000000 && Rdiv != 1 ) Rdiv = 1, divf = 1;     // tx R is 4
   if( freq < 1000000 && Rdiv != 16 ) Rdiv = 16, divf = 1;   // tx R is 64
   f4 = Rdiv * 4 * freq;
   f4 = f4 / 100000;       // divide by zero next line if go below 100k on 4x vfo

   if( divf ) divider = 7500 / f4;      // else we are using the current divider
   if( divider & 1) divider += 1;       // make it even
   
   if( divider > 254 ) divider = 254;
   if( divider < 6 ) divider = 6;

   // setup the PLL and dividers if needed
  si_pll_x(PLLA,Rdiv*4*freq,divider,0);
  if( divf ){
      si_load_divider(divider,0,0,Rdiv*4);   // tx at 1/4 the rx freq
      si_load_divider(divider,1,1,Rdiv);  // load rx divider and reset PLL
  }

  freq_display();
  
}

void loop() {
static unsigned long ms;
static int temp;            // just for flashing the LED when there is I2C activity. Check for I2C hangup.
int8_t t;

   if( Serial.availableForWrite() > 20 ) radio_control();
   temp += i2poll();
   
   if( ms != millis()){     // run once each ms
       ms = millis();
       frame_timer(ms);

       if( wspr_tx_enable || wspr_tx_cancel ) wspr_tx(ms);
       if( cal_enable ) run_cal();
       //wwvb_sample(ms);
       wwvb_sample2(ms);
       if( temp  ){
         if( temp > 100 ) temp = 100;
         --temp;
         if( temp ) digitalWrite(13,HIGH);
         else       digitalWrite(13,LOW);   
       }

       // print out debug messages without waiting for the serial ready
       if( wwvb_quiet == 1 && dbug_print_state && Serial.availableForWrite() > 20) dbug_errors( 0, 0, 0, 0, 0 );

       t = encoder();
       if( t ){
          qsy( freq + t * stp );
       }

       t = switches();
       if( t > DONE ){
           switch(t){
              case TAP:
                 stp /= 10;
                 if( stp == 1 ) stp = 1000000;   
              break;
              case DTAP:
                 operate_mode ^= 1;
              break;
              case LONGP:
                 if(++wband > 9 ) wband = 0;
                 qsy(magic_freq[wband]);
              break;
           }
           sstate[0] = DONE;
           mode_display();
       }
   }

}

void calc_date(){    // from total days and leap flag
const int cal[2][12] =  
   { 31,28,31,30,31,30,31,31,30,31,30,31,
     31,29,31,30,31,30,31,31,30,31,30,31 };
int i,d;

   d = tot_days;
   for( i = 0; i < 12; ++i ){
      if( d <= cal[leap][i] ) break;
      d -= cal[leap][i];
   }
     
   gmon = i + 1;
   gday = d;
  
}

void wwvb_decode(){   // WWVB transmits the data for the previous minute just ended
uint16_t tmp;
uint16_t tmp2;
uint16_t yr;
uint16_t hr;
uint16_t mn;
uint16_t dy;
uint8_t i;


  tmp2 = frame_sec;
  tmp = frame_msec;         // capture milliseconds value before it is corrected so we can print it.

  ++decodes;

  yr = wwvb_decode2( 53, 0x1ff );   // year is 0 to 99
  dy = wwvb_decode2( 33, 0xfff );   // day is 0 to 365/366
  hr = wwvb_decode2( 18, 0x7f );
  mn = wwvb_decode2( 8, 0xff );
  leap = wwvb_decode2( 55, 0x1 );
  DST  = wwvb_decode2( 57, 0x1 );    // in effect bit ( using bit 58 gave wrong time for one day )

  if( ( mn & 1 ) == 0 ){    //last minute was even so just hit the 60 second mark in the frame
                            // only apply clock corrections in the middle of the two minute frame or may
                            // otherwise mess up the frame timing
         if( frame_sec == 59 && frame_msec >= 500 ) ;   // ok
         else if( frame_sec == 60 && frame_msec < 500 ) ;  // ok
         else{                                              // way off, reset to the correct time
            frame_sec = 60;
            frame_msec = 0;  
            FF = 3, ff = 0;             // reset timing fudge factor
            clr_trends = 1;             // the trend buckets will be incorrect now
         }
  }
  
  if( wwvb_quiet == 1 ){    // wwvb logging mode
    // Serial.print(decodes);
    // Serial.write(' ');  Serial.print(tmp2); Serial.write('.'); Serial.print(tmp);   // show jitter
    // Serial.print(" WWVB "); 
    // Serial.print("20");                        // the year 2100 bug
    // Serial.print( yr );  Serial.write(' ');
    // Serial.print( dy );  Serial.write(' ');
    // Serial.print( hr );  Serial.write(':');
    // if( mn < 10 ) Serial.write('0');
    // Serial.println(mn);
  }
    
  ghr = hr;
  gmin = mn;
  gyr = yr;
  tot_days = dy;
  keep_time();            // wwvb sends minute just ended info, so increment
  calc_date( );
  time_flags |= TS;

}

// wwvb fields decode about the same way
uint16_t wwvb_decode2( uint8_t pos, uint16_t mask ){ 
uint16_t tmp;
uint16_t val;

  tmp = ( wwvb_data >> ( 59 - pos ) ) & mask;
  val = 0;
  if( tmp & 0x800 ) val += 200;
  if( tmp & 0x400 ) val += 100;
  if( tmp & 0x100 ) val += 80;
  if( tmp & 0x80 ) val += 40;
  if( tmp & 0x40 ) val += 20;
  if( tmp & 0x20 ) val += 10;
  val += (tmp & 0xf);

  return val;
  
}

// the original idea was to correct the 27 mhz clock using the UNO 16 mhz clock as a reference.
// calibrating the SI5351 against the 16mhz clock does not seem to be viable.
// the 16mhz clock varies as much or more than the 27mhz clock with changes in temperature
// this function has been changed to correct the time keeping of the 16 meg clock based upon the 27 mhz reference
// this seems to be working very well with no change in WSPR received delta time for over 48 hours.
void run_cal(){    // count pulses on clock 2 wired to pin 5
                   // IMPORTANT: jumper W4 to W7 on the arduino shield
long error1,error2;


   if( cal_enable == 1 ){
       FreqCount.begin(1000);   //
       ++cal_enable;
   }

   if( FreqCount.available() ){
       cal_result = (long)FreqCount.read() + (long)FF;
       tm_correction = 0;                   // default
       if( cal_result > 2999996L && cal_result < 3000004 )tm_correction = 0, tm_correct_count = 30000;
       else{
          if( cal_result < 3000000L ){
            tm_correction = -1, error1 = 3000000L - cal_result;
            error2 = error1 - 1;
          }
          else if( cal_result > 3000000L ){
            tm_correction =  1, error1 = cal_result - 3000000L;
            error2 = error1 + 1;
          }

          if( error1 != 0 ) error1 = 3000000L / error1;
          if( error2 != 0 ) error2 = 3000000L / error2;

          tm_correct_count = interpolate( error1,error2,ff );
       }
                
       if( wwvb_quiet == 1 && tm_correction == 0 ){    // wwvb logging mode
        // Serial.print( result );   Serial.write(' ');
        //  Serial.print( error );    Serial.write(' ');
        //  Serial.print(tm_correction);  Serial.write(' ');
        //  Serial.print(tm_correct_count);  Serial.write(' ');
       }
       FreqCount.end();
       cal_enable = 0;
       temp_correction();
   }  
}


// amt is a value from 0 to 99 representing the percentage
long interpolate( long val1, long val2, int amt ){
long diff;
long result;

    diff = val2 - val1;
    result = (diff * amt) / 100;
    result += val1;
    // Serial.print( val1 );  Serial.write(' ');  Serial.print( val2 ); Serial.write(' ');
    return result;
}

void clock_correction( int8_t val ){    // time keeping only, change the fudge factor

   ff += val;                           // val is always -1,0,or 1
   if( ff >= 100 ) ff = 0, ++FF;        // keep ff in 0 to 99 range
   if( ff < 0 ) ff = 99, --FF;

}

void temp_correction( ){    // short term drift correction with a linear map
uint64_t local_drift;       // corrects for drift due to day/night temperature changes

    if( wspr_tx_enable ) return;                                // ignore this when transmitting

    local_drift = map( cal_result, 2999900, 3000000, 2000, 0 );

    if( local_drift != drift ){
       drift = local_drift;
       si_pll_x(PLLB,cal_freq,cal_divider,0);
       si_pll_x(PLLA,Rdiv*4*freq,divider,0);    // new receiver 4x clock, transmitter 1x clock
    }
}


// correct errors on bits that do not change often
// this version moves the counters up to a hard limit and moves down on different bit decoded
//  decodes from trends only for 1 missed bit
// slips in time on weak signal as is called from frame_sync
char wwvb_trends( char val, uint8_t dat ){
static int i = 60;
static int unslip;
uint8_t count;
uint8_t trend_t,new_t;
int w;
static int s_count;     // counts from last sync - detect when spaced 9 apart. Most are 10 apart.

#define LIMIT 6         // 2 min 30 max
#define SYNC 32
#define ONES 64
#define ZEROS 128
#define ERR  0

    if( clr_trends ){     // reset the data 
       for( w = 0; w < 60; ++w ) trends[w] = 0;
       clr_trends = 0;
    }
    
    if( unslip ) unslip = 0;
    else if( ++i >= 60 ){
       i = 0;
       // the sample bucket ends at 1 sec past the time sampled. Test once per two minute frame
       if( frame_sec < 60 && (frame_msec < 400 || frame_msec > 600) ){
           if( frame_sec == 0 && frame_msec > 600 ) ;       // ok, just early
           else if( frame_sec == 1 && frame_msec < 400 ) ;  // also ok
           else if( frame_sec < 30 && frame_sec > 0 ) ++i, report_i = 0;  // running slow is normal on weak signals
           else unslip = 1, report_i = 0;;                                // probably a time reset or decode happened
       }
    }

   
       count = trends[i] & 31;                             // get existing trend data
       trend_t = trends[i] & ( ONES | SYNC | ZEROS );
       
       new_t = ERR;                                        // assume error
       if( val == 'S' ) new_t = SYNC;
       if( val == '1' ) new_t = ONES;
       if( val == '0' ) new_t = ZEROS;

       ++s_count;                   // count from the last sync.  Attempt early sync to 9 syncs spacing.
       if( new_t == SYNC ){
            if( s_count == 9 ) report_i = i;
            if( s_count == 1 ) report_i = (i+9) % 60;     // double sync detect
            s_count = 0;
       }

       if( trend_t == new_t && trend_t != ERR ){          // increment the trend if match
           if( count < LIMIT ) ++count;
       }
    
       if( trend_t != new_t && new_t != ERR ){                    // valid decode, bit changed,  age type
         if( count == LIMIT && i > 8 ) val = 'x';                 // question this change even though valid decode
         if( count > 1 ) count -=  ( trend_t == SYNC ? 1 : 2 );   // age existing type, favor sync
         else{
            count = 1;          
            trend_t = new_t;
         }
       }
    
       trends[i] = trend_t + count;       // save new trend values

       // some unnessary playing around. Partial decode turned out to be not useful.
       // if( decodes == 0 && ( count >= LIMIT/2 || trend_t == SYNC || i == 8 ) ) part_decode(i);
       
       // return history on errors with only 1 bit incorrect
       if( new_t == ERR && count == LIMIT ){
          if( trend_t == ZEROS ){
              if( bit_errors(dat,0xfc,i) < 2 ) val = 'o';
          }
          if( trend_t == ONES ){
              if( bit_errors(dat,0xf0,i) < 2 ) val = 'i';
          }
          if( trend_t == SYNC ){
              if( bit_errors(dat,0xc0,i) < 2 ) val = 's';
          }          
       } 

      
       if( i == 0 && val == '.' ) val = 'Z';   //  view index on no decode no history   
       if( i == 0 ) disp_date_time();
       LCD.gotoRowCol(7,110);
       LCD.putch(val);
       LCD.printNumI(i,75,ROW7,2,'0');
        
    if( wwvb_quiet == 1 ){  
       Serial.write(val);
       if( i%10 == 9 ) Serial.write(' ');
    }

    debug_i = i;

    return val;
  
}


// return the number of bits different between mask and value
int bit_errors( uint8_t val, uint8_t mask, int i ){
int count;
uint8_t b;

   // only here if the val did not decode as exact
   // provide different levels of correction depending upon what field the val is in.
   // bit 1 that follows the double sync seems to be the most likely bit to decode incorrectly
   // require the double sync and other bits to all decode without correcting any bit errors bits 59-9
   if( i <= 9 || i == 59 ) return 8;        // require exact decode in the minutes field

   // hours field changes once per hour, don't correct bit pattern 11111000 which has one bit error for
   // both a 0 and a 1.
   if( val == 0xf8 && i > 11 && i < 19 && i != 14 ) return 8;    // 10,11,14 are unused always zero

   // the rest of the fields change once a year or are don't care bytes for this application.
   // count how many bits are different from the mask.  Decode from trends if only one bit is incorrect.

   count = 0,  b = 0x80;
   val = val ^ mask;                // val = error bits
   for( i = 0; i < 8; ++i ){        // count the errors
       if( val & b ) ++count;
       b >>= 1;  
   }
   
   return count;

}


/*
// correct errors on bits that do not change often
// this version moves the counters up to a hard limit and moves down on different bit decoded
// slips in time on weak signal as is called from frame_sync
char wwvb_trends( char val ){
static int i = 60;
static int unslip;
static uint8_t z;                      // stale out data counter
uint8_t count;
uint8_t trend_t,new_t;
int w;

#define LIMIT 10         // 2 min 30 max
#define SYNC 32
#define ONES 64
#define ZEROS 128
#define ERR  0

    if( clr_trends ){     // reset the data 
       for( w = 0; w < 60; ++w ) trends[w] = 0;
       clr_trends = 0;
    }
    
    if( unslip ) unslip = 0;
    else if( ++i >= 60 ){
       i = 0;
       // the sample bucket ends at 1 sec past the time sampled. Test once per two minute frame
       if( frame_sec < 60 && (frame_msec < 400 || frame_msec > 600) ){
           if( frame_sec == 0 && frame_msec > 600 ) ;       // ok, just early
           else if( frame_sec == 1 && frame_msec < 400 ) ;  // also ok
           else if( frame_sec < 30 ) ++i;                   // running slow is normal on weak signals
           else unslip = 1;                                 // probably a time reset or decode happened
       }
    }

   
       count = trends[i] & 31;                             // get existing trend data
       trend_t = trends[i] & ( ONES | SYNC | ZEROS );
       
       new_t = ERR;                                        // assume error
       if( val == 'S' ) new_t = SYNC;
       if( val == '1' ) new_t = ONES;
       if( val == '0' ) new_t = ZEROS;       

       if( trend_t == new_t && trend_t != ERR ){          // increment the trend if match
           if( count < LIMIT ) ++count;
       }
    
       if( trend_t != new_t && new_t != ERR ){                    // valid decode, bit changed,  age type
         if( count > 1 ) count -=  ( trend_t == SYNC ? 1 : 2 );   // age existing type, favor sync
         else{
            count = 1;          
            trend_t = new_t; 
         }
       }

       if( new_t == ERR && wspr_tx_enable == 0){      // slowly stale out the data on errors
           ++z;                                       // reduce count on 1 out of 16 errors
           z &= 15;
           if( z == 0 && count > 1 ) --count;       
       }
    
       trends[i] = trend_t + count;       // save new trend values

       // return history on errors, do not send trend for the minutes,hours fields
       if( new_t == ERR && ( i >= 19 || trend_t == SYNC || i == 4 || i == 10 || i == 11 || i == 14 ) ){ 
          if( count == LIMIT ){
             if( trend_t == ZEROS  ) val = 'o';
             if( trend_t == ONES ) val = 'i';
             if( trend_t == SYNC ) val = 's';
          }
       }

      
       if( i == 0 && val == '.' ) val = 'x';   //  view index on no decode no history    
    
    if( wwvb_quiet == 1 ){  
       Serial.write(val);
       if( i%10 == 9 ) Serial.write(' ');
    }

    return val;
  
}
*/

/*
// remove any errors on don't care bits, force to zero
// force syncs when we think we are in sync
char wwvb_trends( char val ){
const int zeros[17] = {4,10,11,14,20,21,24,34,35,36,37,38,44,54,56,57,58};
static int i,z;
static char last_val;
static int mod10;
static int q;
static int force;
static int fix;
static int syncs[4];
static int disable;
int w;

   
   // double sync reset
    if( last_val == 'S' && val == 'S' ){
       if( i != 59 ){
          i = 59;                           // index reset next lines of code
          force = 0;
       }
       disable = 0;
    }
    last_val = val;
    
    if( ++i >= 60 ){                     // start of next minute
       i = 0;
       z = 0;
       fix = 0;
       if( frame_sec == 4 ) ++i;                //  slipping in time.  Happens on very weak wwvb signal
                                                //  this test is 1 sec after the print at second 59
       if( frame_sec > 4 && frame_sec < 56 ) disable = 1;   // out of correction range, wait for double sync
    }

    if( val == 'S' && i > 4 && i < 56 ){      // syncs should all be modulo 9, double sync ignored,
       syncs[0] = syncs[1]; syncs[1] = syncs[2]; syncs[2] = syncs[3]; syncs[3] = i;   // for reporting only
       w = i%10;
       if( w == mod10 ) ++q;
       else q = 0, force = 0;
       mod10 = w;
       
       if( q >= 3 && disable == 0 ){           // quorum of syncs on same modulus index
          if( mod10 == 9 ) force = 1;          // correct index
          else if( mod10 >= 4 ) ++i;           // slipped in time
          else --i;                            // started early or some other issue like a time reset
          q = 0;       
       }
    }
                                             
    w = val;
    if( i == zeros[z] ){
        ++z;
        if( force && val == '.' ) val = 'o';  // remove errors on don't care bits
    }
    if( force && ( i == 0 || i%10 == 9 ) && val != 'S' ) val = 's';
    if( w != val ) ++fix;

    if( fix == 9 ) force = 0;                       // too many fails a minute

    if( wwvb_quiet == 1 && i == 59 ){               // report syncs
       if( frame_sec < 100 ) Serial.write(' ');
       if( frame_sec < 10  ) Serial.write(' ');
       Serial.print(frame_sec); Serial.write(' ');
       Serial.print("F "); Serial.print(force);
       Serial.print(" Fix ");
       // if( fix < 10 ) Serial.write(' '); 
       Serial.print(fix);
       Serial.print(" Sync ");
       for( w = 0; w < 4; ++w ){
           if( syncs[w] < 10 ) Serial.write(' ');
           Serial.print(syncs[w]);  Serial.write(' ');
       } 
    }

    
    return val;
}
*/

void frame_timer( unsigned long t ){   
static unsigned long old_t;
static uint8_t slot;

static long time_adjust;   
// 16mhz clock measured at 16001111.  Will gain 1ms in approx 14401 ms.  Or 1 second in 4 hours.
// the calibrate function has been repurposed to correct the time keeping of the Arduino.

   frame_msec += ( t - old_t );
    time_adjust += (t - old_t);
    if( time_adjust >= tm_correct_count && frame_msec != 0 ){
        time_adjust -= tm_correct_count;
        frame_msec += tm_correction;    // add one, sub one or no change depending upon tm_correction value
    }

    if( tm_correction2 && frame_msec > 400 && frame_msec < 600 ){
      frame_msec += tm_correction2;
      tm_correction2 = 0;
    }
    
   old_t = t;
   if( frame_msec >= 1000 ){
      frame_msec -= 1000;
      if( ++frame_sec >= 120 ){     // 2 minute slot time
        frame_sec -= 120;
        if( ++slot >= 8 ) slot = 0;   // 10 slots is a 20 minute frame
        if( slot == 1 && operate_mode == FRAME_MODE ) wspr_tx_enable = 1;
      }
      if( frame_sec == 116 ) cal_enable = 1;   // do once per slot in wspr quiet time
      if( frame_sec == 90 || frame_sec == 30 ) keep_time();   // half minute to avoid colliding with wwvb decodes
     // if( frame_sec ==  0 || frame_sec == 60 ) tick = 1;      // print wwvb decode info if enabled
   } 
}

void keep_time(){

   
   if( ++gmin >= 60 ){
      gmin = 0;
      if( ++ghr >= 24 ){
         ghr = 0;
         ++tot_days;
         if( tot_days > 365 + leap ) ++gyr, tot_days = 1;
         calc_date();
      }
   }
  
  // if( time_flags & TS ) time_flags = TP + TK;     // clear TS but flag a print of wwvb decode indicator
  // else  time_flags = TK;

}


void wspr_tx( unsigned long t ){
static int i;
static unsigned long timer;
static uint8_t mod;
static unsigned int one_second;

   // delay one second before starting transmission, this function is called once per millisecond
   if( one_second < 1000 && wspr_tx_cancel == 0 ){
      ++one_second;
      return;
   }

   if( wspr_tx_cancel ){      // quit early or just the end of the message
      if( i < 160 ) i = 162;  // let finish if near the end, else force done
   }
   
   if( (t - timer) < 683 ) return;   // baud time is 682.66666666 ms
   timer = t;
   if( ++mod == 3 ) mod = 0, ++timer;    // delay 683, 683, 682, etc.

   if( i == 162 ){
      tx_off();
      i = 0;                 // setup for next time to begin at zero index
      wspr_tx_cancel = wspr_tx_enable = 0;    // flag done
      one_second = 0;
      return;
   }

   // set the frequency
   si_pll_x(PLLA,Rdiv*4*(freq+audio_freq),divider,Rdiv*4*146*wspr_msg[i]);
   if( i == 0 ) tx_on();
   ++i; 
}

void tx_on(){

  digitalWrite(MUTE,HIGH);
  digitalWrite(WWVB_PWDN,LOW); 
  i2cd(SI5351,3,0xff ^ (CLK0_EN));   // tx clock on, other clocks off during tx
}

void tx_off(){
  
    i2cd(SI5351,3,0xff ^ (CLK1_EN + CLK2_EN) );   // turn off tx, turn on rx and cal clocks
    i2flush();                                    // wait for tx off to complete before enabling the rx
    si_pll_x(PLLA,Rdiv*4*freq,divider,0);         // return to RX frequency
    digitalWrite(MUTE,LOW);                       // enable receiver
    digitalWrite(WWVB_PWDN,HIGH);                 // enable wwvb receiver

    ee_save();     // save this freq to use during stand alone mode(FRAME MODE).
}


void i2cd( unsigned char addr, unsigned char reg, unsigned char dat ){
  // direct register writes.  A possible speed up could be realized if one were
  // to use the auto register inc feature of the SI5351

   i2start(addr);
   i2send(reg);
   i2send(dat);
   i2stop();
}


/******   SI5351  functions   ******/

void  si_pll_x(unsigned char pll, uint32_t freq, uint32_t out_divider, uint32_t fraction ){
 uint64_t a,b,c;
 uint64_t bc128;             // floor 128 * b/c term of equations
 uint64_t pll_freq;
 uint64_t cl_freq;

 uint32_t P1;            // PLL config register P1
 uint32_t P2;            // PLL config register P2
 uint32_t P3;            // PLL config register P3
 uint64_t r;

   cl_freq = clock_freq;
   
   //if( pll == PLLA ) cl_freq += drift;               // drift not applied to 3 mhz calibrate freq
   cl_freq += drift;                                 // drift applied to 3 mhz.  Pick one of these.
   
   c = 1000000;     // max 1048575
   pll_freq = 100ULL * (uint64_t)freq + fraction;    // allow fractional frequency for wspr
   pll_freq = pll_freq * out_divider;
   a = pll_freq / cl_freq ;
   r = pll_freq - a * cl_freq ;
   b = ( c * r ) / cl_freq;
   bc128 =  (128 * r)/ cl_freq;
   P1 = 128 * a + bc128 - 512;
   P2 = 128 * b - c * bc128;
   if( P2 > c ) P2 = 0;        // avoid negative numbers 
   P3 = c;

   i2cd(SI5351, pll + 0, (P3 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 1, (P3 & 0x000000FF));
   i2cd(SI5351, pll + 2, (P1 & 0x00030000) >> 16);
   i2cd(SI5351, pll + 3, (P1 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 4, (P1 & 0x000000FF));
   i2cd(SI5351, pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
   i2cd(SI5351, pll + 6, (P2 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 7, (P2 & 0x000000FF));
   
 //  i2cd( SI5351, 177, 0xAC );         // PLLA PLLB soft reset  
}


// load new divider for specified clock, reset PLLA and PLLB if desired
void si_load_divider( uint32_t val, uint8_t clk , uint8_t rst, uint8_t Rdiv){
uint8_t R;

   R = 0;
   Rdiv >>= 1;      // calc what goes in the R divisor field
   while( Rdiv ){
      ++R;
      Rdiv >>= 1;
   }
   R <<= 4;     
   
   val = 128 * val - 512;
   i2cd( SI5351, 44+8*clk, ((val >> 16 ) & 3) | R );
   i2cd( SI5351, 45+8*clk, ( val >> 8 ) & 0xff );
   i2cd( SI5351, 46+8*clk, val & 0xff );   
   if( rst ) i2cd( SI5351, 177, 0xAC );         // PLLA PLLB soft reset needed
}

void freq_display(){
int rem;
  
   LCD.setFont(MediumNumbers);
   LCD.printNumI(freq/1000,0,ROW0,5,'/');       // '/' is a leading space with this font table
   LCD.setFont(SmallFont);
   rem = freq % 1000;
   LCD.printNumI(rem,64,ROW0,3,'0');   
}

// display mode and step
void mode_display(){

   LCD.setFont(SmallFont); 
   if( operate_mode == CAT_MODE ) LCD.print(" CAT",RIGHT,ROW0);
   else LCD.print("WSPR",RIGHT,ROW0);

   LCD.printNumI(stp,RIGHT,ROW1,7,' ');
}

/*****************************************************************************************/
// TenTec Argonaut V CAT emulation

//int un_stage(){    /* send a char on serial */
//char c;

//   if( stg_in == stg_out ) return 0;
//   c = stg_buf[stg_out++];
//   stg_out &= ( STQUESIZE - 1);
//   Serial.write(c);
//   return 1;
//}

#define CMDLEN 20
char command[CMDLEN];
uint8_t vfo = 'A';

void radio_control() {
static int expect_len = 0;
static int len = 0;
static char cmd;

char c;
int done;

    if (Serial.available() == 0) return;
    
    done = 0;
    while( Serial.available() ){
       c = Serial.read();
       command[len] = c;
       if(++len >= CMDLEN ) len= 0;  /* something wrong */
       if( len == 1 ) cmd = c;       /* first char */
       /* sync ok ? */
       if( cmd == '?' || cmd == '*' || cmd == '#' );  /* ok */
       else{
          len= 0;
          return;
       }
       if( len == 2  && cmd == '*' ) expect_len = lookup_len(c);    /* for binary data on the link */       
       if( (expect_len == 0 &&  c == '\r') || (len == expect_len) ){
         done = 1;
         break;   
       }
    }
    
    if( done == 0 ) return;  /* command not complete yet */
        
    if( cmd == '?' ){
      get_cmd();
      operate_mode = CAT_MODE;            // switch modes on query cat command
      if( wwvb_quiet < 2 ) ++wwvb_quiet;  // only one CAT command enables wwvb logging, 2nd or more turns it off
      mode_display();
    }
    if( cmd == '*' )  set_cmd();
    if( cmd == '#' ){
        pnd_cmd(); 
        if( wwvb_quiet < 2 ) ++wwvb_quiet;  // allow FRAME mode and the serial logging at the same time
    }

 /* prepare for next command */
   len = expect_len= 0;
   stage('G');       /* they are all good commands */
   stage('\r');

}

int lookup_len(char cmd2){     /* just need the length of the command */
int len;

   
   switch(cmd2){     /* get length of argument */
    case 'X': len = 0; break;
    case 'A':
    case 'B': len = 4; break;
    case 'E':
    case 'P':
    case 'M': len = 2; break;
    default:  len = 1; break ;
   }
   
   return len+3;     /* add in *A and cr on the end */
}

void set_cmd(){
char cmd2;
unsigned long val4;

   cmd2 = command[1];
   switch(cmd2){
    case 'X':   stage_str("RADIO START"); stage('\r'); break; 
    case 'O':   /* split */ 
    break;
    case 'A':   // set frequency
    case 'B':
       val4 = get_long();
       qsy(val4);       
    break;
    case 'E':
       if( command[2] == 'V' ) vfo = command[3];
    break;
    case 'W':    /* bandwidth */
    break;
    case 'K':    /* keying speed */
    break;
    case 'T':    /* added tuning rate as a command */
    break;       
   }  /* end switch */   
}

void get_cmd(){
char cmd2;
long arg;
int len;

   cmd2 = command[1];   
   stage(cmd2);
   switch(cmd2){
    case 'A':     // get frequency
    case 'B': 
      arg = freq;
      stage_long(arg);
    break;
    case 'V':   /* version */
      stage_str("ER 1010-516");
    break;
    case 'W':          /* receive bandwidth */
       stage(30);
    break;
    case 'M':          /* mode. 1 is USB USB  ( 3 is CW ) */
       stage('1'); stage('1');
    break;
    case 'O':          /* split */   
       stage(0);
    break;
    case 'P':         /*  passband slider */
       stage_int( 3000 );
    break;
    case 'T':         /* added tuning rate command */
    break;   
    case 'E':         /* vfo mode */
      stage('V');
      stage(vfo);
    break;
    case 'S':         /* signal strength */
       stage(7);
       stage(0);
    break;
    case 'C':      // transmitting status 
       stage(0);
       if( wspr_tx_enable ) stage(1);
       else stage(0);
    break;
    case 'K':   /* wpm on noise blanker slider */
       stage( 15 - 10 );
    break;   
    default:           /* send zeros for unimplemented commands */
       len= lookup_len(cmd2) - 3;
       while( len-- ) stage(0);  
    break;    
   }
  
   stage('\r');  
}


void stage_str( String st ){
int i;
char c;

  for( i = 0; i < st.length(); ++i ){
     c= st.charAt( i );
     stage(c);
  }    
}

void stage_long( long val ){
unsigned char c;
   
   c= val >> 24;
   stage(c);
   c= val >> 16;
   stage(c);
   c= val >> 8;
   stage(c);
   c= val;
   stage(c);
}


unsigned long get_long(){
union{
  unsigned long v;
  unsigned char ch[4];
}val;
int i;

  for( i = 0; i < 4; ++i) val.ch[i] = command[5-i]; // or i+2 for other endian
  return val.v;
}

void stage_int( int val ){
unsigned char c;
   c= val >> 8;
   stage(c);
   c= val;
   stage(c);
}

void stage_num( int val ){   /* send number in ascii */
char buf[35];
char c;
int i;

   itoa( val, buf, 10 );
   i= 0;
   while( c = buf[i++] ) stage(c);  
}

void pnd_cmd(){
char cmd2;
   
   cmd2 = command[1];
   switch(cmd2){
     case '0':      // enter rx mode
        if( wspr_tx_enable ) wspr_tx_cancel = 1;
     break;
     case '1':  wspr_tx_enable = 1;  break;    // TX
   }

}

/***** non-blocking, write only, I2C  functions   ******/

#define I2BUFSIZE  64
int i2buf[I2BUFSIZE];
uint8_t i2in, i2out;

// use some upper bits in the buffer for control
#define ISTART 0x100
#define ISTOP  0x200

void i2init()
{
  TWBR = 72;   //  12 400k for 16 meg clock.  72 100k   ((F_CPU/freq)-16)/2
  TWSR = 0;
  TWDR = 0xFF;
  PRR = 0;
}

void i2start( unsigned char adr ){
int dat;
  // shift the address over and add the start flag
  dat = ( adr << 1 ) | ISTART;
  i2send( dat );
}

void i2send( unsigned int data ){   // just save stuff in the buffer
uint8_t t;

  // but check for space first
  t = ( i2in + 1 ) & (I2BUFSIZE - 1);
  while( t == i2out ) i2poll();        // wait for space
  
  i2buf[i2in++] = data;
  i2in &= (I2BUFSIZE - 1);
  i2poll();
}

void i2stop( ){
   i2send( ISTOP );   // que a stop condition
}


void i2flush(){  // call poll to empty out the buffer. 

  while( i2poll() ); 
}


uint8_t i2poll(){    // everything happens here.  Call this from loop.
static uint8_t state = 0;
static int data;
static uint8_t delay_counter;

 // the library code has a delay after loading the transmit buffer
 // and before the status bits are tested for transmit active
   if( delay_counter ){  
     --delay_counter;
     return (16 + delay_counter);
   }
   
   switch( state ){    
      case 0:      // idle state or between characters
        if( i2in != i2out ){   // get next character
           data = i2buf[i2out++];
           i2out &= (I2BUFSIZE - 1 );
           
           if( data & ISTART ){   // start
              data &= 0xff;
              // set start condition
              TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
              state = 1; 
           }
           else if( data & ISTOP ){  // stop
              // set stop condition
              TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
              state = 3;
           }
           else{   // just data to send
              TWDR = data;
              TWCR = (1<<TWINT) | (1<<TWEN);
              delay_counter = 1;   // delay for transmit active to come true
              state = 2;
           }
        }
      break; 
      case 1:  // wait for start to clear, send saved data which has the address
         if( (TWCR & (1<<TWINT)) ){
            state = 2;
            TWDR = data;
            TWCR = (1<<TWINT) | (1<<TWEN);
            delay_counter = 1;
         }
      break;
      case 2:  // wait for ack/nack done and tbuffer empty, blind to success or fail
         if( (TWCR & (1<<TWINT)) ){  
            state = 0;
         }
      break;
      case 3:  // wait for stop to clear
         if( (TWCR & (1<<TWSTO)) == 0 ){
            state = 0;
            delay_counter = 1;  // a little delay at the end of a sequence
         }
      break;    
   }

   if( i2in != i2out ) return (state + 8);
   else return state;
}


// display a value on blank row 4 in binary
void debug_print( uint8_t val, int col ){
int i;
char c;

    LCD.gotoRowCol(4,col);

    for( i = 0; i < 8; ++i ){
       c = ( val & 0x80 )? '1' : '0';
       val <<= 1;
       LCD.putch(c);        
    }
}

// WWVB receiver in a fringe area - integrate the signal to remove noise
// Although it probably makes more sense to dump the integrator 10 times per second, here we use 8.
// sample each millisecond, sum 100 or 150 samples , decide if low or high, shift into temp variable
// at end of 1 second( 8 bits), decide if temp has a 1, 0, or sync. Shift into 64 bit data and sync variables.
// when the sync variable contains the magic number, decode the 64 bit data.
// each second starts with a low signal and ends with a high signal
// much like software sampling rs232 start and stop bits.
// this routine runs fast by design until it locks on the wwvb signal( or slow depending upon point of view )
// if more than 30 seconds out of sync with the frame timer, then the time printed may be incorrect
// but that may take days and the time is only printed in debug mode
// this original version seems to work better in noise than the 2nd version
void wwvb_sample2(unsigned long t){
static unsigned long old_t;
int loops;
uint8_t b,s,e;
static uint8_t wwvb_clk, wwvb_sum, wwvb_tmp, wwvb_count;  // data decoding
const uint8_t counts[8] = { 100,100,150,150,150,150,100,100 };  // total of 1000 ms
static uint8_t secs,errors,early,late;
static uint8_t dither = 4;              // quick sync, adjusts to 1 when signal is good
char ch;
static char valp[9];       // LCD display of slow/fast timekeeping
static int8_t vali, valc;

   loops = t - old_t;
   old_t = t;
   
  while( loops-- ){   // repeat for any missed milliseconds  

   if( digitalRead(WWVB_OUT) == LOW ) ++wwvb_sum;

   if( --wwvb_clk == 0 ){    // end of period, dump integrator
      b = ( wwvb_sum > (counts[wwvb_count] >> 1) ) ? 0 : 128;
      wwvb_tmp >>= 1;
      wwvb_tmp |= b;
      wwvb_sum = 0;

      // 8 dumps of the integrator is one second, decode this bit ?
      wwvb_count++;
      wwvb_count &= 7;
      wwvb_clk = counts[wwvb_count];  // 100 100 150 150 150 150 100 100
                              // decode     0       1      sync    stop should be high
      if( wwvb_count == 0 ){    // decode time

        // clocks late or early, just dither them back and forth across the falling edge
        // when not in sync, more 1's than 0's are detected and this slips in time.
      if( wwvb_tmp != 0xff  && wwvb_tmp != 0x00  ){
         if( digitalRead(WWVB_OUT) == 0 ){  
            ++late;                    // sampling late
            wwvb_clk -= dither;        // adjust sample to earlier
         }
         else{
            ++early;                   // need to sample later
            wwvb_clk += dither;        // longer clock
         }
      }
      
        // decode
        // 11111100 is a zero,  11110000 is a one, 11000000 is a sync      
        b = 0;  s = 0;  e = 1;   // assume it is an error
        
        // strict decode works well, added some loose decode for common bit errors
        if( wwvb_tmp == 0xfc /*|| wwvb_tmp == 0xfd || wwvb_tmp == 0xfe*/ ) e = 0, b = 0;
        if( wwvb_tmp == 0xf0 /*|| wwvb_tmp == 0xf1*/ ) e = 0, b = 1;
        if( wwvb_tmp == 0xc0 /*|| wwvb_tmp == 0xc1*/ ) e = 0, s = 1;

        gather_stats( wwvb_tmp , e );         // for serial logging display

        if( e ) ch = '.';
        else if( s ) ch = 'S';
        else if( b == 0 ) ch = '0';
        else if( b == 1 ) ch = '1';
        ch = wwvb_trends(ch , wwvb_tmp);

        debug_print( wwvb_tmp, 20 );
        if( debug_i == 12 ) debug_print( wwvb_tmp, 80 );

        // decode from trends
        if( ch == 'o' ) b = 0, e = 0;
        if( ch == 's' ) s = 1, e = 0;
        if( ch == 'i' ) b = 1, e = 0;
        if( ch == 'x' ) e = 1;               // flag as error if decoded different from the past history

        if( e ) ++errors;
        
        wwvb_data <<= 1;   wwvb_data |= b;    // shift 64 bits data
        wwvb_sync <<= 1;   wwvb_sync |= s;    // sync
        wwvb_errors <<= 1; wwvb_errors |= e;  // errors

        // magic 64 bits of sync  ( looking at 60 seconds of data with 4 seconds of the past minute )
        // xxxx1000000001 0000000001 0000000001 0000000001 0000000001 0000000001
        // wwvb_sync &= 0x0fffffffffffffff;   // mask off the old bits from previous minute
        // instead of masking, use the old bits to see the double sync bits at 0 of this minute
        // and 59 seconds of the previous minute.  This decodes at zero time.
        if( wwvb_sync == 0b0001100000000100000000010000000001000000000100000000010000000001 ){
          if( wwvb_errors == 0 ){    // decode if no bit errors
             wwvb_decode();
          }
        }

        if( ++secs >= 60 /* || tick */ ){  //  adjust dither each minute
          // tick = 0;
           val_print = ' ';
           frame_sync2( errors,frame_msec );
           
           // running in print,       wwvb decode,   keep_time order    or
           //         in wwvb decode, print,         keep_time order    this case needs a time update
           // if( time_flags & TS ) keep_time();  // need to increment the time since the last decode
           // new method = time +- 30 seconds is printed as current time. This routine will eventually seem to
           // skip ahead a minute to catch up in actual time.
    
                 // debug print out some stats when in test mode
                 // break this up for 1200 baud, takes too much time and causes missed decode after line feed                
           if( wwvb_quiet == 1 ){
               Serial.write(' ');
               Serial.print(report_i);  Serial.write(' ');
               print_date_time();       
               Serial.write(' '); 
               if( frame_msec < 100 ) Serial.write(' ');
               if( frame_msec < 10 ) Serial.write(' ');
               Serial.print(frame_msec);
               dbug_errors(1,errors,val_print,early,late);
               
             //  Serial.print("  Err "); Serial.print(errors); Serial.write(val_print);
             //  Serial.print("  Clk ");  Serial.print(early);
             //  Serial.write(',');   Serial.print(late);
             //  print_stats(1,errors);
             //  Serial.print("  FF "); Serial.print(FF);  
             //  Serial.write(' ');  Serial.print(ff);
             //  Serial.print("  Drift ");   Serial.print((int)drift/100);
             //  Serial.print("  CC "); Serial.print(tm_correct_count);
             //  Serial.print("  Cal "); Serial.print(cal_result);
             //  Serial.println();
           }
           else print_stats(0,errors);

           // use stats for an early sync to correct second
           if( decodes == 0 && report_i > 0 && report_i < 20 ){
              if( report_i < 9 ) tm_correction2 += 100;
              if( report_i > 9 ) tm_correction2 -= 100;
             // if( report_i != 9 ) report_i = 0;         // one time only for each detect( now one second adjust )
           }

           // time_flags = 0;
          // LCD.setFont(MediumNumbers);
          // LCD.printNumI(errors,RIGHT,ROW0,2,'/');
          // LCD.printNumI(frame_msec,RIGHT,ROW5,3,' ');
          if( val_print == '-' || val_print == '+' ) valp[vali++] = val_print;    // time trend slow or fast
          if( val_print == ' ' ){                                                 // time trend keeping time well
             ++valc;
             valc &= 63;
             if( valc == 0 ) valp[vali++] = val_print;
          }
          vali &= 7;
          LCD.print(valp,RIGHT,ROW5);
          LCD.printNumI(FF,100,ROW6);
          LCD.putch(' ');
          LCD.printNumI(ff,RIGHT,ROW6,2,' ');
           
           dither = ( errors >> 4 ) + 1;
           early = late = secs = errors = 0;   // reset the stats for the next minute

        }

      }  // end decode time    
    }    // end integration timer
  }      // loops - repeat for lost milliseconds if any
}

void dbug_errors(uint8_t st,uint8_t errors,char val_print,uint8_t early, uint8_t late){
static uint8_t err, vp, earl, lt;

     if( st == 1 ) dbug_print_state = 1;        // flag for start of new data
     
     switch( dbug_print_state ){
       case 1:                      // save the data
          err = errors, vp = val_print, earl = early, lt = late;
          ++dbug_print_state;
          break;
       case 2:                      // print first group   
          Serial.print("  Err "); Serial.print(err); Serial.write(vp);
          Serial.print("  Clk ");  Serial.print(earl);
          Serial.write(',');   Serial.print(lt);
          ++dbug_print_state;
          break;
       case 3:  
          print_stats(1,err);
          ++dbug_print_state;
          break;
       case 4:              
          Serial.print("  FF "); Serial.print(FF);  
          Serial.write(' ');  Serial.print(ff);
          Serial.print("  Drift ");   Serial.print((int)drift/100);
          ++dbug_print_state;
          break;
       case 5:   
          Serial.print("  CC "); Serial.print(tm_correct_count);
          Serial.print("  Cal "); Serial.print(cal_result);
          Serial.println();
          dbug_print_state = 0;
          break;
     }
}


void print_date_time(){

   if( gmon < 10 ) Serial.write('0');
   Serial.print(gmon);  Serial.write('/');
   if( gday < 10 ) Serial.write('0');
   Serial.print(gday);  Serial.write('/');
   if( gyr < 10 ) Serial.write('0');
   Serial.print(gyr);  Serial.write(' ');

   if( ghr < 10 ) Serial.write('0');
   Serial.print(ghr);   Serial.write(':');
   if( gmin < 10 ) Serial.write('0');
   Serial.print(gmin);

   if( time_flags & TS ) Serial.write('*'), time_flags = 0;      // decode from wwvb flagged
   else Serial.write(' ');
  
}

void disp_date_time(){
int local_hr;           // display local time for eastern timezone

   local_hr = ghr - 5 + (int)DST;
   if( local_hr < 0 ) local_hr += 24;
   if( local_hr > 11 ) local_hr -= 12;
   if( local_hr == 0 ) local_hr = 12;
  
   LCD.setFont(MediumNumbers);
   LCD.printNumI(gmon,LEFT,ROW2,2,'0');
   LCD.printNumI(gday,40,ROW2,2,'0');
   LCD.printNumI(gyr,80,ROW2,2,'0');
   LCD.setFont(BigNumbers);
   //LCD.printNumI(ghr,0,ROW5,2,'/');        // UTC time
   LCD.printNumI(local_hr,0,ROW5,2,'/');      // local time
   LCD.printNumI(gmin,40,ROW5,2,'0');
   LCD.setFont(SmallFont);
}

void gather_stats( uint8_t data, uint8_t err ){
uint8_t i;

   if( err ) wwvb_last_err = data;  // capture the last failed data bits for Serial log
   
   for( i = 0; i < 8; ++i ){
      if( data & 1 ) ++wwvb_stats[i];
      data >>= 1;
   }
 
}


void print_stats(uint8_t prnt, int tot){
uint8_t i;

   if( prnt ){                // ones and zeros distribution
      Serial.print("  ");     // when in sync with WWVB, will see a display such as 11XXxx00

      if( tot > 3 || tot == 0 ){
         for( i = 7;  i < 8; --i ){
            if( wwvb_stats[i] > 55 ) Serial.write('1');
            else if( wwvb_stats[i] < 5 ) Serial.write('0');
            else if( wwvb_stats[i] > 40 ) Serial.write('X');
            else Serial.write('x');
         }
      }
      else{

     // print an error in binary, example failing data
         for( i = 7; i < 8; --i ){
            if( wwvb_last_err & 0x80 ) Serial.write('1');
            else Serial.write('0');
            wwvb_last_err <<= 1;
         }
      }
   }
 
   for( i = 0; i < 8; ++i ) wwvb_stats[i] = 0;
 
}

// adjust frame timing based upon undecoded wwvb statistics, locks to the falling edge of the 
// wwvb signal.
void frame_sync2(int err, long tm){
int8_t t,i; 
int loops;
int cnt;

   if( err >= CLK_UPDATE_THRESHOLD2 ){
      val_print = '^';
    return;
   }

   tm = ( tm < 500 ) ? tm : tm - 1000 ;
   cnt = CLK_UPDATE_THRESHOLD2 - err;
   tm = last_time_average( tm, cnt );
   if( tm >= 0 && tm < DEADBAND ) return;
   if( tm < 0  && tm > -DEADBAND) return;
   
   loops = tm/100;
   if( loops < 0 ) loops = -loops;
   ++loops;
   
   t = 0;  
   for( i = 0; i < loops; ++i ){        // run mult times for faster convergence

       if( tm > 0 ) t = -1;
       if( tm < 0 ) t = 1;

       tm_correction2 += t;
       clock_correction( t );           // long term clock drift correction

   }

   if( t == 1 ) val_print = '+';
   if( t == -1) val_print = '-';
  
}


long last_time_average( long val, int count ){    // average values

static long run_ave;     // weighted running average
int wt;
long rval;

   val <<= 8;            // scale up to keep a fractional part
   
   wt = ( count > 64 ) ? 64 : count;
   run_ave = ( 64 - wt ) * run_ave + wt * val;
   run_ave >>= 6;

   rval = run_ave >> 8;
   if( rval > DEADBAND ) run_ave -= 256;            // sub one ms when frame timing will be changed
   if( rval < -DEADBAND ) run_ave += 256;
   // Serial.print(rval); Serial.write(' ');
   return rval;

}

/*************************** some old code with interesting algorithms ****************************
//   sample the wwvb signal and detect bits, syncs, and errors
void wwvb_sample(unsigned long t){
static unsigned long old_t;
int loops;
static uint8_t bounce;
static uint8_t state;
static long ms;
static unsigned int low_counter;
static unsigned int high_counter;
static long ave_time;
static long ave_count;
uint8_t b,s,e;
int adj;
// int clock_adj_sum;
static uint64_t wwvb_data, wwvb_sync, wwvb_errors;  // defeat this algorithm by not using the globals

  loops = t - old_t;
  old_t = t;
   
  while( loops-- ){   // repeat for any missed milliseconds

   bounce <<= 1;
   if( digitalRead(WWVB_OUT) == HIGH ) bounce |= 1;     // debounce, looking for zero or 255 value
   ++ms;
   
   switch(state){
      case 0:                          // looking for a low signal
         ++high_counter;
   
         if( bounce == 0 ){            // found the low,  decode the bit for the previous second
            state = 1;
            b = s = e = 0;
            high_counter += low_counter;                                 // get total frame ms
            if( high_counter < 800 || high_counter > 1200 ) e = 1;       // too short or too long
            
            if( low_counter > 100 && low_counter < 300 ) b = 0;          // decode the bit
            else if( low_counter > 400 && low_counter < 600 ) b = 1;
            else if( low_counter > 700 && low_counter < 900 ) s = 1;
            else e = 1;                                                  // no valid decode
            
            low_counter = 0;
            
            wwvb_data <<= 1;   wwvb_data |= b;    // shift 64 bits data
            wwvb_sync <<= 1;   wwvb_sync |= s;    // sync
            wwvb_errors <<= 1; wwvb_errors |= e;  // errors
            
        // magic 64 bits of sync  ( looking at 60 seconds of data with 4 seconds of the past minute )
        // xxxx1000000001 0000000001 0000000001 0000000001 0000000001 0000000001
        // use the old bits to see the double sync bits at 0 of this minute
        // and 59 seconds of the previous minute.  This decodes at zero time rather than some
        // algorithms that decode at 1 second past.
           if( wwvb_sync == 0b0001100000000100000000010000000001000000000100000000010000000001 ){
             if( wwvb_errors == 0 ){    // decode if no bit errors
              // wwvb_decode();         // not used if commented
             }
           }            

            //  gather some stats for printing
           if( e == 0 ){
              ave_time += frame_msec;
              if( frame_msec >= 500 ) ave_time -= 1000;      // 500-999 averaged in as -(1000-frame_msec)
              ++ave_count;                                   // implemented as +frame_msec - 1000
           }
         }
      break;
      case 1:                          // looking for a high signal
         ++low_counter;
         if( bounce == 255 ){
            state = 0;
            high_counter = 0;
         }

      break;
   }

   if( ms >= 60000 ){                   // print out some stats if in printing mode
      
      if( ave_count ){
         ave_time = ave_time/ave_count;
         if( ave_time < 0 ) ave_time += 1000;
      }

      val_print = ' ';
      adj = frame_sync( 60-ave_count, ave_time );      // sync our seconds to wwvb falling edge signal
      
      // debug print out some stats when in test mode
      if( wwvb_quiet == 1 && val_print == '*'){   
          //clock_adj_sum = clock_freq - START_CLOCK_FREQ;    
          Serial.print("Tm "); 
          if( ave_time < 100 ) Serial.write(' ');
          if( ave_time < 10 ) Serial.write(' ');
          Serial.print(ave_time);
          Serial.print("  Ave");
          if( adj >= 0 ) Serial.write(' ');
          if( abs(adj) < 100 ) Serial.write(' ');
          if( abs(adj) < 10 ) Serial.write(' ');
          Serial.print(adj);
          Serial.print("  Valid ");
          if( ave_count < 10 ) Serial.write(' ');
          Serial.print(ave_count);
          Serial.write(val_print);
          Serial.print("  FF "); Serial.print(ff);  
          Serial.write(' ');  Serial.print(hourFF);
          Serial.write(',');  Serial.print(dayFF);
          Serial.print("  Drift ");   Serial.print((int)drift/100);
          Serial.print("  CC "); Serial.print(tm_correct_count);
          Serial.print("  Cal Freq "); Serial.print(cal_result);  
          Serial.println();
      }
        // reset the stats for the next minute
      ms = ave_count = ave_time = 0;
   }

  }      // loops - repeat for lost milliseconds if any
}


long median( long val ){    // return the median of 3 values
int i,j,k;
static long vals[3];
static int in;

   // first time
   if( vals[0] == 0 ){
      vals[0] = vals[1] = vals[2] = val;
      return val;
   }

   vals[in++] = val;
   if( in > 2 ) in = 0;

   i = 0, j = 1, k = 2;                      // assume in correct order low to high
   if( vals[i] > vals[k] ) i = 2, k = 0;     // swap low and high
   if( vals[j] < vals[i] ) j = i;            // mid val >= low val
   if( vals[j] > vals[k] ) j = k;            // mid val <= high val

   //Serial.print( vals[0] );   Serial.write(' ');
   //Serial.print( vals[1] );   Serial.write(' ');
   //Serial.print( vals[2] );   Serial.write(' ');
   
   return vals[j];
}


// adjust frame timing based upon undecoded wwvb statistics, locks to the falling edge of the 
// wwvb signal.
int frame_sync(int err, long tm){
int8_t t,i;
static int last_time_error;
//static int last_error_count = 60;  // made global for printing  
int loops;
int cnt;
int temp;

   if( tm > 980 || tm < 20 ) tm = 0; // deadband for clock corrections
   cnt = 60 - err;
   loops = last_time_error/100;      // loop 1,2,3,4 or 5 times for error <100, <200, <300, <400, <500
   if( loops < 0 ) loops = -loops;
   ++loops;

   if( last_error_count <= CLK_UPDATE_THRESHOLD )  ++last_error_count;   // relax the test threshold

                    // average the values for large error counts
   temp = ( tm < 500 ) ? tm : tm - 1000 ;
   temp = last_time_average( temp, cnt );    
   
   for( i = 0; i < loops; ++i ){                     // run mult times for faster correction convergence
    
       t = 0;                                        // signal better than the relaxing threshold ?

       if( err < last_error_count ){
           t = ( tm < 500 ) ? -1 : 1 ;  
           if( tm == 0 ) t = 0;
           last_time_error = temp + t;
           //if( cnt < 3 ) last_time_error >>= 3;      // small valid signals, limit time delta allowed
           last_time_error = constrain(last_time_error,-5*cnt,5*cnt);  // limit change for larger errors count
           last_error_count = err;       // new threshold
           val_print = '*';
       }

       if( t == 0 ){       // use old data for the correction amount
          if( last_time_error > 0 ) last_time_error--, t = -1;
          if( last_time_error < 0 ) last_time_error++, t = 1;
       }

          tm_correction2 += t;
          clock_correction( t );  // long term clock drift correction

       err = 60;    // use last_time info for the 2nd pass in the loop
   }
   
   //return -(last_time_error);    // return value for printing
   return temp;
}



void clock_correction( int8_t val ){    // long term frequency correction to time fudge factor FF
static int8_t time_trend;               // a change of +-100 is 1hz change
uint8_t changed;                        // correct for seasonal temperature variations to clocks
                                        // using the WWVB time to arduino time errors
    if( wspr_tx_enable ) return;                                // ignore this when transmitting
    
    time_trend -= val;             // or should it be += val;

    changed = 0;
    if( time_trend >= CLK_UPDATE_MIN ) clock_freq += CLK_UPDATE_AMT, changed = 1;  
    if( time_trend <= -CLK_UPDATE_MIN ) clock_freq -= CLK_UPDATE_AMT, changed = 1;
    
    if( changed ){    // set new dividers for the new master clock freq value to take effect
       time_trend = 0;
       if( clock_freq > 2700486600ULL ) clock_freq -= 100;   // stay within some bounds, +- 400
       if( clock_freq < 2700406600ULL ) clock_freq += 100;   // +- 100 at 7mhz
       si_pll_x(PLLB,cal_freq,cal_divider,0);   // calibrate frequency on clock 2
       si_pll_x(PLLA,Rdiv*4*freq,divider,0);    // receiver 4x clock
    }
}

// regular decode needs 64 bits correct on the same minute.  This decode builds each bit by itsself.
// Will probably decode incorrectly at some error rate.  Found to be not very useful as weak signals decode
// mostly as zeros and when the signal improves and these algorithms get a decode,
// an error free regular decode quickly follows

void part_decode( int i ){
#define SYNC 32
#define ONES 64
#define ZEROS 128
static int8_t offset = SYNC;
uint8_t count,type,b;
static int8_t sync_val, sync_mod;

    if( i < 8 || i > 56 ) return;
    if( i == 8 ) sync_val = 0, sync_mod = 128;    // reset sync count
    
    type = trends[i] & ( SYNC | ONES | ZEROS );
    count = trends[i] & 31;
    
    // first need to find the offset so bits go in correct place. i should be close to 9 mod 10.
    if( offset == SYNC ){
        if( type != SYNC ) return;
        b = i % 10;
        if( b != sync_mod ) sync_val = 0, sync_mod = b;    // reset on no match
        sync_val += count; 
        if( sync_val < 4 ) return;
        offset = sync_mod;
        if( offset < 5 ) offset += 10;
        offset = 9 - offset;
        // Serial.print("Offset "); Serial.println(offset);
        return;
    }
    if( type == SYNC ) return;      // don't need to look at syncs anymore

    i = i + offset;
    b = ( type == ONES )? 1:0;

    if( i < 19 ) part_decode_hours(i,b);
    if( i > 21 ) part_decode_date(i,b);
  
}

void part_decode_hours( int i, uint8_t dat ){
static uint8_t mask,val;
uint8_t setb,clearb;
uint64_t save;

   setb = 1;  clearb = 0xff;
   i = 18 - i;
   while( i-- ) setb <<= 1;
   clearb ^= setb;
   mask |= setb;
   if( dat ) val |= setb;
   else val &= clearb;

   if( mask == 0x7f ){
       // Serial.print("Hour bits"); Serial.println(val,BIN);
       save = wwvb_data;
       wwvb_data = val;
       wwvb_data <<= 59-18;
       ghr = wwvb_decode2( 18, 0x3f );
       wwvb_data = save;
       mask = 0;            // start over or just disable.  Will it decode every minute now.
   }
  
}

void part_decode_date( int i, uint8_t dat ){
static uint16_t mask = 0xf010;
static uint16_t val;
uint16_t setb,clearb;
uint64_t save;

    if( i == 55 ){
       leap = dat;
       return;
    }

    if( i <= 33 ){
       setb = 1;  clearb = 0xffff;
       i = 33 - i;
       while( i-- ) setb <<= 1;
       clearb ^= setb;
       mask |= setb;
       if( dat ) val |= setb;
       else val &= clearb;
       if( mask == 0xffff ){
          //Serial.println("Date");
          save = wwvb_data;
          wwvb_data = val;
          wwvb_data <<= 59-33;
          tot_days = wwvb_decode2( 33, 0xfff );
          calc_date();
          wwvb_data = save;
          mask = 0xf010;
       }   
    }
    else part_decode_year(i,dat);
  
}

// this will fail year 2080 as only looking at 8 bits
void part_decode_year( int i, uint8_t dat ){
static uint8_t mask = 0x10;
static uint8_t val;
uint8_t setb,clearb;
uint64_t save;

   if( i > 53 || i < 46 ) return;

   setb = 1;  clearb = 0xff;
   i = 53 - i;
   while( i-- ) setb <<= 1;
   clearb ^= setb;
   mask |= setb;
   if( dat ) val |= setb;
   else val &= clearb;

   if( mask == 0xff ){
          save = wwvb_data;
          wwvb_data = val;
          wwvb_data <<= 59-53;
          gyr = wwvb_decode2( 53, 0xff );
          wwvb_data = save;
          mask = 0x10;       
   }

}




**************************************************************************************************/
