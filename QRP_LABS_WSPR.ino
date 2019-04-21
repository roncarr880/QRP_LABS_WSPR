
// QRP_LABS_WSPR
//   Arduino, QRP Labs Arduino shield, SI5351 clock, QRP Labs RX module, QRP Labs relay board.
//   NOTE:  The tx bias pot works in reverse, fully clockwise is off.

//   The CAT emulation is TenTec Argonaut V at 1200 baud.

//   To set a new operation frequency for stand alone Frame Mode, restart the Arduino, start wsjt-x or HRD.
//   Tune to one of the magic WSPR frequencies and toggle TX (tune in wsjt will work).
//   The new frequency will be stored in EEPROM.
//   If using the band hopping feature of WSJT, make sure the first transmit is on the band you wish for
//   the default.  Only one save is performed per Arduino reset. 
//
//   A 4:1 frequency relationship between the tx freq and the rx clock is maintained using the
//   R dividers in the SI5351.  Dividers 1 - rx and 4 - tx will cover 1mhz to 30mhz
//   Dividers 16 - rx and 64 - tx will cover 40 khz to 2 mhz

 
#include <Wire.h>
#include <FreqCount.h>   // one UNO I have does not work correctly with this library, another one does
#include <EEPROM.h>

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
#define FRAME_MODE 1    // or self timed frame (stand alone mode)
#define MUTE  A1        // receiver module T/R switch pin

#define stage(c) Serial.write(c)

  // using even dividers between 6 and 254 for lower jitter
  // freq range 2 to 150 without using the post dividers
  // we are using the post dividers and can receive down to 40khz
  // vco 600 to 900
  
uint64_t clock_freq = 2700452200;    // * 100 to enable setting fractional frequency
uint32_t freq = FREQ;                // ssb vfo freq
const uint32_t cal_freq = 3000000;   // calibrate frequency
const uint32_t cal_divider = 200;
uint32_t divider = DIV;
uint32_t audio_freq = 1528;          // wspr 1400 to 1600 offset from base vfo freq 
uint8_t  Rdiv = RDIV; 

uint8_t operate_mode = FRAME_MODE;   // start in stand alone timing mode
uint8_t wspr_tx_enable;              // transmit enable
uint8_t wspr_tx_cancel;              // CAT control RX command cancels tx
uint8_t cal_enable;
long tm_correct_count = 10753;       // add or sub one ms for time correction per this many ms
int8_t tm_correction = 0;            // 0, 1 or -1 time correction

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

// WWVB receiver in a fringe area - integrate the signal to remove noise
// Although it probably makes more sense to dump the integrator 10 times per second, here we use 8.
// sample each millisecond, sum 100 or 150 samples , decide if low or high, shift into temp variable
// at end of 1 second( 8 bits), decide if temp has a 1, 0, or sync. Shift into 64 bit data and sync variables.
// when the sync variable contains the magic number, decode the 64 bit data.
#define WWVB_OUT 9
#define WWVB_PWDN 8
uint64_t wwvb_data, wwvb_sync, wwvb_errors;

uint8_t wwvb_quiet = 0;  // wwvb debug print flag, set to 2 for normal use to avoid all printing
uint8_t wwvb_stats[8];   // bit distribution over 60 seconds

uint8_t frame_sec;    // frame timer counts 0 to 120
int frame_msec;

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
  
  Serial.begin(1200);      // TenTec Argo V baud rate
  Wire.begin();
// Wire.setClock(400000);  // should work but clock 1 starts on the wrong frequency

  ee_restore();            // get default freq for frame mode from eeprom

  pinMode(MUTE,OUTPUT);    // receiver t/r switch
  digitalWrite(MUTE,LOW);  // enable the receiver
  pinMode(WWVB_OUT, INPUT);
  pinMode(WWVB_PWDN, OUTPUT);
  digitalWrite(WWVB_PWDN,LOW);  // enable wwvb receiver

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
  band_change();                           // select the correct relay
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
  
}

void loop() {
static unsigned long ms;

   if( Serial.availableForWrite() > 20 ) radio_control();
   
   if( ms != millis()){     // run once each ms
       ms = millis();
       frame_timer(ms);

       if( wspr_tx_enable || wspr_tx_cancel ) wspr_tx(ms);
       if( cal_enable ) run_cal();
       wwvb_sample(ms);
   }

}


// each second starts with a low signal and ends with a high signal
// much like software sampling rs232 start and stop bits.
void wwvb_sample(unsigned long t){
static unsigned long old_t;
int loops;
uint8_t b,s,e;
static uint8_t wwvb_clk, wwvb_sum, wwvb_tmp, wwvb_count;  // data decoding
const uint8_t counts[8] = { 100,100,150,150,150,150,100,100 };  // total of 1000 ms
static uint8_t secs,errors,early,late;   // debug use
static uint16_t decodes;
static uint8_t dither = 4;   // quick sync

   loops = t - old_t;
   old_t = t;
   
  while( loops-- ){   // repeat for any missed milliseconds  

   if( digitalRead(WWVB_OUT) == LOW ) ++wwvb_sum;

   if( --wwvb_clk == 0 ){    // end of period, dump integrator
      b = ( wwvb_sum > 50 ) ? 0 : 128;
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
      if( wwvb_tmp != 0xff  && wwvb_tmp != 0x00  ){
         if( digitalRead(WWVB_OUT) == 0 ){  
            ++late;               // sampling late
            wwvb_clk -= dither;        // adjust sample to earlier
         }
         else{
            ++early;              // need to sample later
            wwvb_clk += dither;        // longer clock ( more of these as arduino runs fast )
         }
      }
      
      gather_stats( wwvb_tmp );

        // decode
        // 11111100 is a zero,  11110000 is a one, 11000000 is a sync      
        b = 0;  s = 0;  e = 1;   // assume it is an error
        if( wwvb_tmp == 0xfc ) e = 0, b = 0;
        if( wwvb_tmp == 0xf0 ) e = 0, b = 1;
        if( wwvb_tmp == 0xc0 ) e = 0, s = 1;
        wwvb_data <<= 1;   wwvb_data |= b;
        wwvb_sync <<= 1;   wwvb_sync |= s;
        wwvb_errors <<= 1; wwvb_errors |= e;
        if( e ) ++errors;

        if( wwvb_quiet < 30 ){   // - debug print out some stats when not in CAT mode
           if( ++secs == 60 ){
               Serial.print("Errors "); Serial.print(errors);
               Serial.print("  Early "); Serial.print(early);
               Serial.print("  Late ");  Serial.print(late);
               print_stats();
               Serial.print("  Decodes "); Serial.println(decodes);
               dither = ( errors > 30 ) ? 4 : 1;
               early = late = secs = errors = 0;
           }
        }
        // magic 64 bits of sync  ( looking at 60 seconds of data with 4 seconds of the past minute )
        // xxxx1000000001 0000000001 0000000001 0000000001 0000000001 0000000001
        // wwvb_sync &= 0x0fffffffffffffff;   // mask off the old bits from previous minute
        // instead of masking, use the old bits to see the double sync bits at 0 of this minute
        // and 59 seconds of the previous minute.  This decodes at zero time rather than some
        // algorithms that decode at 1 second past.
        if( wwvb_sync == 0b0001100000000100000000010000000001000000000100000000010000000001 ){
          if( wwvb_errors == 0 ){    // decode on no bit errors
            wwvb_decode();
            ++decodes;
          }
        }
      }    
   }
  }
}

void gather_stats( uint8_t data ){
uint8_t i;

   for( i = 0; i < 8; ++i ){
      if( data & 1 ) ++wwvb_stats[i];
      data >>= 1;
   } 
}


void print_stats(){
uint8_t i;

   Serial.print("  Data ");
   for( i = 7;  i < 8; --i ){
      if( wwvb_stats[i] > 50 ) Serial.write('1');
      else if( wwvb_stats[i] < 10 ) Serial.write('0');
      else if( wwvb_stats[i] > 30 ) Serial.write('X');
      else Serial.write('x');
      wwvb_stats[i] = 0;
   }
}



void wwvb_decode(){   // decodes the previous minute
uint16_t tmp;
uint8_t yr;
uint8_t hr;
uint8_t mn;
uint8_t dy;

  // can set frame_sec to correct value here( 0 or 60 depending if odd or even minute )
  // if decode an odd minute, that is the start of the frame since the decoded minute is the previous one
  // !!! should zero the milli counter also which is currently a static, need global
  // can keep a count of how long it takes to gain or loose and adjust
  // the value of the 27 mhz clock

  tmp = ( wwvb_data >>( 59 - 53) ) & 0x1ff;
  yr = 0;
  if( tmp & 0x100 ) yr += 80;
  if( tmp & 0x80 ) yr += 40;
  if( tmp & 0x40 ) yr += 20;
  if( tmp & 0x20 ) yr += 10;
  yr += tmp & 0xf;

  tmp = ( wwvb_data >> ( 59 - 33 ) ) & 0xfff;
  dy = 0;
  if( tmp & 0x800 ) dy += 200;
  if( tmp & 0x400 ) dy += 100;
  if( tmp & 0x100 ) dy += 80;
  if( tmp & 0x80 ) dy += 40;
  if( tmp & 0x40 ) dy += 20;
  if( tmp & 0x20 ) dy += 10;
  dy += tmp & 0xf;

  tmp = ( wwvb_data >> ( 59 - 18 ) ) & 0x3f;
  hr = 0;
  if( tmp & 0x40 ) hr += 20;
  if( tmp & 0x20 ) hr += 10;
  hr += tmp & 0xf;

  tmp = ( wwvb_data >> ( 59 - 8 ) ) & 0xff;
  mn = 0;
  if( tmp & 0x80 ) mn += 40;
  if( tmp & 0x40 ) mn += 20;
  if( tmp & 0x20 ) mn += 10;
  mn += tmp & 0xf;

  // sync the frame timer to wwvb, don't change time at the start or end of the frame or
  // may skip frames or repeat frames
  if( ( mn & 1 ) == 0 ){    //last minute was even so just hit the 60 second mark in the frame
    if( frame_sec > 20 && frame_sec < 100 ){   // just in case there was a bit error in the minute field
      frame_sec = 60;
      frame_msec = 0;
    }  
  }

  if( wwvb_quiet > 30 ) return;    // avoid printing on the serial port, interfers with radio_control
  Serial.print(frame_sec);  Serial.write(' ');
  Serial.print("WWVB SYNC   ");
  Serial.print("20");  // the year 2100 bug
  Serial.print( yr );  Serial.write(' ');
  Serial.print( dy );  Serial.write(' ');
  Serial.print( hr );  Serial.write(':');
  Serial.println(mn);
}



// the original idea was to correct the 27 mhz clock using the UNO 16 mhz clock as a reference.
// calibrating the SI5351 against the 16mhz clock does not seem to be viable.
// the 16mhz clock varies as much or more than the 27mhz clock with changes in temperature
// this function has been changed to correct the time keeping of the 16 meg clock based upon the 27 mhz reference
// this seems to be working very well with no change in WSPR received delta time for over 48 hours.
void run_cal(){    // count pulses on clock 2 wired to pin 5
                   // IMPORTANT: jumper W4 to W7 on the arduino shield
unsigned long result;
long error;


   if( cal_enable == 1 ){
       FreqCount.begin(1000);   //
       ++cal_enable;
   }

   if( FreqCount.available() ){
       result = FreqCount.read();
       if( result < 3000000L ) tm_correction = -1, error = 3000000L - result;
       else if( result > 3000000L ) tm_correction =  1, error = result - 3000000L;
       else tm_correction = 0, error = 1500;                    // avoid divide by zero
       if( error < 2000 ) tm_correct_count = 3000000L / error;
       else tm_correction = 0;                   // defeat correction if freq counted is obviously wrong
       //Serial.println( result );
       //Serial.println(tm_correction);
       //Serial.println(tm_correct_count);
       FreqCount.end();
       cal_enable = 0;
   }  
}

void frame_timer( unsigned long t ){   
static unsigned long old_t;
static uint8_t slot;

static int time_adjust;   
// 16mhz clock measured at 16001111.  Will gain 1ms in approx 14401 ms.  Or 1 second in 4 hours.
// the calibrate function has been repurposed to correct the time keeping of the Arduino.

   frame_msec += ( t - old_t );
    time_adjust += (t - old_t);
    if( time_adjust >= tm_correct_count && frame_msec != 0 ) time_adjust = 0, frame_msec += tm_correction;
   old_t = t;
   if( frame_msec >= 1000 ){
      frame_msec -= 1000;
      if( ++frame_sec >= 120 ){     // 2 minute slot time
        frame_sec -= 120;
        if( ++slot >= 7 ) slot = 0;   // 10 slots is a 20 minute frame
        if( slot == 1 && operate_mode == FRAME_MODE ) wspr_tx_enable = 1;
      }
      if( frame_sec == 118 && operate_mode == FRAME_MODE ) cal_enable = 1;   // do once per slot in wspr quiet time
   } 
}

void wspr_tx( unsigned long t ){
static int i;
static unsigned long timer;
static uint8_t mod;

   if( wspr_tx_cancel ){      // quit early or just the end of the message
      if( i < 160 ) i = 162;  // let finish if near the end, else force done
   }
   
   if( i != 0 && (t - timer) < 683 ) return;   // baud time is 682.66666666 ms
   timer = t;
   ++mod;   mod &= 3;
   if( mod == 0 ) ++timer;    // delay 683, 683, 682, etc.

   if( i == 162 ){
      tx_off();
      i = 0;                 // setup for next time to begin at zero index
      wspr_tx_cancel = wspr_tx_enable = 0;    // flag done
      return;
   }
   // set the frequency
   si_pll_x(PLLA,Rdiv*4*(freq+audio_freq),divider,Rdiv*4*146*wspr_msg[i]);
   if( i == 0 ) tx_on();
   ++i; 
}

void tx_on(){

  digitalWrite(MUTE,HIGH);
  digitalWrite(WWVB_PWDN,HIGH);   
  i2cd(SI5351,3,0xff ^ (CLK0_EN));   // tx clock on, other clocks off during tx
}

void tx_off(){
  
    i2cd(SI5351,3,0xff ^ (CLK1_EN + CLK2_EN) );   // turn off tx, turn on rx and cal clocks
    si_pll_x(PLLA,Rdiv*4*freq,divider,0);         // return to RX frequency
    digitalWrite(MUTE,LOW);                       // enable receiver
    digitalWrite(WWVB_PWDN,LOW);                  // enable wwvb receiver

    ee_save();     // save this freq to use during stand alone mode(FRAME MODE).
}


void i2cd( unsigned char addr, unsigned char reg, unsigned char dat ){
  // direct register writes.  A possible speed up could be realized if one were
  // to use the auto register inc feature of the SI5351
   Wire.beginTransmission(addr);
   Wire.write(reg);
   Wire.write(dat);
   Wire.endTransmission();
}


/******   SI5351  functions   ******/

void  si_pll_x(unsigned char pll, uint32_t freq, uint32_t out_divider, uint32_t fraction ){
 uint64_t a,b,c;
 uint64_t bc128;             // floor 128 * b/c term of equations
 uint64_t pll_freq;

 uint32_t P1;            // PLL config register P1
 uint32_t P2;            // PLL config register P2
 uint32_t P3;            // PLL config register P3
 uint64_t r;
   
   c = 1000000;     // max 1048575
   pll_freq = 100ULL * (uint64_t)freq + fraction;    // allow fractional frequency for wspr
   pll_freq = pll_freq * out_divider;
   a = pll_freq / clock_freq ;
   r = pll_freq - a * clock_freq ;
   b = ( c * r ) / clock_freq;
   bc128 =  (128 * r)/ clock_freq;
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
      operate_mode = CAT_MODE;   // switch modes on query cat command
      if( wwvb_quiet < 40 ) ++wwvb_quiet;  // allow some manual CAT commands before shutting off wwvb debug
    }
    if( cmd == '*' )  set_cmd();
    if( cmd == '#' )  pnd_cmd(); 

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
       stage(wspr_tx_enable);
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



