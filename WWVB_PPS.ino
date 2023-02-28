
/*
 *   WWVB experiments
 *   
 */

#include <LCD5110_Basic.h>
LCD5110 LCD( 2,3,4,6,5 );

extern uint8_t SmallFont[];
extern uint8_t  MediumNumbers[];
extern uint8_t BigNumbers[];

#define WWVB_IN 7
#define PPS_OUT 8
 
long tm_correct_count = 60000;       // add or sub one ms for time correction per this many ms
int8_t tm_correction = 1;            // 0, 1 or -1 time correction
long time_adjust;
int gmon = 1,gday = 1,gyr = 1,ghr,gmin;
int tot_days = 1;
uint16_t leap = 1;
uint64_t wwvb_data, wwvb_sync, wwvb_errors;
uint8_t DST;                         // daylight savings bit

void setup() {
  
  pinMode(WWVB_IN, INPUT_PULLUP);     // sample wwvb receiver signal
  pinMode(PPS_OUT, OUTPUT);
  Serial.begin(9600);
  LCD.InitLCD();
  LCD.setFont(SmallFont);
  LCD.print("WWVB PPS TEST",CENTER,8*0);
  delay( 5000 );
  LCD.clrRow(0);
  
}

void loop() {
static unsigned long tm;

  if( tm != millis() ){
     tm = millis();
     wwvb_sample2(tm);
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

// WWVB receiver in a fringe area - integrate the signal to remove noise
// Although it probably makes more sense to dump the integrator 10 times per second, here we use 8.
// sample each millisecond, sum 100 or 150 samples , decide if low or high, shift into temp variable
// at end of 1 second( 8 bits), decide if temp has a 1, 0, or sync. Shift into 64 bit data and sync variables.
// when the sync variable contains the magic number, decode the 64 bit data.
// each second starts with a low signal and ends with a high signal
// much like software sampling rs232 start and stop bits.
// this routine runs fast by design until it locks on the wwvb signal( or slow depending upon point of view )

void wwvb_sample2(unsigned long t){
static unsigned long old_t;
int loops;
uint8_t b,s,e;
static uint8_t wwvb_clk, wwvb_sum, wwvb_tmp, wwvb_count;  // data decoding
const uint8_t counts[8] = { 100,100,150,150,150,150,100,100 };  // total of 1000 ms
static uint8_t secs,errors,early,late;
static uint8_t dither = 4;              // quick sync, adjusts to 1 when signal is good
static uint8_t enable_dither;           // enabled when think have a good wwvb signal
char ch[2];
//static char valp[9];       // LCD display of slow/fast timekeeping
//static int8_t vali, valc;

   loops = t - old_t;
   old_t = t;
   if( loops > 10 ) loops = 1;    // startup 
   
  while( loops-- ){   // repeat for any missed milliseconds

    // adjust for 16mhz millis() error
    if( ++time_adjust >= tm_correct_count && wwvb_clk != 0 ){
        time_adjust = 0;
        wwvb_clk += tm_correction;
    }

   if( digitalRead(WWVB_IN) == LOW ) ++wwvb_sum;

   if( --wwvb_clk == 0 ){    // end of period, dump integrator

      if( wwvb_sum == counts[wwvb_count] ) enable_dither = 1;      // good signal detect, allow slip in time
      
      b = ( wwvb_sum > (counts[wwvb_count] >> 1) ) ? 0 : 128;
      wwvb_tmp >>= 1;
      wwvb_tmp |= b;
      wwvb_sum = 0;

      // 8 dumps of the integrator is one second, decode this bit
      wwvb_count++;
      wwvb_count &= 7;
      if( wwvb_count == 0 ) digitalWrite( PPS_OUT,HIGH);
      if( wwvb_count == 1 ) digitalWrite( PPS_OUT,LOW);
      wwvb_clk = counts[wwvb_count];  // 100 100 150 150 150 150 100 100
                              // decode     0       1      sync    stop should be high
      if( wwvb_count == 0 ){    // decode time

        // clocks late or early, just dither them back and forth across the falling edge
        // when not in sync, more 1's than 0's are detected and this slips in time.
        
      if( wwvb_tmp != 0xff  && wwvb_tmp != 0x00  ){
         if( digitalRead(WWVB_IN) == 0 ){  
            ++late;                                // sampling late
            wwvb_clk -= enable_dither * dither;    // adjust sample to earlier
         }
         else{
            ++early;                               // need to sample later
            wwvb_clk += enable_dither * dither;    // longer clock
         }
      }
      
        // decode
        // 11111100 is a zero,  11110000 is a one, 11000000 is a sync      
        b = 0;  s = 0;  e = 1;   // assume it is an error
        pbin( wwvb_tmp );
        // strict decode works well, use loose decode for common bit errors ?
        if( wwvb_tmp == 0xfc || wwvb_tmp == 0xfd  ) e = 0, b = 0;
        if( wwvb_tmp == 0xf0  ) e = 0, b = 1;
        if( wwvb_tmp == 0xc0  ) e = 0, s = 1;

        ch[0] = 'e';   ch[1] = 0;
        if( e == 0 ){
           if( s == 1 ) ch[0] = 'S';
           else if( b == 1 ) ch[0] = '1';
           else ch[0] = '0';
        }
        LCD.print(ch,LEFT,0);

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
             secs = 59;              // secs = 0 next statement
          }
        }

        Serial.print( secs );  Serial.write(' ');
        if( secs == 29 || secs == 59 ) Serial.println();
        LCD.setFont(MediumNumbers);
        LCD.printNumI(secs,RIGHT,2*8,2,'0');
        LCD.setFont(SmallFont);
        
        if( ++secs >= 60 ){  //  adjust dither each minute
           dither = ( errors >> 4 ) + 1;

           // will this work for both slow and fast 16 mhz clock?
           if( errors < 32 ){
              //tm_correct_count += tm_correction * (late - early);  // ? which is correct ?
              tm_correct_count += tm_correction * (early - late);
              if( tm_correct_count > 120000 ){
                 tm_correct_count = 119000;
                 tm_correction *= -1;
              }
           }

           Serial.print( errors );  Serial.write(' ');
           Serial.print( tm_correction); Serial.write(' ');
           Serial.println( tm_correct_count );
           LCD.printNumI( tm_correct_count,LEFT,1*8,6,' ');
           LCD.printNumI( tm_correction,8*6,1*8,2,' ');
           LCD.printNumI(errors,RIGHT,1*8,2,' ');      

           enable_dither = early = late = secs = errors = 0;   // reset the stats for the next minute
           if( wwvb_errors > 0 ) keep_time();
        }

      }  // end decode time    
    }    // end integration timer
  }      // loops - repeat for lost milliseconds if any
}


void wwvb_decode(){   // WWVB transmits the data for the previous minute just ended
uint16_t tmp;
uint16_t tmp2;
uint16_t yr;
uint16_t hr;
uint16_t mn;
uint16_t dy;
uint8_t i;


  //tmp2 = frame_sec;
  //tmp = frame_msec;         // capture milliseconds value before it is corrected so we can print it.

  //++decodes;

  yr = wwvb_decode2( 53, 0x1ff );   // year is 0 to 99
  dy = wwvb_decode2( 33, 0xfff );   // day is 0 to 365/366
  hr = wwvb_decode2( 18, 0x7f );
  mn = wwvb_decode2( 8, 0xff );
  leap = wwvb_decode2( 55, 0x1 );
  DST  = wwvb_decode2( 57, 0x1 );    // in effect bit ( using bit 58 gave wrong time for one day )

/***
  if( ( mn & 1 ) == 0 ){    //last minute was even so just hit the 60 second mark in the frame
                            // only apply clock corrections in the middle of the two minute frame or may
                            // otherwise mess up the frame timing
         if( frame_sec == 59 && frame_msec >= 500 ) ;   // ok
         else if( frame_sec == 60 && frame_msec < 500 ) ;  // ok
         else{                                              // way off, reset to the correct time
            frame_sec = 60;
            frame_msec = 0;  
           // FF = 0, ff = 0;             // reset timing fudge factor
           // clr_trends = 1;             // the trend buckets will be incorrect now
         }
  }
  ****/
  
    
  ghr = hr;
  gmin = mn;
  gyr = yr;
  tot_days = dy;
  calc_date();
  keep_time();            // wwvb sends minute just ended info, so increment
  //calc_date( );
  //time_flags |= TS;

}

// wwvb fields all decode about the same way
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

   p_fill( gmon,2 );  Serial.write('/');
   p_fill( gday,2 );  Serial.write('/');
   Serial.print("20");  p_fill(gyr,2);  Serial.write(' '); Serial.write(' ');
   p_fill( ghr,2);  Serial.write(':');
   p_fill( gmin,2 );
   Serial.println();
   
   LCD.setFont(MediumNumbers);
   LCD.printNumI(ghr,LEFT,2*8,2,'0');
   LCD.printNumI(gmin,CENTER,2*8,2,'0');
   LCD.setFont(SmallFont);
   LCD.printNumI(gmon,0,5*8,2,'0');
   LCD.print("/",3*6,5*8);
   LCD.printNumI(gday,5*6,5*8,2,'0');
   LCD.print("/",8*6,5*8);
   LCD.printNumI(gyr,10*6,5*8,2,'0');
   


}

void p_fill( int val, int digits ){     // zero fill printing

   if( digits >= 4 && val < 1000 ) Serial.write('0');
   if( digits >= 3 && val < 100 ) Serial.write('0');
   if( digits >= 2 && val < 10 ) Serial.write('0');
   Serial.print(val);
}

void pbin( uint8_t val ){
int i;
uint8_t v;

   for( i = 7; i >=0; --i ){
      v = 0;
      if( val & _BV(i)) v = 1;
      LCD.printNumI( v, 78-6*i, 0*8);
   }
  
}
