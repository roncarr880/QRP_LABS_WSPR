
/*
 *   WWVB experiments
 *       WWVB as fake gps for U3S, how much will pps cause the tx frequency to be incorrect
 */

#include <LCD5110_Basic.h>
LCD5110 LCD( 2,3,4,6,5 );

extern uint8_t SmallFont[];
extern uint8_t  MediumNumbers[];
extern uint8_t BigNumbers[];

#define WWVB_IN 7
#define PPS_OUT 8


int gmon = 1,gday = 1,gyr = 1,ghr,gmin;
int tot_days = 1;
uint16_t leap = 1;
uint64_t wwvb_data, wwvb_sync, wwvb_errors;
uint8_t DST;                         // daylight savings bit
uint8_t gsec;
uint8_t msg_que = 1;
unsigned long old_t;

long time_adjust;                  // counter
long tm_correct_count = 30000;     // adjust one ms in this many ms
int8_t tm_correction = 1;          // +1 fast,  -1 slow

#define NUM_HISTORY 32
int phase;                         // lost second detection
// int tot_phase;                  // normal on time NANO, my NANO seems to run very fast
int tot_phase = 32*9;
//int phase_history[NUM_HISTORY];  // normal on time NANO
int phase_history[NUM_HISTORY] = { 9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9};  // testing
float rtotal;

void setup() {
int i;  
  pinMode(WWVB_IN, INPUT_PULLUP);     // sample wwvb receiver signal
  pinMode(PPS_OUT, OUTPUT);
  Serial.begin(9600);
  LCD.InitLCD();
  LCD.setFont(SmallFont);
  LCD.print("WWVB PPS TEST",CENTER,8*0);
  delay( 5000 );
  LCD.clrRow(0);
  old_t = millis();

}

void loop() {
static unsigned long tm;
static uint8_t msg;

  if( tm != millis() ){
     tm = millis();
     wwvb_sample2(tm);
  }

/*
  if( gsec == msg_que ){                  // sending one message per second to avoid overfilling the serial buffer
     switch( msg_que & 7 ){               // still some of these messages are larger than 64 characters
       case 1:  send_gga();  break;       // avoiding sending on second zero as there are a large amount of screen writes
       case 2:  send_gsv();  break;
       case 3:  send_gsa();  break;
       case 4:  send_rmc();  break;
       // case 5 6 7 available
     }
     if( ++msg_que >= 60 ) msg_que = 0;
  }
  */
  if( gsec == msg_que && Serial.availableForWrite() > 60 ){      // send all each second
     switch( msg ){
       case 0:  send_gga(); ++msg;  break;     // order sent must be just so, alphabetical I guess
       case 2:  send_gsv(); ++msg;  break;
       case 1:  send_gsa(); ++msg;  break;
       case 3:  send_rmc(); ++msg;  break;
       case 4:  ++msg_que;  
                if( msg_que >= 60 ) msg_que = 0;
                msg = 0;
                break;
     }
    
  }

}

void send_gga(){

  gps_puts( "$GPGGA," );
  send_num( ghr ); send_num( gmin ); send_num( gsec );
  gps_puts( ".20," );
  gps_puts( "4426.8053,N,");        // lat
  gps_puts( "06931.4612,W,");       // long
  gps_puts( "1,");                  // fix
  gps_puts( "04,");                 // num sats
  gps_puts( "1.5,");                // horizontal something
  gps_puts( "96.8,M," );            // altitude
  gps_puts( "-34.0,M," );
  gps_puts( ",*" );
}

void send_gsv(){

  gps_puts( "$GPGSV," );
  gps_puts( "1,1,04," );              // messages, sats in view
  gps_puts( "04,44,104,24," );       // prn,elevation,azimuth,snr
  gps_puts( "05,45,105,25," );       // prn,elevation,azimuth,snr
  gps_puts( "06,46,106,26," );       // prn,elevation,azimuth,snr
  gps_puts( "07,47,107,27*" );       // prn,elevation,azimuth,snr
}

void send_gsa(){

  gps_puts( "$GPGSA," );
  gps_puts( "A,3," );                // 3d fix
  gps_puts( "04,05,06,07," );        // ID's
  gps_puts( ",,,,,,,," );          // unused slots
  gps_puts( "1.7,1.1,1.3*" );        // pdop,hdop,vdop
}

void send_rmc(){

  gps_puts( "$GPRMC," );
  send_num( ghr ); send_num( gmin ); send_num( gsec );
   gps_puts( ".20," );
  gps_puts( "A," );
  gps_puts( "4426.8053,N,");        // lat
  gps_puts( "06931.4612,W,");       // long
  gps_puts( "000.5,054.7,");        // speed,course
  send_num( gday );  send_num(gmon);  send_num( gyr );   // date
  gps_puts( ",018.1,W,A*" );          // mag declination, A or D?
}

void send_num( int val ){
char buf[30];

   if( val < 10 ) gps_putch('0');
   itoa( val, buf, 10 );
   gps_puts( buf );
}


void gps_putch( unsigned char c ){
static uint8_t  crc;

   if( c == '$' ) crc = 0;
   else if( c != '*' ) crc ^= c;
   
   Serial.write( c );
   if( c == '*' ){
      send_hex( crc );
      Serial.println();
   }
}

void gps_puts( char * p ){
char c;

   while( ( c = *p++ ) ) gps_putch( c ); 
}

void send_hex( uint8_t d ){
char buf[30];

   itoa( d, buf, 16 );
   buf[0] = toupper( buf[0] );
   buf[1] = toupper( buf[1] );
   gps_puts( buf );
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
int loops;
uint8_t b,s,e;
static uint8_t wwvb_clk, wwvb_sum, wwvb_tmp, wwvb_count;  // data decoding
const uint8_t counts[8] = { 100,100,150,150,150,150,100,100 };  // total of 1000 ms
static uint8_t secs,errors,early,late;
static uint8_t dither = 4;              // quick sync, adjusts to 1 when signal is good
char ch[2];

   loops = t - old_t;
   old_t = t;
   
  while( loops-- ){   // repeat for any missed milliseconds

    // adjust for 16mhz millis() error
    if( ++time_adjust >= tm_correct_count && wwvb_clk > 100 ){
        time_adjust -= tm_correct_count;
        wwvb_clk += tm_correction;
    }


   if( digitalRead(WWVB_IN) == LOW ) ++wwvb_sum;

   if( --wwvb_clk == 0 ){    // end of period, dump integrator
      
      b = ( wwvb_sum > (counts[wwvb_count] >> 1) ) ? 0 : 128;
      wwvb_tmp >>= 1;
      wwvb_tmp |= b;
      wwvb_sum = 0;

      // 8 dumps of the integrator is one second, decode this bit
      wwvb_count++;
      wwvb_count &= 7;
      if( wwvb_count == 1 && rtotal < 250.0 && rtotal > -250.0) digitalWrite( PPS_OUT,HIGH);
      if( wwvb_count == 2 ) digitalWrite( PPS_OUT,LOW), gsec = secs;     // que serial messages

      wwvb_clk = counts[wwvb_count];  // 100 100 150 150 150 150 100 100
                              // decode     0       1      sync    stop should be high
      if( wwvb_count == 0 ){    // decode time

        // clocks late or early, just dither them back and forth across the falling edge
        // when not in sync, more 1's than 0's are detected and this slips in time.
        
      if( wwvb_tmp != 0xff  && wwvb_tmp != 0x00  ){

         if( digitalRead(WWVB_IN) == 0 ){  
            ++late;                                // sampling late
            wwvb_clk -= dither;                 // adjust sample to earlier
         }
         else{
            ++early;                               // need to sample later     
            wwvb_clk +=  dither;                   // longer clock            
         }
      }
      
        // decode
        // 11111100 is a zero,  11110000 is a one, 11000000 is a sync      
        b = 0;  s = 0;  e = 1;   // assume it is an error
        pbin( wwvb_tmp );
        // strict decode 
        if( wwvb_tmp == 0xfc  ) e = 0, b = 0;
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

        //Serial.print( secs );  Serial.write(' ');
        //if( secs == 29 || secs == 59 ) Serial.println();
        LCD.setFont(MediumNumbers);
        LCD.printNumI(secs,RIGHT,2*8,2,'0');
        LCD.setFont(SmallFont);
        
        if( ++secs >= 60 ){  //  adjust dither each minute
           dither = ( errors >> 4 ) + 1;

           phase = early-late;

           if( errors < 40 ){
           // will this work for both slow and fast 16 mhz clock?
           // adjust correction for the 16 mhz nano clock
              tm_correct_count += tm_correction * (late - early);  // ? which is correct ?
              //tm_correct_count += tm_correction * (early - late);
              if( tm_correct_count > 60000 ){
                 tm_correct_count =  59000;
                 tm_correction *= -1;
              }
           }
           
           //Serial.print( errors );  Serial.write(' ');
           //Serial.print( tm_correction); Serial.write(' ');
           //Serial.println( tm_correct_count );
           LCD.printNumI( tm_correct_count,LEFT,1*8,6,' ');
           LCD.printNumI( tm_correction,8*6,1*8,2,' ');
           LCD.printNumI(errors,RIGHT,1*8,2,' ');
           
           early = late = secs  = 0;   // reset the stats for the next minute
           secs += save_phase_hist(phase,errors);
           errors = 0;
           LCD.printNumF( (float)tot_phase/(float)NUM_HISTORY,2,RIGHT,4*8,'.',6,' ' );
           LCD.printNumI( (int)rtotal,LEFT,4*8,5,' ' );
           phase = 0;       
           if( wwvb_errors > 0 ) keep_time();
        }

      }  // end decode time    
    }    // end integration timer
  }      // loops - repeat for lost milliseconds if any
}


// save correction history, adjust seconds if think lost one
int8_t save_phase_hist(int phase, int errors){
int i;

   if( errors < 45 ){                           // assume in phase if receiving some good data, save expected data
      tot_phase = phase;
      for( i = 0; i < NUM_HISTORY-1; ++i ){
         phase_history[i] = phase_history[i+1];
         tot_phase += phase_history[i];
      }
      phase_history[NUM_HISTORY-1] = phase;
      rtotal = 0.0;
   }
   else{
      rtotal -= (float)tot_phase / (float)NUM_HISTORY;       // sub expected
      rtotal += phase;                         // add new
      if( rtotal > 1000.0 ){
        rtotal -= 1000.0;
        return 1;                              // lost a second?
      }
   }

   return 0;
}



void wwvb_decode(){   // WWVB transmits the data for the previous minute just ended
uint16_t tmp;
uint16_t tmp2;
uint16_t yr;
uint16_t hr;
uint16_t mn;
uint16_t dy;
uint8_t i;


  yr = wwvb_decode2( 53, 0x1ff );   // year is 0 to 99
  dy = wwvb_decode2( 33, 0xfff );   // day is 0 to 365/366
  hr = wwvb_decode2( 18, 0x7f );
  mn = wwvb_decode2( 8, 0xff );
  leap = wwvb_decode2( 55, 0x1 );
  DST  = wwvb_decode2( 57, 0x1 );    // in effect bit ( using bit 58 gave wrong time for one day )  
    
  ghr = hr;
  gmin = mn;
  gyr = yr;
  tot_days = dy;
  calc_date();
  keep_time();            // wwvb sends minute just ended info, so increment

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

   //p_fill( gmon,2 );  Serial.write('/');
   //p_fill( gday,2 );  Serial.write('/');
   //Serial.print("20");  p_fill(gyr,2);  Serial.write(' '); Serial.write(' ');
   //p_fill( ghr,2);  Serial.write(':');
   //p_fill( gmin,2 );
   //Serial.println();
   
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

/***
void p_fill( int val, int digits ){     // zero fill printing

   if( digits >= 4 && val < 1000 ) Serial.write('0');
   if( digits >= 3 && val < 100 ) Serial.write('0');
   if( digits >= 2 && val < 10 ) Serial.write('0');
   Serial.print(val);
}
***/

void pbin( uint8_t val ){
int i;
uint8_t v;

   for( i = 7; i >=0; --i ){
      v = 0;
      if( val & _BV(i)) v = 1;
      LCD.printNumI( v, 78-6*i, 0*8);
   }
  
}
