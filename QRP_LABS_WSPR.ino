
// QRP_LABS_WSPR
//   Arduino, QRP Labs Arduino shield, SI5351 clock, QRP Labs RX module
//   NOTE:  The tx bias pot works in reverse, fully clockwise is off.

//   The CAT emulation is TenTec Argonaut V at 1200 baud.

//   To set a new operation frequency for stand alone Frame Mode, start wsjt-x or HRD.
//   Tune to one of the magic WSPR frequencies and toggle TX (tune in wsjt will work).
//   The new frequency will be stored in EEPROM.
//   If using the band hopping feature of WSJT, disable the EEPROM writes, function ee_save(). 
//
//   A 4:1 frequency relationship between the tx freq and the rx clock is maintained using the
//   post dividers aka R dividers in the SI5351.  Dividers 1 - rx and 4 - tx will cover 1mhz to 30mhz
//      Dividers 16 - rx and 64 - tx will cover 40 khz to 2 mhz

 
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
uint32_t audio_freq = 1538;          // wspr 1400 to 1600 offset from base vfo freq 
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
//struct BAND band_info[6] = {    // filter
//  {  7,   40000,   600000 },    // 630m
//  { A0,  600000,  2500000 },    // 160m
//  { 10, 2500000,  5000000 },    // 80m
//  { 11, 5000000, 11500000 },    // 30m
//  { 12,11500000, 20000000 },    // 17m
//  { A3,20000000, 30000000 }     // 10m
//};  

// I don't have a 160m filter yet, so this is a modified table
struct BAND band_info[6] = {    // filter
  {  7,   40000,   600000 },    // 630m
  { A0,  600000,  600001 },    // 160m
  { 10,  600001,  5000000 },    // 80m
  { 11, 5000000, 11500000 },    // 30m
  { 12,11500000, 20000000 },    // 17m
  { A3,20000000, 30000000 }     // 10m
};  

// wspr frequencies for eeprom save routine.  Only these will be saved.
const uint32_t magic_freq[10] = {
  474200, 1836600, 3568600, 7038600, 10138700, 14095600, 18104600, 21094600, 24924600, 28124600
};

void ee_save(){ 
uint8_t i;
static uint8_t last_i = 255;

  //  return;   // uncomment if using the frequency hopping feature ( and transmitting )
                // to avoid wearing out the eeprom
                
  for( i = 0; i < 10; ++i ){
    if( freq == magic_freq[i] ) break;
  }
  if( i == 10 ) return;      // not a wspr frequency
  if( i == last_i ) return;  // already wrote this one

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
}

void setup() {
uint8_t i;
  
  Serial.begin(1200);      // TenTec Argo V baud rate
  Wire.begin();
  Wire.setClock(400000);

  ee_restore();            // get default freq for frame mode from eeprom

  pinMode(MUTE,OUTPUT);    // receiver t/r switch
  digitalWrite(MUTE,LOW);  // enable the receiver

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
static uint32_t old_freq = FREQ;

   divf = 0;   // flag if we need to reset the PLL's
   if( abs((int32_t)old_freq - (int32_t)new_freq) > 500000){
       divf = 1;    // large qsy from our current dividers
       old_freq = new_freq;
   }
   freq = new_freq;
   if( band_change() ) divf = 1;    // check the proper relay is selected

   // force freq above a lower limit
   if( freq < 40000 ) freq = 40000;
   
   if( freq > 2000000 && Rdiv != 1 ) Rdiv = 1, divf = 1;     // tx Rdiv is 4
   if( freq < 1000000 && Rdiv != 16 ) Rdiv = 16, divf = 1;   // tx Rdiv is 64
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
   }

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
static int msec;
static unsigned long old_t;
static uint8_t slot;
static uint8_t sec;
static int time_adjust;   
// 16mhz clock measured at 16001111.  Will gain 1ms in approx 14401 ms.  Or 1 second in 4 hours.
// the calibrate function has been repurposed to correct the time keeping of the Arduino.

   msec += ( t - old_t );
    time_adjust += (t - old_t);
    if( time_adjust >= tm_correct_count && msec != 0 ) time_adjust = 0, msec += tm_correction;
   old_t = t;
   if( msec >= 1000 ){
      msec -= 1000;
      if( ++sec >= 120 ){     // 2 minute slot time
        sec -= 120;
        if( ++slot >= 6 ) slot = 0;   // 10 slots is a 20 minute frame
        // Serial.print(F("Slot ")); Serial.println(slot);
        if( slot == 1 && operate_mode == FRAME_MODE ) wspr_tx_enable = 1;
        // enable other modes in different slots
      }
      if( sec == 118 && operate_mode == FRAME_MODE ) cal_enable = 1;   // do once per slot in wspr quiet time
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
  i2cd(SI5351,3,0xff ^ (CLK0_EN));   // tx clock on, other clocks off during tx
}

void tx_off(){
  
    i2cd(SI5351,3,0xff ^ (CLK1_EN + CLK2_EN) );   // turn off tx, turn on rx and cal clocks
    si_pll_x(PLLA,Rdiv*4*freq,divider,0);         // return to RX frequency
    digitalWrite(MUTE,LOW);                       // enable receiver

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
       if( len == 1 ) cmd = c;  /* first char */
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
        
    if( cmd == '?' )  get_cmd(), operate_mode = CAT_MODE;   // switch modes on query cat command
    if( cmd == '*' )  set_cmd();
    if( cmd == '#' )  pnd_cmd(); 

 /* prepare for next command */
   len = expect_len= 0;
  // if( cmd != '?' ){   // does wsjt need to see the G returned for query commands?.  Yes it does.
     stage('G');       /* they are all good commands */
     stage('\r');
  // }
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
long arg;
char val1;
int  val2;
unsigned long val4;

   cmd2 = command[1];
   switch(cmd2){
    case 'X':   stage_str("RADIO START"); stage('\r'); break; 
    case 'O':   /* split */ 
       val1 = command[2];
     //  if( val1 && user[SPLIT] ) val1 = 0;      /* toggle from HRD instead of zero to shut off? */
     //  if( val1 ) split= user[SPLIT] = 1;
     //  else{
     //    if( split == 2 ) rx_vfo = tx_vfo;
     //    else tx_vfo = rx_vfo;
     //    split= user[SPLIT] = user[SWAPVFO] = 0;
      // }
    break;
    case 'A':
    case 'B':
       val4 = get_long();
      // if( mode == CW && fun_selected[0] != WIDE ) val4 = val4 - (( sideband == USB ) ? mode_offset : - mode_offset);            
      // cat_band_change(val4);    // check for band change
      // if( cmd2 == 'B' ) tx_vfo = val4;
      // else{
      //     rx_vfo = val4;
      //     if( split == 0 ) tx_vfo = val4;
      // }
      qsy(val4);       
    break;
    case 'E':
       if( command[2] == 'V' ) vfo = command[3];
    break;
    case 'W':    /* bandwidth */
       val1 = command[2];
   //    if( val1 < 12 ) fun_selected[0] = NARROW;   /* narrow */
   //    else if( val1 > 23 ) fun_selected[0] = WIDE;  /* wide */
   //    else fun_selected[0] = MEDIUM;
   //    set_band_width(fun_selected[0]);
    break;
    case 'K':            /* putting keying speed on the Noise Blanker slider. Range is 10 to 19 */
                         /* or could be easily doubled for a range of 10 to 28 - if so change the get cmd also*/
     //  wpm = command[2] + 10;
    break;
    case 'T':            /* added tuning rate as a command */
      // set_tuning_rate(command[2]);
      // fun_selected[1] = command[2];
    break;       
    
   }  /* end switch */

   //update_frequency(DISPLAY_UPDATE);
   
   //write_sleds(sleds[fun_selected[function]]);
   //write_fleds(fleds[function], 1);  /* update on/off status  on FGRN led */
   //led_on_timer = 1000;  
   
}

void get_cmd(){
char cmd2;
long arg;
int bat;
int len;

   cmd2 = command[1];
//   nope breaks HRD also stage(command[0]);    // does wsjt need to see the question mark ?   
   stage(cmd2);
   switch(cmd2){
    case 'A':  //arg= rx_vfo;
    case 'B': 
      arg = freq;
      // if( cmd2 == 'B' ) arg= tx_vfo;
      // if( mode == CW && fun_selected[0] != WIDE ) arg = arg + (( sideband == USB ) ? mode_offset : - mode_offset);           
       stage_long(arg);
    break;
    case 'V':   /* version */
     //  stage(' ');
     //  bat = 135;   // battery(0);
     //  stage_num(bat/10);
     //  stage('.');
      // stage_num(bat % 10);
      // if( user[SWAPVFO] ) stage_str(" SWAP"); 
      stage_str("ER 1010-516");
    break;
    case 'W':          /* receive bandwidth */
       stage(30);   //stage(40 - fun_selected[0] * 10 );
    break;
    case 'M':          /* mode */
       stage('1'); stage('1');
    break;
    case 'O':          /* split */   
       //if( split ) stage(1);
       //else stage(0);
       stage(0);
    break;
    case 'P':         /*  passband slider */
       stage_int( 3000 );
    break;
    case 'T':         /* added tuning rate command */
       //stage( fun_selected[1] );
    break;   
    case 'E':         /* vfo mode */
       stage('V');
      // if( split == 2 ) stage('B');
      // else
      stage(vfo);
    break;
    case 'S':         /* signal strength */
       stage(7);
       stage(0);
    break;
    case 'C':
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
 //   stage('\n');    // does wsjt need to see line feeds ?  
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



