
// QRP_LABS_WSPR
//   Arduino, QRP Labs Arduino shield, SI5351 clock, QRP Labs RX module

#include <Wire.h>

#define SI5351   0x60    // i2c address
#define PLLA 26   // register address offsets for PLL's
#define PLLB 34
#define CLK0_EN   1
#define CLK1_EN   2
#define CLK2_EN   4

#define CAT_MODE  0     // computer control of TX
#define FRAME_MODE 1    // or self timed frame (stand alone mode)

  // use even dividers between 6 and 254 for lower jitter
  // freq range 2 to 150 without using the post dividers
  // vco 600 to 900
  
uint64_t clock_freq = 2700368000;  // * 100 to enable setting fractional frequency
uint32_t freq = 7038600;       // ssb vfo freq
const uint32_t cal_freq = 3000000;   // calibrate frequency
const uint32_t cal_divider = 200;
uint32_t divider = 14;        //  7 mhz with Rdiv of 8, 28 mhz with Rdiv of 2
uint32_t audio_freq = 1500;   // wspr 1400 to 1600 offset from base vfo freq 
uint8_t  Rdiv = 2; 

uint8_t  operate_mode = FRAME_MODE;   // start in stand alone timing mode


void setup() {
int i;
  
  Serial.begin(38400);
  Wire.begin();
  Wire.setClock(400000);


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

  //si_pll_x(PLLA,freq+audio_freq,4*divider,0);   // transmit
  //si_load_divider(4*divider,0,0);

  si_pll_x(PLLB,cal_freq,cal_divider,0);   // calibrate frequency on clock 2
  si_load_divider(cal_divider,2,0,1);

  si_pll_x(PLLA,Rdiv*4*freq,divider,0); // receiver 4x clock
  si_load_divider(divider,0,0,Rdiv*4);  // TX clock 1/4th of the RX clock
  si_load_divider(divider,1,1,Rdiv);    // load divider for clock 1 and reset pll's
  
//  i2cd(SI5351,3,0xff ^ (CLK1_EN + CLK2_EN) );   // turn on clocks receiver and calibrate
  i2cd(SI5351,3,0xff ^ (CLK0_EN + CLK1_EN + CLK2_EN));   //!!! testing only, all on
}

void qsy(){         // change frequency
unsigned char divf;
uint32_t f4;

   divf = 0;   // flag if we need to reset the PLL's
   if( freq > 15000000 && Rdiv >= 2 ) Rdiv = 1, divf = 1;
   if( freq <  3000000 && Rdiv == 1 ) Rdiv = 2, divf = 1;
   f4 = Rdiv * 4 * freq;
   f4 = f4 / 1000000;
   while( f4 * divider > 900 ) divider -= 2, ++divf;
   while( f4 * divider < 600 ) divider += 2, ++divf;
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

   if( Serial.availableForWrite() > 20 ) radio_control();
   

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
   if( P2 > c ) P2 = 0;        // ? avoid negative numbers 
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


/******   Elecraft Command Emulation  ******/
      //  K3 emulation code
// !!!!!!comment out errors for now - clean up when working            
void get_freq(uint32_t vfo ){   /* report vfo */

    //if( mode == CW && fun_selected[0] != WIDE ) vfo = vfo + (( sideband == USB ) ? mode_offset : - mode_offset);     
    stage_str("000");
    if( vfo < 10000000 ) stage('0');
    stage_long(vfo);  
}

void radio_control() {

static String command = "";
String lcommand;
char c;
int ritl;    // local copy of rit
int sm;
// int bat;  /* put battery voltage in front panel revision */

    if (Serial.available() == 0) return;
    
    while( Serial.available() ){
       c = Serial.read();
       if( c < ' ' ) continue;
       command += c;
       if( c == ';' ) break;
    }
  // !!! test for setup command here  
    if( c != ';' ) return;  /* command not complete yet */

    operate_mode = CAT_MODE;      // switch to CAT MODE when receive a ';'
    lcommand = command.substring(0,2);
 
    if( command.substring(2,3) == ";" || command.substring(2,4) == "$;" || command.substring(0,2) == "RV" ){      /* it is a get command */
      stage_str(lcommand);  /* echo the command */
      if( command.substring(2,3) == "$") stage('$');
      
      if (lcommand == "IF") {
/*
RSP format: IF[f]*****+yyyyrx*00tmvspbd1*; where the fields are defined as follows:
[f] Operating frequency, excluding any RIT/XIT offset (11 digits; see FA command format)
* represents a space (BLANK, or ASCII 0x20)
+ either "+" or "-" (sign of RIT/XIT offset)
yyyy RIT/XIT offset in Hz (range is -9999 to +9999 Hz when computer-controlled)
r 1 if RIT is on, 0 if off
x 1 if XIT is on, 0 if off
t 1 if the K3 is in transmit mode, 0 if receive
m operating mode (see MD command)
v receive-mode VFO selection, 0 for VFO A, 1 for VFO B
s 1 if scan is in progress, 0 otherwise
p 1 if the transceiver is in split mode, 0 otherwise
b Basic RSP format: always 0; K2 Extended RSP format (K22): 1 if present IF response
is due to a band change; 0 otherwise
d Basic RSP format: always 0; K3 Extended RSP format (K31): DATA sub-mode,
if applicable (0=DATA A, 1=AFSK A, 2= FSK D, 3=PSK D)
*/      
        get_freq(freq );
        stage_str("     ");
        ritl=  0;  // rit;
        if( ritl >= 0 ) stage_str("+0");
        else{
          stage_str("-0"); 
          ritl = - ritl;
        }
        if( ritl < 100 ) stage('0');
        if( ritl < 10 ) stage('0');    //IF[f]*****+yyyyrx*00tmvspbd1*;
        stage_num(ritl);
        stage_str("10 0003");    /* rit,xit,xmit,cw mode */
      //  if( split == 2 ) stage_str("10");
       /* else */ stage_str("00");
      //  if( split ) stage('1');
       /* else */ stage('0');
        stage_str("001 ");      
      }
      else if(lcommand == "FA") get_freq( freq );
      else if(lcommand == "FB") get_freq( freq );
      else if(lcommand == "RT") stage('0') ; //{ if(rit_state) stage('1'); else stage('0'); }
      else if(lcommand == "FR") stage('0');
      else if(lcommand == "FT"){
        // if( split ) stage('1');
        /* else */ stage('0');
      }
      else if( lcommand == "KS"){
        stage('0');
        stage_num(12);   //(wpm);
      }
      else if(lcommand == "XF") stage_num(2);     // bandwidth fun_selected[0]);
      else if(lcommand == "AG") stage_str("030");
      else if(lcommand == "RG") stage_str("250");
      else if(lcommand == "PC") stage_str("005");
      else if(lcommand == "FW") {stage_str("0000") ; stage_num(2);}    //stage_num(fun_selected[0]);
      else if(lcommand == "IS") stage_str("0000");
      else if(lcommand == "AN") stage('1');
      else if(lcommand == "GT") stage_str("004");
      else if(lcommand == "TQ") stage_num(0);   //stage_num(transmitting);
      else if(lcommand == "PA" || lcommand == "XT" || lcommand == "NB" ) stage('0');
      else if(lcommand == "RA") stage_str("00");
      else if(lcommand == "OM") stage_str("-----F------");
      else if(lcommand == "LK") stage_num( 0 );   //stage_num(user[LOCK]);
      else if(lcommand == "MD") stage_num(2);   // (3-mode );   // stage('3');
      else if(lcommand == "RV" && command.substring(2,3) == "F"){  /* battery voltage in the revision field */
        stage(command.charAt(2));
       // bat = battery(0);
       // stage_num(bat/10);
       // stage('.');
       // stage_num(bat % 10);
        stage('0');
      }
      else if(lcommand == "RV" && command.substring(2,3) == "A"){  /* swap status in revision field */
        stage(command.charAt(2));
       // if( split == 2 ) stage_str("SWAP ");
        /*else*/ stage_str("    ");
      }
      else if(lcommand == "RV"){   // revisions
        stage(command.charAt(2));
        stage_str("     ");
      }
      else if(lcommand == "SM"){
        stage_str("00");
        sm =  9;   //smeter(0);
        if( sm < 10 ) stage('0');
        stage_num(sm);
      }  
      else if(lcommand == "ID"){
        stage_str("017");
      }
      else if(lcommand == "BW"){
        stage_str("0300");
      }       
      else{
         stage('0');  /* don't know what it is */
      }
 
    stage(';');   /* response terminator */
    }
    
    else  set_k3(lcommand,command);    /* else it is a set command ? */
   
    command = "";   /* clear for next command */
}


void set_k3(String lcom, String com ){
String arg;
uint32_t val;
char buf[25];

 
    if( lcom == "FA" || lcom == "FB" ){    /* set vfo freq */
      arg = com.substring(2,13);
      arg.toCharArray(buf,25);
      val = atol(buf);
  //    if( mode == CW && fun_selected[0] != WIDE ) val = val - (( sideband == USB ) ? mode_offset : - mode_offset);     
   //     cat_band_change((unsigned long)val);
  //      if( lcom == "FB" && xit_state ) xit = freq - val;
        if( lcom == "FA" ){
            freq = val;
            qsy();
        }
   //     freq_display(freq);
    }
    else if( lcom == "KS" ){    /* keyer speed */
      arg= com.substring(2,5);
      arg.toCharArray(buf,25);
      val = atol(buf);
  //    wpm = val;
    }
    else if( lcom == "LK" ){     /* lock vfo's */
    //  val = com.charAt(2);
    //  if( val == '$' ) val = com.charAt(3);
    //  user[LOCK] = val - '0';
    }
    else if( lcom == "FW" ){     /* xtal filter select */
      val = com.charAt(6) - '0';
      if( val < 4 && val != 0 ){
     //   fun_selected[0] = val;
     //   set_band_width(val);
      }
    }
    else if( lcom == "FT" ){     /* enter split */
      val = com.charAt(2) - '0';
      if( val == 0 ){
     //   if( split == 2 ) rx_vfo = tx_vfo;
     //   else tx_vfo = rx_vfo;        
      }
    //  split = user[SPLIT] = val;
    //  user[SWAPVFO] = 0;        
    }
    else if( lcom == "FR" ){    /* cancel split ? */
      val = com.charAt(2);
      if( val == '0' ){
      //  if( split == 2 ) rx_vfo = tx_vfo;
       // else tx_vfo = rx_vfo;
       // split = user[SPLIT] = user[SWAPVFO] = 0;
      }
    }
    else if( com == "SWT11;" ){    /* A/B tap. swap (listen) vfo */
    //  if( split < 2 ){            /* turns on split if off */
    //    split = 2;
    //    user[SPLIT]= user[SWAPVFO] = 1;
    //  }
     // else{                        /* back to listen on RX freq, stay in split */
     //  split = 1;
     //  user[SWAPVFO] = 0;
     // }
     // update_frequency(DISPLAY_UPDATE);  /* listen on selected vfo */
    } 
                
}



/*******************   stage buffer to avoid serial.print blocking ****************/
//   modified to use serial function availableForWrite removing the need for a local
//   character buffer




void stage( unsigned char c ){
  Serial.write(c);
 // stg_buf[stg_in++] = c;
 // stg_in &= ( STQUESIZE - 1 );
}

void stage_str( String st ){
//int i;
//char c;

  Serial.print(st);
//  for( i = 0; i < st.length(); ++i ){
//     c= st.charAt( i );
//     stage(c);
//  }    
}

void stage_num( int val ){   /* send number in ascii */
//char buf[35];
//char c;
//int i;
   Serial.print(val);
 //  itoa( val, buf, 10 );
 //  i= 0;
 //  while( c = buf[i++] ) stage(c);  
}

void stage_long( long val ){
//char buf[35];
//char c;
//int i;

   Serial.print(val);
//   ltoa( val, buf, 10 );
//   i= 0;
//   while( c = buf[i++] ) stage(c);  
}



        /* end of K3 emulation functions */
#ifdef NOWAY

void tune_band_change( ){    // if tune past band region, switch to another filter and band registers
int b;                       // not saving the band info, so if band switch will go back to last save and not the band edge
    
    b = check_band(freq);
    if( b != band ){    // band change
       band = b;
       band_change2();  
    }
}


void cat_band_change( uint32_t val ){    /* detect if we have a large qsy */
int b;
    
    b = check_band(val);
    if( b != band ){    // band change
       band = b;
       freq = val;
       rit = band_info[band].rit;
       xit = band_info[band].xit;
       rit_state = band_info[band].rit_state;
       xit_state = band_info[band].xit_state;
     //  stp = band_info[band].stp;
       mode = band_info[band].mode;
      // tone_offset = band_info[band].tone_offset;
       band_change2();  
    }
}

int check_band( uint32_t val ){
int b;
 
    b= 0;
    val /= 100;
    if( val < 5000000 ) b = 0; 
    if( val >= 5000000 && val < 6000000) b= 1; 
    if( val >= 6000000 && val < 8000000) b= 2;
    if( val >= 8000000 && val < 12000000 ) b= 3;
    if( val >= 12000000 && val < 16000000 ) b= 4;
    if( val >= 16000000  ) b= 5;
    return b;
}

int un_stage(){    /* send a char on serial */
char c;

   if( stg_in == stg_out ) return 0;
   c = stg_buf[stg_out++];
   stg_out &= ( STQUESIZE - 1);
   Serial.write(c);
   return 1;
}
#endif

