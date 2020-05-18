/*********************************************************
  Test lecture AD7656 par Arduino DUE en mode parallèle 03/2019 par Olivier pour P. Roncin
  il y deux versions, le cablage direct par fil : #define Olivier_WIRING
  celle avec le petit PCB #define Lionel_PCB

  In FAKE_DATA mode, we generate 4 numbers from channel 0 to channel 4, result[0]=4930,result[1]=7820,result[2]=1050,result[3]=7811.
  Channel 5 counts the event number as result[4]=nombre_appels. Channel 6 to generate a number as result[5]=0xf0f0 to separate the data;
*********************************************************/
// First you should check that you have the correct PCB (see Harware.h file) Stack_PCB, Inline_PCB, Lionel_PCB, Olivier_Wiring
#define Olivier_Wiring           
#include <DueTimer.h>

#define LEDKEY
#define FAKE_DATA
#include "Hardware.h"

//#define FILL_BUFFER           // When defined the Arduino DUE is generating the BUSY signal instead of the YX card. pin 10 toggles in the loop, pin 11 toggle at Send Buffer,
                            // The data are read but not put into Buffer so it should not generate a real transmission, enter Send Buffer and exit immediatly after togling pin 11

#define SOFTWARE_TRIGGER    // IF selected the loop will generate n_event_per_ms event per ms (in fact twice because fake--; is inside the send_Buffer prog
const int Software_trigger_pin=6; // the fake signal is emitted on pin 8/6 and should be hard wired to pin BUSY1 (28/4)
const int n_event_per_ms=1;  // the total amount of fake data generated each second is n_event_per_ms*2000*12 bytes and 8 times more in bits 
uint32_t t_ms=0,fake=0;              // example n_event_per_ms=1 is 2000 event per second and 192 kbits with n_event_per_ms=100 we reach 2.4 MBytes/s and 200 000 events per second.

//#define NATIVE_TRANSMIT    // When defined transmission is via Nativr USB port otherwise data are sent vie the programming port
const int Channel_per_event = 6; //We read all six ADC channels
uint32_t Reg_A,Reg_B,Reg_C,Reg_D,Reg_E; // we only use C 
#define Baud_rate_prog 115200  // max tested was 250000

const int  probe10 = 10;  //utilisées pour sonder à l'oscillo
const int  probe11 = 11;
const int  probe12 = 12;
const int  probe13 = 13;  //is also the LED  pin
  

#define Buff_char_size 16384           // 14 bit en char soit 8192 mots de 16 bits (soit sorties de l'ADC 16 bits) soit 1365.33 evenements de six valeurs ou 2048 ev de 4 valeurs
uint8_t Buffer[2][Buff_char_size];     // array to hold the data before sending it by packet through USB to the PC

uint16_t* v1= (uint16_t*) (&(Buffer[0][0]));
uint16_t* vmax1 = v1 + (Buff_char_size/2);    // on doit diviser par deux car l'addition de pointeur tient compte du type, ici uint16_t prends deux octets
uint16_t* v2= (uint16_t*) (&(Buffer[1][0]));
uint16_t* vmax2 = v2 + (Buff_char_size/2);   // on doit diviser par deux car l'addition de pointeur tient compte du type, ici uint16_t prends deux octets

uint16_t* v=v1    ;
uint16_t* vmax=vmax1 ;  


uint16_t* v12[2]= {v1,v2};          // un tableau de pointeur pour swicher de buffer lorsque l'un est plein en attendant la transmission
uint16_t* vmax12[2]= {vmax1,vmax2}; // un tableau de pointeur pour swicher de buffer lorsque l'un est plein en attendant la transmission

int num_actuel = 0;
int num_autre  = 1;

bool BUFFERS_FULL=false;
//long int testmille = 0;
bool Bufferfull[2] = {false,false};             // are the buffers full ?
int byteswritten = 0;               // count the number of bytes written for display
uint32_t exclusions = 0;                 // Counts the number of unaccepted events
uint32_t local_buff[6];
int16_t  local_int[6];
int16_t  running_sum[6];
int16_t  result[6];
int button_selected=0;


//USB
int x, b, i, nombre_appels;
//uint16_t buf[4][256];

//===================================setup================================
void setup()
{// C1 TO C9 ARE PLUGGED TO D0-D8 (9 Bits) AND /C12 TO C18 ARE PLUGGED TO D9-D15 . (7 Bits) Note that D is for data from ADC
  //REG_PIOC_PER |=(mask_C11_17|mask_C0_8)<<1; //- write 1's here to override other peripherals and allow GPIO use for pins
  //REG_PIOC_ODR |=(mask_C11_17|mask_C0_8)<<1; //- write 1's here to set pins as INPUT


  pinMode(DB0, INPUT_PULLUP);
  pinMode(DB1, INPUT_PULLUP);
  pinMode(DB2, INPUT_PULLUP);
  pinMode(DB3, INPUT_PULLUP);
  pinMode(DB4, INPUT_PULLUP);
  pinMode(DB5, INPUT_PULLUP);
  pinMode(DB6, INPUT_PULLUP);
  pinMode(DB7, INPUT_PULLUP);
  pinMode(DB8, INPUT_PULLUP);
  pinMode(DB9, INPUT_PULLUP);
  pinMode(DB10, INPUT_PULLUP);
  pinMode(DB11, INPUT_PULLUP);
  pinMode(DB12, INPUT_PULLUP);
  pinMode(DB13, INPUT_PULLUP);
  pinMode(DB14, INPUT_PULLUP);
  pinMode(DB15, INPUT_PULLUP);


  //pinMode(LED_BUILTIN, OUTPUT);
  //pinMode(CS_SCREEN, OUTPUT);
  //pinMode(RST_SCREEN, OUTPUT);
  pinMode(CS_ADC1, OUTPUT);pinMode(CS_ADC2, OUTPUT);pinMode(CS_ADC3, OUTPUT);pinMode(CS_ADC4, OUTPUT);
  pinMode(BUSY1, INPUT_PULLUP);pinMode(BUSY2, INPUT_PULLUP);pinMode(BUSY3, INPUT_PULLUP);pinMode(BUSY4, INPUT_PULLUP);
  
  #ifdef SOFTWARE_TRIGGER
  pinMode(CONVERT1, OUTPUT);    //when triggered internaly CONST should be declared OUTPUT, when used externally then INPUT
  #endif
  #ifndef SOFTWARE_TRIGGER 
  pinMode(CONVERT1, INPUT);pinMode(CONVERT2, INPUT);pinMode(CONVERT3, INPUT);pinMode(CONVERT4, INPUT);  
  #endif
  
  pinMode(RESET, OUTPUT);
  pinMode(Read_ADC, OUTPUT);
  pinMode(probe10, OUTPUT);
  pinMode(probe11, OUTPUT);
  pinMode(probe12, OUTPUT);
  pinMode(probe13, OUTPUT);
  pinMode(Software_trigger_pin, OUTPUT);


  delayMicroseconds(100); // just to make sure the chip powers up !! carefull delay uses timers
  digitalWrite(Software_trigger_pin, LOW);
  //digitalWrite(probe10, LOW);
  //digitalWrite(probe11, LOW);
  //digitalWrite(probe12, LOW);
  //digitalWrite(probe13, LOW);
  #ifdef LEDKEY
   tm.displayBegin();
  #endif

  attachInterrupt(digitalPinToInterrupt(BUSY1), fastread, FALLING); // go to BUSY_ISR() when falling edge on BUSY
  Timer3.attachInterrupt(SendBuffer).setFrequency(2000).start();   //Check to send the buffers if full at a given frequency
  NVIC_SetPriority (TC3_IRQn, 3);                                  // Sets the Buffer Sending to a lower priority than the Buffer Reading
Serial.begin(115200);  // like all arduino, can be used to send and receive datas at moderate speed

//  ////////////////////////////////fast USB on the native USB port Only on arduino DUE used here to transmit buffers /////////////////////////////
digitalWrite(probe13, HIGH);
Light_LEDKEY(0b10101010);
#ifdef NATIVE_TRANSMIT
    SerialUSB.begin(2000000);
    while (!SerialUSB);
 #else
    Serial.begin(Baud_rate_prog);
    while (!Serial);
 #endif

Reset_ADC();
digitalWrite(probe13, LOW);     // the LED writen L is ON as long as native USB is not active
}

//===================================loop (could be left empty) ================================
void loop()
{ int16_t val;
  
  //if (!(t_ms & 0x3ff)) {Serial.print("b");}
 if (!(t_ms & 0x3ff)) {val=result[button_selected];
                       float V=Volts*val;Serial.print(button_selected);Serial.print(":");Serial.print(val, HEX);Serial.print(":");Serial.print(val, BIN);Serial.print(" ");
                       Serial.print(val);Serial.print(" ");Serial.print(V,3);Serial.println(" Volts ");}
  #ifdef LEDKEY
  if (!(t_ms & 0x0f)) {Display_int_LEDKEY(result[button_selected]);} //every 16ms {Display_int_LEDKEY(t_ms);}
  //if (!(t_ms & 0x03ff)) 
       {uint8_t but=Read_LEDKEY_buttons();
       if (but!=0) {
        if (but<0b1000000) Light_LEDKEY(but);
        if (but==0b1) button_selected=0;
        if (but==0b10) button_selected=1;
        if (but==0b100) button_selected=2;
        if (but==0b1000) button_selected=3;
        if (but==0b10000) button_selected=4;
        if (but==0b100000) button_selected=5;
        //if (but==0b1000000) button_selected=6;
        if (but==0b10000000) Reset_ADC();       
        }
        //else Light_LEDKEY(0b01);
       }// every half second {Display_int_LEDKEY(t_ms);}
  #endif
  #ifdef SOFTWARE_TRIGGER
     if(fake>0) {convertADC();fake--;}
  #endif
  #ifndef FILL_BUFFER
 
    //digitalWrite(probe10, LOW);  // pin 10 goes up and down all the time
    //digitalWrite(probe10, HIGH); // to check that program is running (only When FILL_BUFFER is not Active)
  #endif 
}
//===========================function definitions========================

void SendBuffer()     // envoie des buffers sur la liaison USB native
{
  #ifdef NATIVE_TRANSMIT
   if (SerialUSB)
 #else
   if (Serial)
 #endif
  {if (Bufferfull[num_autre]) {
    #ifdef NATIVE_TRANSMIT
    byteswritten +=SerialUSB.write(Buffer[num_autre], Buff_char_size);// the units are in number of byte (2 per data)
    #else
    byteswritten +=Serial.write(Buffer[num_autre], Buff_char_size);// the units are in number of byte (2 per data)
    #endif
   Bufferfull[num_autre] =false;
    BUFFERS_FULL=false;
    memset(Buffer[num_autre], 0, Buff_char_size);                     // fast erase (cf Web) usefell only if we decide to send unfilled buffers
  }
  if (Bufferfull[num_actuel]) {
    #ifdef NATIVE_TRANSMIT
    byteswritten +=SerialUSB.write(Buffer[num_actuel], Buff_char_size); // la procédure fonctionne par octet
    #else
    byteswritten +=Serial.write(Buffer[num_actuel], Buff_char_size); // la procédure fonctionne par octet
    #endif
    Bufferfull[num_actuel] =false;
    BUFFERS_FULL=false;
    memset(Buffer[num_actuel], 0, Buff_char_size);                      // remise à zéro rapide (cf Web) pour éviter les erreurs lors de transmission de buffers incomplets
  }
 }
 t_ms++;
#ifdef SOFTWARE_TRIGGER 
   fake=n_event_per_ms;
#endif
#ifndef FILL_BUFFER
    //digitalWrite(probe11, LOW); // pin 11 goes up and down each time we enter send buffer
    //digitalWrite(probe11, HIGH);// 
#endif 
}
//===================initiate convertion for interrupt testing====================================
void convertADC()    // to simulate events, pin 8(lionel)/6(olivier) has to be inserted in BUSY pin 28/4 
  {//digitalWrite(Software_trigger_pin, HIGH); //pulse pin 8(lionel)/6(olivier) to simulate CONVERT
   //digitalWrite(Software_trigger_pin, LOW);
   //digitalWrite(Software_trigger_pin, HIGH);
   digitalWrite(CONVERT1, HIGH); //pulse pin 8(lionel)/6(olivier) to simulate CONVERT
   digitalWrite(CONVERT1, LOW);
   digitalWrite(CONVERT1, HIGH); }

//=======================================================================
// Reset the ADC
void Reset_ADC()    //reset All ADC boards
  {digitalWrite(RESET, LOW); //start low  
   digitalWrite(CS_ADC1, HIGH);digitalWrite(CS_ADC2, HIGH);digitalWrite(CS_ADC3, HIGH);digitalWrite(CS_ADC4, HIGH); 
   digitalWrite(CONVERT1, HIGH);digitalWrite(CONVERT2, HIGH);digitalWrite(CONVERT3, HIGH);digitalWrite(CONVERT4, HIGH); 
   digitalWrite(Read_ADC, HIGH);
   //idle high, pulse low and back to high to initiate conversion, should be high during the reset pulse
  // digitalWrite(RESET, HIGH); //reset pulse
  // digitalWrite(RESET, LOW);
  Reset_ADC_high;
  Reset_ADC_low;
   digitalWrite(CS_ADC1, LOW);digitalWrite(CS_ADC1, HIGH);
}

//=====================================================================
void fastread()     // normalement on arrive içi tant que le convertisseur émet des BUSY acr c'est l'interruprion qui branche
{ uint32_t dummy,*ptr=local_buff;uint16_t dummy16; int16_t value,*p_int16;
  nombre_appels++;  // compteur global du nombre d'appels
  if(v+6>=vmax) {                     // Attention à vérifier les bornes et égalités
    Bufferfull[num_actuel]=true;
    if(!Bufferfull[num_autre])
          {v=v12[num_autre];
           vmax=vmax12[num_autre];
           int a = num_actuel;      //swap numbers
           num_actuel = num_autre;
           num_autre =  a;
          }
      else {                      // plus de place, il faut attendre
           exclusions++;               // BUFFERS_FULL=true;
           BUFFERS_FULL=true;
           }
  }     // Arrivé içi on est sur d'avoir de la place, reste à lire les data
         // Reading Procedure, une boucle de 6 lectures du port C de 32 bit tronquée aux 16 bit significatifs de l'ADC
 // if(!BUFFERS_FULL)      // enter only if we have room to place data, our 16 bit data are all in port C which has 32 bits
      {CS_ADC1_low;       // first thing to do is put CS low (High when we leave)
       //Read_ADC_low;     // data are given by ADC, one after each other when Read_ADC pin is going low
       for (uint32_t i = 0; i < 6; i++)  // We could read less channels
        {Read_ADC_low;     // data are given by ADC, one after each other when Read_ADC pin is going low
          *(ptr++)=Read_C;          // We read the 32 bits of port_C among which are our 16, after the shift right C1..C9 (D0) becomes C0..C8 and C12..18 becomes C11..17           
         Read_ADC_high;              // Read_ADC pin is going high
         //Read_ADC_low;               // Read_ADC pin is going low (next data will be ready soon but meanwhile we store the data read above
        }
        CS_ADC1_high;
        //Read_ADC_high; 
         
                
       for (uint32_t i = 0; i < 6; i++){  // now we fill the buffer;
          dummy=local_buff[i]>>1;
          dummy=(((dummy  & mask_C11_17)>>2) | (dummy & mask_C0_8));
          dummy16=uint16_t (dummy);
          value  =int16_t(dummy16);
          result[i]=value;
          }

 #ifdef FAKE_DATA
   result[0]=4930;result[1]=7820;result[2]=1050;result[3]=7811;result[4]=nombre_appels;result[5]=0xf0f0;
 #endif
 #ifdef FILL_BUFFER
        p_int16=result;
        for (uint32_t i = 0; i < 6; i++){  // now we fill the buffer;
          *(v++)=*p_int16++;}
              //*(v++)=(uint16_t) ((dummy  & mask_C11_17)>>2) | (dummy & mask_C0_8);  // C0..8 are in place and C11..17 need to be shifted two times to form our 16 bits data
 #endif
        
 
      }
}
void Display_int_LEDKEY(uint32_t n){
     tm.displayIntNum(n, true);
    }
void Display_hex_LEDKEY(uint32_t n){
      tm.displayHex(7, n&0xf);
      tm.displayHex(6, (n>>4)&0xf);
      tm.displayHex(5, (n>>8)&0xf);
      tm.displayHex(4, (n>>12)&0xf);
    }
void Light_LEDKEY(uint8_t value) {// each bit at 1 will light the associated LED 0001 is the leftmost
  for (uint8_t position = 0; position < 8; position++) {
    tm.setLED(position, value & 1);
    value = value >> 1;
  }
}

uint8_t Read_LEDKEY_buttons(){uint8_t b=tm.readButtons();
     return b;
    }


/*
       {CS_ADC1_low;       // first thing to do is put CS low (High when we leave)
       Read_ADC_low;     // data are given by ADC, one after each other when Read_ADC pin is going low
       for (int i = 0; i < 6; i++)  // We could read less channels
        {Reg_C =Read_C>>1;          // We read the 32 bits of port_C among which are our 16, after the shift right C1..C9 (D0) becomes C0..C8 and C12..18 becomes C11..17           
         Read_ADC_high;              // Read_ADC pin is going high
         Read_ADC_low;               // Read_ADC pin is going low (next data will be ready soon but meanwhile we store the data read above
         #ifndef TEST_MODE
              *(v++)=(uint16_t) ((Reg_C  & mask_C11_17)>>2) | (Reg_C & mask_C0_8);  // C0..8 are in place and C11..17 need to be shifted two times to form our 16 bits data
         #endif
        }
        CS_ADC1_high;
      }
*/
 
