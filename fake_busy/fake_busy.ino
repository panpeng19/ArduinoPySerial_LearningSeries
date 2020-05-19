/*********************************************************
  Test AD7656 en mode parallèle 03/2019 pour P. Roncin
  Arduino DUE
*********************************************************/
// longint est un entier signé sur 32 bit c'est donc un int32_t
// pour écrire en 32 bit sur le port B   :     REG_PIOB_ODSR = 0x45a12bb2;    
// pour lire le port B en 32 bit               uint_32   bid =REG_PIOB_PDSR;
// La lettre B est changée en A, B, C... pour les autres ports


#define Bsy_A 3

//#include <SPI.h>
#include <DueTimer.h>

const int  probe10 = 10;
const int  probe11 = 11;
const int  probe12 = 12;
const int  probe13 = 13;//LED ?
const int  fake_busy=21;

uint32_t Reg_A,Reg_B,Reg_C,Reg_D;
// definition and action on CS_ADC pin 31 port A7
const int CS_ADC = 31;    // PORT A7 chip select
const uint32_t Mask_HighCS = 1<<7;
const uint32_t Mask_LowCS = ~Mask_HighCS;
#define CS_ADC_low  REG_PIOA_ODSR= REG_PIOA_PDSR & Mask_LowCS
#define CS_ADC_high REG_PIOA_ODSR= REG_PIOA_PDSR | Mask_HighCS


// definition and action on Read_ADC pin 32 port D10
const int Read_ADC = 32;    // PORT D10
const long Mask_High_Read = 1<<10;
const long Mask_Low_Read  =  ~Mask_High_Read;
#define Read_ADC_low  REG_PIOD_ODSR &= Mask_Low_Read
#define Read_ADC_high REG_PIOD_ODSR |= Mask_High_Read

// definition and action on ADC_Convert pin 25 port D0
const int  CONVERT = 25;    // PORT D0
const long Mask_High_Conv = 1;
const long Mask_Low_Conv  =  ~Mask_High_Conv;
#define Convert_ADC_low  REG_PIOD_ODSR &= Mask_Low_Conv
#define Convert_ADC_high REG_PIOD_ODSR |= Mask_High_Conv

// definition and action on ADC_Reset pin 24 port A15
const int  RESET = 24;    // PORT A15
const long Mask_HighReset = 1<<7;
const long Mask_LowReset = ~Mask_HighReset;
#define Reset_ADC_low  REG_PIOA_ODSR &= Mask_LowReset
#define Reset_ADC_high REG_PIOA_ODSR |= Mask_HighReset

  const long mask_C12_18 = 0x0003F800;
  const long mask_C1_9   = 0x000001FF;


const int BUSY = 28;  // PORT D3 PIN 28
// C1 TO C9 ARE PLUGGED TO D0-D8 (9 BITS) AND /C12 TO C18 ARE PLUGGED TO D9-D15 . (7 BITS) Note that D is for data
const int DB0 = 33;//C1
const int DB1 = 34;//C2
const int DB2 = 35;//C3
const int DB3 = 36;//C4
const int DB4 = 37;//C5
const int DB5 = 38;//C6
const int DB6 = 39;//C7
const int DB7 = 40;//C8
const int DB8 = 41;//C9
const int DB9 = 51;//C12
const int DB10 = 50;//C13
const int DB11 = 49;//C14
const int DB12 = 48;//C15
const int DB13 = 47;//C16
const int DB14 = 46;//C17
const int DB15 = 45; //C18 sign
#define Buff_char_size 16384   // 14 bit en char soit 8192 mots de 16 bits (soit sorties de l'ADC 16 bits) soit 1365.33 evenements de six valeurs ou 2048 ev de 4 valeurs

//long int Vout[6] = {0, 0, 0, 0, 0, 0}; //array to hold six voltages (bits for now)
uint8_t Buffer[2][Buff_char_size];     // array to hold the data before sending it by packet through USB to the PC

uint16_t* v1= (uint16_t*) (&(Buffer[0][0]));
uint16_t* vmax1 = v1 + (Buff_char_size/2);  // on doit diviser par deux car l'addition de pointeur tient compte du type, ici uint16_t prends deux octets

uint16_t* v2= (uint16_t*) (&(Buffer[1][0]));
uint16_t* vmax2 = v2 + (Buff_char_size/2); // on doit diviser par deux car l'addition de pointeur tient compte du type, ici uint16_t prends deux octets

uint16_t* v=v1    ;
uint16_t* vmax=vmax1 ;   


uint16_t* v12[2]= {v1,v2};          // un tableau de pointeur pour swicher de buffer lorsque l'un est plein en attendant la transmission
uint16_t* vmax12[2]= {vmax1,vmax2}; // un tableau de pointeur pour swicher de buffer lorsque l'un est plein en attendant la transmission

int num_actuel = 0;
int num_autre = 1;

bool BUFFERS_FULL=false;
//long int testmille = 0;
bool Bufferfull[2] = {false,false};             // are the buffers full ?
int byteswritten = 0;               // count the number of bytes written for display
uint32_t exclusions = 0;                 // Counts the number of unaccepted events




#define SettoLow_(i) &(~(1<<i))  // utilisé pour forcer un bit à zéro le bit numéro i quelque soit la taille du mot
#define SettoHigh_(i) |(1<<i)    // utilisé pour forcer un bit à un le bit numéro i quelque soit la taille du mot

long t0 = 0;
long t = 0;
uint32_t fake=0;

//USB
int x, b, i, nombre_appels;
//uint16_t buf[4][256];

//===================================setup================================
void setup()
{
  //pinMode(LED_BUILTIN, OUTPUT);
  //pinMode(CS_SCREEN, OUTPUT);
  //pinMode(RST_SCREEN, OUTPUT);
  pinMode(fake_busy, OUTPUT);
  pinMode(CS_ADC, OUTPUT);
  pinMode(CONVERT, INPUT);  //pinMode(CONVST, OUTPUT); when triggered internaly CONST should be declared OUTPUT, when used externally then INPUT
  pinMode(RESET, OUTPUT);
  pinMode(Read_ADC, OUTPUT);
  pinMode(probe10, OUTPUT);
  pinMode(probe11, OUTPUT);
  pinMode(probe12, OUTPUT);
  pinMode(probe13, OUTPUT);
  pinMode(BUSY, INPUT_PULLUP);
  pinMode(DB0, INPUT);//INPUT_PULLUP);
  pinMode(DB1, INPUT);//INPUT_PULLUP);
  pinMode(DB2, INPUT);//INPUT_PULLUP);
  pinMode(DB3, INPUT);//INPUT_PULLUP);
  pinMode(DB4, INPUT);//INPUT_PULLUP);
  pinMode(DB5, INPUT);//INPUT_PULLUP);
  pinMode(DB6, INPUT);//INPUT_PULLUP);
  pinMode(DB7, INPUT);//INPUT_PULLUP);
  pinMode(DB8, INPUT);//INPUT_PULLUP);
  pinMode(DB9, INPUT);//INPUT_PULLUP);
  pinMode(DB10, INPUT);//INPUT_PULLUP);
  pinMode(DB11, INPUT);//INPUT_PULLUP);
  pinMode(DB12, INPUT);//INPUT_PULLUP);
  pinMode(DB13, INPUT);//INPUT_PULLUP);
  pinMode(DB14, INPUT);//INPUT_PULLUP);
  pinMode(DB15, INPUT);//INPUT_PULLUP);
  delayMicroseconds(100); // just to make sure the chip powers up
  digitalWrite(CS_ADC, HIGH); //idle high, take low for transfers
  digitalWrite(Read_ADC, HIGH);
  digitalWrite(CONVERT, HIGH); //idle high, pulse low and back to high to initiate conversion, should be high during the reset pulse
  digitalWrite(RESET, LOW); //start low
  delayMicroseconds(1);
  digitalWrite(RESET, HIGH); //reset pulse
  delayMicroseconds(1);
  digitalWrite(RESET, LOW);
  digitalWrite(probe10, LOW);
  digitalWrite(probe10, HIGH);
  digitalWrite(probe10, LOW);
  digitalWrite(probe11, LOW);
  digitalWrite(probe12, LOW);
  digitalWrite(probe13, LOW);
  digitalWrite(fake_busy, LOW);
  digitalWrite(fake_busy, HIGH);
  digitalWrite(fake_busy, LOW);
  digitalWrite(fake_busy, HIGH);
  attachInterrupt(digitalPinToInterrupt(BUSY), fastread, FALLING); // go to BUSY_ISR() when falling edge on BUSY
  Timer3.attachInterrupt(SendBuffer).setFrequency(2000).start();   //Check to send the buffers if full at a given frequency
  NVIC_SetPriority (TC3_IRQn, 3);                                  // Sets the Buffer Sending to a lower priority than the Buffer Reading
  Serial.begin(9600);
//
//  //////////////////////////////////////////////////////////USB///////////////////////////////////////////////
SerialUSB.begin(9600);
  while (!SerialUSB);
  
delay(1000);
}

//===================================loop================================
void loop()
{ //convertADC();
if(fake)
  {digitalWrite(fake_busy, HIGH);
  digitalWrite(fake_busy, LOW);
  fake=0;
  }

}
//===========================function definitions========================


void SendBuffer()     // envoie des buffers sur la liaison USB native
{fake++;
if (SerialUSB)
  {if (Bufferfull[num_autre]) {
 //   digitalWrite(probe11, LOW);
    byteswritten +=SerialUSB.write(Buffer[num_autre], Buff_char_size);// la procédure fonction par octet
//  digitalWrite(probe11, HIGH);
    Bufferfull[num_autre] =false;
    BUFFERS_FULL=false;
    
    memset(Buffer[num_autre], 0, Buff_char_size);                     // remise à zéro rapide (cf Web) pour éviter les erreurs lors de transmission de buffers incomplets
  }
  if (Bufferfull[num_actuel]) {
    byteswritten +=SerialUSB.write(Buffer[num_actuel], Buff_char_size); // la procédure fonction par octet
    
    Bufferfull[num_actuel] =false;
    BUFFERS_FULL=false;
    
    memset(Buffer[num_actuel], 0, Buff_char_size);                      // remise à zéro rapide (cf Web) pour éviter les erreurs lors de transmission de buffers incomplets
  }
 }
}
//=======================================================================
// start convertion for interrupt testing
void convertADC()
{
  digitalWrite(CONVERT, LOW); //pulse CONVST
  //delayMicroseconds(1);
  digitalWrite(CONVERT, HIGH);
}

//=====================================================================
void fastread()     // normalement si on arrive içi c'est que l'on est pas encore en BUFFERS_FULL; le premier buffer à envoyer est num_autre
{ nombre_appels++; // compteur global du nombre d'appels
  if(v+6>=vmax) {                     // Attention à vérifier les bornes et égalités
    Bufferfull[num_actuel]=true;
    digitalWrite(probe10, HIGH);
    digitalWrite(probe11, Bufferfull[0]);
    digitalWrite(probe12, Bufferfull[1]);
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
 // BUFFERS_FULL=true;
  }      // Arrivé içi on est sur d'avoir de la place, reste à lire les data
         // Reading Procedure, une boucle de 6 lectures du port C de 32 bit tronquée aux 16 bit significatifs de l'ADC
  if(!BUFFERS_FULL) 
      {                          // Si il y a de la place  dans le buffer 
       unsigned long brut_C; 
       #define Register_C_input REG_PIOC_PDSR    // our 16 bit data are all in port C which has 32 bits
       CS_ADC_low;  
       Read_ADC_low;
       for (int i = 0; i < 6; i++)                                                         //
        {brut_C =Register_C_input>>1;                                    // On lit les 32 bits du port_C dont 16 contiennent les datas d茅cal茅 une fois a droite le bit 1 devient 0            
         *(v++)=(uint16_t) ((brut_C  & mask_C12_18)>>2) | (brut_C & mask_C1_9);            // On met en forme en isolant 7 bits haut et supprimant 9 et 10 et recollant avec les 9 bits du bas
         //*(v)=(uint16_t) ((brut_C  & mask_C12_18)>>2) | (brut_C & mask_C1_9);            // On met en forme en isolant 7 bits haut et supprimant 9 et 10 et recollant avec les 9 bits du bas
        Read_ADC_high;                        // Niveau haut sur Read 
        Read_ADC_low;                         // Niveau bas sur Read 
        } 
        //v++;
        CS_ADC_high; 
      }
  
   // else could be the place for a reset ?
  }
 
