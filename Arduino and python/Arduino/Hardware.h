/*  We put here all the hardware related numbers, which pin does what so that the main program is hardware-independent
the only important thing is that the correct PCB of wiring is declared among
Olivier_WIRING, Lionel_PCB, Inline_PCB, Stack_PCB

PIO_PER  - write 1's here to override other peripherals and allow GPIO use for pins
PIO_OER  - write 1's here to set pins as OUTPUT
PIO_ODR  - write 1's here to set pins as INPUT
* PIO_SODR   - write 1's here to set output pins HIGH
* PIO_CODR   - write 1's here to set output pins LOW
* PIO_PDSR  - read's actual state of the pins on the port.
PIO_PUDR  - write 1's here to switch off internal pull-up for pins
PIO_PUER  - write 1's here to switch on internal pull-up for pins

AD7656 has three control lines.
CONV when going from low to high the signal is Hold and conversion starts (better to have a signal high to low and then low to high
BUSYis an output of the AD7656 which goes high during convertion and goes low at the end, can be used to alert the ARM to come and read
CS chip select when low the reading can proceed
Read if CS is low, then 10 ns after a high to low transition of read then data are prensented to the BUS (D0 TO D15)
RESET IF CS is high at low to high then high to low pulse on Reset resets the ADC.
YX card has a RANGE Jumper> if jumper connect the center to D3V3 pin then we have +-5V, otherwise the range is +-10V
*/

#define RANGE_JUMPER_3V3
#ifdef RANGE_JUMPER_3V3
const float Volts = 5.0/32768;
#else
const float Volts = 10.0/32768;
#endif


// only one of the following cards should be selected Olivier_WIRING, Lionel_PCB
//#define Inlinel_PCB
//#define Stack_PCB
#define Olivier_WIRING    
//#define Lionel_PCB

#include <Arduino.h>

#define Read_A  REG_PIOA_PDSR
#define Read_B  REG_PIOB_PDSR
#define Read_C  REG_PIOC_PDSR
#define Read_D  REG_PIOD_PDSR
#define Write_A REG_PIOA_ODSR
#define Write_B REG_PIOB_ODSR
#define Write_C REG_PIOC_ODSR
#define Write_D REG_PIOD_ODSR
const  uint32_t Un=1;

  const long mask_C11_17 = 0x0003F800;
  const long mask_C0_8   = 0x000001FF;
/***************************************************************************************Inlinel_PCB*****************************************************************************/
#ifdef Inlinel_PCB  //BUSY:28,29,30,31, CS: 24,25,26,27 Convert 42,43,44,32 Reset 52 Read 23
    const int BUSY_1 = 30;
    const int BUSY_2 = 31;
    const int BUSY_3 = 28;
    const int BUSY_4 = 29;  
    
    const int CS_ADC1 = 36;               // PORT D1 chip select 1
#define CS_ADC1_high Write_D |= Un<<1       // SHOULD BE Write_A = Read_A | Un<<7  But this version is simpler and it works
#define CS_ADC1_low  Write_D &= ~(Un<<1)

    const int CS_ADC2 = 37;               // PORT D2 chip select 2
#define CS_ADC2_high Write_D |= Un<<2       
#define CS_ADC2_low  Write_D &= ~(Un<<2)

    const int CS_ADC3 = 24;               // PORT A15 chip select 3
#define CS_ADC3_high Write_A |= Un<<15      
#define CS_ADC3_low  Write_A &= ~(Un<<15)

    const int CS_ADC4 = 25;               // PORT D0 chip select 3
#define CS_ADC4_high Write_D |= Un       
#define CS_ADC4_low  Write_D &= ~(Un)

  const int Read_ADC = 23;    // PORT A14
#define Read_ADC_high Write_A |= Un<<14
#define Read_ADC_low  Write_A &= ~(Un<<14)

  const int  CONVERT1 = 44;    // PORT C19
#define Convert_ADC1_high Write_C |= Un<<19
#define Convert_ADC1_low  Write_C &= ~(Un<<19)

  const int  CONVERT2 = 43;    // PORT A20
#define Convert_ADC2_high Write_A |= Un<<20
#define Convert_ADC2_low  Write_A &= ~(Un<<20)

  const int  CONVERT3 = 42;    // PORT A19
#define Convert_ADC3_high Write_A |= Un<<19
#define Convert_ADC3_low  Write_A &= ~(Un<<19)

  const int  CONVERT4 = 32;    // PORT D10
#define Convert_ADC4_high Write_D |= Un<<10
#define Convert_ADC4_low  Write_D &= ~(Un<<10)

  const int  RESET = 52    // PORT B21
  #define Reset_ADC_high Write_B |= Un<<21  
  #define Reset_ADC_low  Write_B &= ~(Un<<21)
 
#endif
/***************************************************************************************Stack_PCB*****************************************************************************/
#ifdef Stack_PCB  //BUSY:26,27,28,29  CS:30,31,32,53 Convert 22,23,24,25 Reset 42 Read 43 
    const int BUSY_1 = 28;
    const int BUSY_2 = 29;
    const int BUSY_3 = 26;
    const int BUSY_4 = 27;  
    
    const int CS_ADC1 = 53;               // PORT B14 chip select 1
#define CS_ADC1_high Write_B |= Un<<14       // SHOULD BE Write_A = Read_A | Un<<7  But this version is simpler and it works
#define CS_ADC1_low  Write_B &= ~(Un<<14)

    const int CS_ADC2 = 32;               // PORT D10
#define CS_ADC2_high Write_D |= Un<<10       
#define CS_ADC2_low  Write_D &= ~(Un<<10)

    const int CS_ADC3 = 30;               // PORT D9
#define CS_ADC3_high Write_D |= Un<<9      
#define CS_ADC3_low  Write_D &= ~(Un<<9)

    const int CS_ADC4 = 31;               // PORT A7
#define CS_ADC4_high Write_A |= Un<<7        
#define CS_ADC4_low  Write_A &= ~(Un<<7)

  const int Read_ADC = 43;    // PORT A20
#define Read_ADC_high Write_A |= Un<<20
#define Read_ADC_low  Write_A &= ~(Un<<20)

  const int  CONVERT1 = 24;    // PORT A15
#define Convert_ADC1_high Write_A |= Un<<15
#define Convert_ADC1_low  Write_A &= ~(Un<<15)

  const int  CONVERT2 = 25;    // PORT D0
#define Convert_ADC2_high Write_D |= Un<<0
#define Convert_ADC2_low  Write_D &= ~(Un<<0)

  const int  CONVERT3 = 22;    // PORT B26
#define Convert_ADC3_high Write_B |= Un<<26
#define Convert_ADC3_low  Write_B &= ~(Un<<26)

  const int  CONVERT4 = 23;    // PORT A14
#define Convert_ADC4_high Write_A |= Un<<14
#define Convert_ADC4_low  Write_A &= ~(Un<<14)

  const int  RESET = 42    // PORT A19
  #define Reset_ADC_high Write_A |= Un<<19  
  #define Reset_ADC_low  Write_A &= ~(Un<<19)
#endif

/***************************************************************************************Lionel_PCB*****************************************************************************/
#ifdef Lionel_PCB  //BUSY:28, CS:31 (A7), Read:32 (D10), CONVERT:25 (D0), RESET:24 (A15)
    const int BUSY1 = 28;
    const int BUSY2 = 28;
    const int BUSY3 = 28;
    const int BUSY4 = 28;  
    const int CS_ADC1 = 32;                   // PORT D10 chip select
    const int CS_ADC2 = 31;                   // PORT A7 chip select
    const int CS_ADC3 = 31;                   // PORT A7 chip select
    const int CS_ADC4 = 31;                   // PORT A7 chip select
#define CS_ADC1_high Write_D |= Un<<10       // SHOULD BE Write_A = Read_A | Un<<7  But this version is simpler and it works
#define CS_ADC1_low  Write_D &= ~(Un<<10)

  const int  CONVERT1 = 25;    // PORT D0
  const int  CONVERT2 = 25;    // PORT D0
  const int  CONVERT3 = 25;    // PORT D0
  const int  CONVERT4 = 25;    // PORT D0
#define Convert_ADC1_high Write_D |= Un
#define Convert_ADC1_low  Write_D &= ~Un

  const int Read_ADC = 31;    // PORT A7
#define Read_ADC_high Write_A |= Un<<7
#define Read_ADC_low  Write_A &= ~(Un<<7)
 /* const int Read_ADC = 32;    // PORT D10
#define Read_ADC_high Write_D |= Un<<10
#define Read_ADC_low  Write_D &= ~(Un<<10)*/

  const int  RESET = 24;    // PORT A15
#define Reset_ADC_high Write_A |= Un<<15   
#define Reset_ADC_low  Write_A &= ~(Un<<15)
 
#endif

/***************************************************************************************Olivier_WIRING*****************************************************************************/

#ifdef Olivier_WIRING  //BUSY:4, CS:2 (B25), Read:3 (c28), CONVERT:6 (c24), RESET:5 (c25)
  const int BUSY1 = 4;  
  const int BUSY2 = 4;
  const int BUSY3 = 4;
  const int BUSY4 = 4;


  
  const int CS_ADC1 = 2;       // PORT B25 
  const int CS_ADC2 = 2;       // PORT B25 
  const int CS_ADC3 = 2;       // PORT B25 
  const int CS_ADC4 = 2;       // PORT B25 
  #define CS_ADC1_high Write_B |= Un<<25   
  #define CS_ADC1_low  Write_B &= ~(Un<<25)

  const int Read_ADC = 3;    // PORT C28
  #define Read_ADC_high Write_C |= Un<<28
  #define Read_ADC_low  Write_C &= ~(Un<<28)

  const int  CONVERT1 = 6;    // PORT C24
  const int  CONVERT2 = 6;    // PORT C24
  const int  CONVERT3 = 6;    // PORT C24
  const int  CONVERT4 = 6;    // PORT C24
  #define Convert_ADC1_high Write_C |= Un<<24
  #define Convert_ADC1_low  Write_C &= ~(Un<<24)

  const int  RESET = 5;    // PORT C25
  #define Reset_ADC_high Write_C |= Un<<25   
  #define Reset_ADC_low  Write_C &= ~(Un<<25)
/* 
// C1 TO C9 ARE PLUGGED TO D0-D8 (9 Bits) AND /C12 TO C18 ARE PLUGGED TO D9-D15 . (7 Bits) Note that D is for data from ADC
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
*/
#endif

#ifdef LEDKEY

#include <TM1638plus.h>
#define  STROBE_TM 10     //original pin 4
#define  CLOCK_TM 11     //original pin 6
#define  DIO_TM 12      //original pin7

//Constructor object
TM1638plus tm(STROBE_TM, CLOCK_TM , DIO_TM);
#endif

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

/*  pinMode(DB0, INPUT);//INPUT_PULLUP);
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
  pinMode(DB15, INPUT);//INPUT_PULLUP);*/
