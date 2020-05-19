#include <SPI.h>
#include <stdint.h>
#define Convert 8
#define CSelect 10
#define Convert_LOW PORTB &= 0xef
#define Convert_HIGH PORTB|=0x10
#define CS_LOW PORTB&=0xbf
#define CS_HIGH PORTB|=0x40
//#define DEBUG
//#define XY_MODE .  //generates iX and iY on channel E and F

unsigned char Msg[8];

void setup (void)
{ pinMode(Convert, OUTPUT);
  pinMode(CSelect, OUTPUT);
  pinMode(15, OUTPUT);//clk
  pinMode(14, INPUT);//MISO
  pinMode(16, OUTPUT);//MOSI
  digitalWrite(CSelect, HIGH);  // ensure SS stays high
  SPI.begin ();
  Msg[0] = 0x00; //write #0
  Msg[1] = 0x01; //write #1
  Msg[2] = 0x02; //write #2
  Msg[3] = 0x03; //write #3 and update All
  Msg[4] = 0x04; //write #4
  Msg[5] = 0x05; //write #5
  Msg[6] = 0x06; //write #6
  Msg[7] = 0x27; //write #7 and update All
} // end of setup

void loop (void)
{ int number_of_steps=8;  // we will have number_of_steps x number_of_steps points
  uint16_t di=1024/number_of_steps;
  uint16_t iX,iY,ii,jj,Sigma_Q=32000;
  
  for (iX = 0; iX <= 1023; iX+=(di+di))
     {for (iY = 0; iY <= 1023; iY+=di)
                   Simulate(iX,iY,Sigma_Q);
      ii=iX+di;
       for (jj=0;jj<number_of_steps;jj++) {Simulate(ii,iY,Sigma_Q);iY-=di;}
     }
  
 /* for (iX = 0; iX <= 1023; iX+=di)
     for (iY = 0; iY <= 1023; iY+=di)
        {
         Simulate(iX,iY,Sigma_Q);
        }*/
#ifdef DEBUG  
  delay (4000);
#endif
} // end of loop

void Simulate(uint16_t iX,uint16_t iY,uint16_t SQ)  // we define iX, iY as a number between 0 and 1024 representing a fraction x=iX/1024
    {uint16_t Q[4];
     uint32_t QQ,QL, QR;
     char i;
   #ifndef    XY_MODE
     QQ=SQ;
     QL=(QQ*iX)>>10;
     QR=SQ-QL;
     //Q = (QL*iY)>>10;
     Q[0]=uint16_t ((QL*iY)>>10);
     Q[2]=uint16_t(QL)-Q[0];
     Q[1]=uint16_t((QR*iY)>>10);
     Q[3]=uint16_t(QR)-Q[1];

     for (i=0;i<4;i++)
        {CS_LOW;
        SPI.transfer (Msg[i+4]);// Q1 is in channel E, Q2 is F, Q3 G, Q4 H
        SPI.transfer16 (Q[i]);
        CS_HIGH;      
        }
     delayMicroseconds(50);
     Convert_LOW;
     Convert_HIGH;
  #endif  

 #ifdef    XY_MODE
 unsigned char msgg0=0x36,msgg1=0x35;// X is emitted on G and Y on F
 //unsigned char msgg0=0x36,msgg1=0x37;// X is emitted on G and Y on F
 
for (i=0;i<4;i++)
      {CS_LOW;
        SPI.transfer (0x30+i);//SPI.transfer (Msg[i]);
        SPI.transfer16 (iX<<6);
      CS_HIGH;
      CS_LOW;
        SPI.transfer (0x30+i+4);//SPI.transfer (Msg[i+4]);
        SPI.transfer16 (iY<<6);
      CS_HIGH;
      }
#endif 
 #ifdef DEBUG
     Serial.println();
     Serial.print(" X= ");Serial.print(iX,DEC);Serial.print(" Y= ");Serial.print(iY,DEC); Serial.print(" Sigma Q= ");Serial.println(SQ,DEC);
     SQ=0;
    for (i=0;i<4;i++)
        {Serial.println(Q[i],DEC); 
         SQ+=Q[i];
        }
        Serial.print(" Sigma Q= ");
        Serial.println(SQ,DEC);
 #endif   
    }
