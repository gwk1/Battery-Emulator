#include "../include.h"

#ifdef BYD_KOSTAL_RS485
#include "KOSTAL_RS485.h"
#endif

 KOSTAL_FRAMEHEADER[5]= {0x62,0xFF,0x02,0xFF,0x29}

// KOSTAL_FRAME1[10]={0x09,0x62,0xFF,0x02,0xFF,0x29,0x4A,0x08,0x23,0x00};
// KOSTAL_FRAME2[10]={0x09,0x62,0xFF,0x02,0xFF,0x29,0x4A,0x04,0x27,0x00};
// KOSTAL_FRAME3[10]={0x09,0x62,0xFF,0x02,0xFF,0x29,0x53,0x03,0x1F,0x00};

byte RS485_RXFRAME[10];
size_t RS485_RECEIVEDBYTES;

boolean check_kostal_frame_crc() {
    unsigned int sum = 0;
    for (int i = 1; i < 7; ++i) {
      sum += RS485_RXFRAME[i]; 
      }
    }
  return (if (~sum+1)&0xff) == RS485_RXFRAME[8]));
  }

  
void init_kostal_byd(){
  }


void run_kostal_byd(){
  if Serial2.available() {
    RS485_RECEIVEDBYTES=Serial.readBytesUntil(0x00, RS485_RXFRAME, 10)    
    // frame length 10 bytes and starts with 0x09
    if((RS485_RECEIVEDBYTES == 10) && (RS485_RXFRAME[0] == 0x09)) {
      if(check_kostal_frame_crc())
        {
        boolean notframe=0;
        for (int i = 0;  i < 5; i++)
          {
          if( RS485_RXFRAME[i+1] != KOSTAL_FRAMEHEADER[i] ) 
            {
            notframe=true;
            break;
            }
          }
        }
      }
    }
  }


void update_values_kostal_byd(){
  }
