#include "../include.h"
#include "../datalayer/datalayer.h"


#ifdef BYD_KOSTAL_RS485
#include "KOSTAL-RS485.h"
#endif

static const byte KOSTAL_FRAMEHEADER[5]= {0x62,0xFF,0x02,0xFF,0x29};

// KOSTAL_FRAME1[10]={0x09,0x62,0xFF,0x02,0xFF,0x29,0x4A,0x08,0x23,0x00};
// KOSTAL_FRAME2[10]={0x09,0x62,0xFF,0x02,0xFF,0x29,0x4A,0x04,0x27,0x00};
// KOSTAL_FRAME3[10]={0x09,0x62,0xFF,0x02,0xFF,0x29,0x53,0x03,0x1F,0x00};

struct LongFrame {
              byte    header[6];
              float   BatteryVoltage;
              float   MaxVoltage;
              float   Temp;
              float   Current1;
              float   Current2;
              float   PeakDischarge;
              float   NominalDischarge;
              byte    Unknown[4];           //Max charge current?
              float   MaxCellTemp;
              float   MinCellTemp;
              float   MaxCellVolt;
              float   MinCellVolt;
              byte    cyclecount;
              byte    Unknown2[3];
              byte    SOC;
              byte    Unknown3[3];
              byte    CRC;
              byte    endbyte;
              };


union LongFrameC {
             LongFrame     FrameIn;
             byte          Frameout[64];
             };

LongFrame frame2;

byte frame1[40]={0x06,0xE2,0xFF,0x02,0xFF,0x29,
             0x01,0x08,0x80,0x43,     // 256.063 Max Charge?
             0xE4,0x70,0x8A,0x5C,
             0xB5,0x02,0xD3,0x01,
             0x01,0x05,0xC8,0x41,     // 25.0024  TEMP??
             0xC2,0x18,0x01,0x03,
             0x59,0x42,0x01,0x01,
             0x01,0x02,0x05,0x02,
             0xA0,0x01,0x01,0x02,
             0x4D,0x00};

byte RS485_RXFRAME[10];
size_t RS485_RECEIVEDBYTES;

byte calculate_2s_crc(byte *lfc, int lastbyte) {
  unsigned int sum = 0;
  for (int i = 1; i < lastbyte; ++i) {
    sum += lfc[i];
    }
  return((byte)(~sum&+1)+0xff);
}

boolean check_kostal_frame_crc() {
  unsigned int sum = 0;
  for (int i = 1; i < 8; ++i) {
    sum += RS485_RXFRAME[i];
    }

if( ((~sum+1)&0xff) == (RS485_RXFRAME[8]&0xff) )
    {
    return(true);
    }
  else
    {
    return(false);
    }
  }

void init_kostal_byd()
  {
  frame2.header[0] = 0x0A;
  frame2.header[1] = 0xE2;
  frame2.header[2] = 0xFF;
  frame2.header[3] = 0X02;
  frame2.header[4] = 0xFF;
  frame2.header[5] = 0x29;

  frame2.Unknown[0]=0x01;
  frame2.Unknown[1]=0x01;
  frame2.Unknown[2]=0x01;
  frame2.Unknown[3]=0x05;

  frame2.Unknown2[0]=0x04;
  frame2.Unknown2[1]=0x01;
  frame2.Unknown2[2]=0x20;

  frame2.Unknown3[0]=0x01;
  frame2.Unknown3[1]=0x01;
  frame2.Unknown3[2]=0x02;
  frame2.endbyte = 0x00;
  }

byte incomingindex=0;

void run_kostal_byd(){
  if (Serial2.available()) {
      RS485_RXFRAME[incomingindex]=Serial2.read();
      Serial.print(RS485_RXFRAME[incomingindex], HEX);
      Serial.print(":");
    incomingindex++;
    //if(  RS485_RXFRAME[incomingindex] == 0x00 )

    if((incomingindex == 10) && (RS485_RXFRAME[0] == 0x09) ) {
      Serial.println("\n");
#ifdef DEBUG_KOSTAL_RS485_DATA
      Serial.print("RX: ");
      for (int i = 0;  i < 10; i++) {
         Serial.print(RS485_RXFRAME[i], HEX);
         Serial.print(" ");
         }
      Serial.println("");
#endif

      incomingindex=0;
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
        if (!notframe && (RS485_RXFRAME[6]==0x4A) && (RS485_RXFRAME[7]==0x08))
          {
          Serial2.write(frame1,40);
          }
        if (!notframe && (RS485_RXFRAME[6]==0x4A) && (RS485_RXFRAME[7]==0x04 ))
          {
            LongFrameC tmpframe;
            tmpframe.FrameIn=frame2;
            for(int i=1; i < 63; i++)
              {
              if(tmpframe.Frameout[i]==0x00) {
                 tmpframe.Frameout[i]=0x01;
              }
              Serial.print(tmpframe.Frameout[i], HEX);
              }
            Serial.println();
            tmpframe.Frameout[62]=calculate_2s_crc(tmpframe.Frameout,62)+0xc1;
            for(int i=1; i < 63; i++)
              {
              Serial.print(tmpframe.Frameout[i], HEX);
              }
            Serial.println();
            Serial2.write(tmpfrane.Frameout,64);
          }
        }
      }
    }
  }

void update_values_kostal_byd(){
  Serial.println("Update values");
  frame2.BatteryVoltage=(float)datalayer.battery.status.voltage_dV/10;
  frame2.MaxVoltage=(float)datalayer.battery.info.max_design_voltage_dV/10;
  frame2.Temp=(float)(datalayer.battery.status.temperature_max_dC+datalayer.battery.status.temperature_min_dC)/20;
  frame2.Current1=(float)datalayer.battery.status.current_dA/10;
//   frame2.Current2;
   frame2.PeakDischarge=(float)datalayer.battery.info.max_discharge_amp_dA/10;
//              float   NominalDischarge;
   frame2.MaxCellTemp=(float)datalayer.battery.status.temperature_max_dC/10;
   frame2.MinCellTemp=(float)datalayer.battery.status.temperature_min_dC/10;
   frame2.MaxCellVolt=(float)datalayer.battery.status.cell_max_voltage_mV/1000;
   frame2.MinCellVolt=(float)datalayer.battery.status.cell_min_voltage_mV/1000;
//             byte    cyclecount;
   frame2.SOC=(byte)datalayer.battery.status.real_soc/100;

//datalayer.battery.status.cell_max_voltage_mV
//
//datalayer.battery.status.current_dA
//datalayer.battery.status.real_soc
//datalayer.battery.info.max_design_voltage_dV
//datalayer.battery.info.max_design_voltage_dV


  }
