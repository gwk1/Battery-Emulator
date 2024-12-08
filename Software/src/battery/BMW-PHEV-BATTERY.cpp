#include "../include.h"
#ifdef BMW_PHEV_BATTERY
#include "../datalayer/datalayer.h"
#include "../datalayer/datalayer_extended.h"
#include "BMW-PHEV-BATTERY.h"
#include "CRC8.h"

/* Do not change code below unless you are sure what you are doing */
static unsigned long previousMillis10 = 0;   // will store last time a 10ms CAN Message was send
static unsigned long previousMillis100 = 0;  // will store last time a 100ms CAN Message was send
static unsigned long previousMillis5s = 0;  // will store last time a 1s CAN Message was send
static unsigned long previousMillis10s = 0;  // will store last time a 1s CAN Message was send

//float Cell_mV[16];          // calculated as 16 bit value * 6.250 / 16383 = volts


CRC8 crc8;
const uint8_t finalxor[12] = { 0xCF, 0xF5, 0xBB, 0x81, 0x27, 0x1D, 0x53, 0x69, 0x02, 0x38, 0x76, 0x4C };


uint8_t Imod, mescycle = 0;
uint8_t nextmes = 0;
uint8_t testcycle = 0;
uint8_t DMC[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t Unassigned, NextID = 5;
char msgString[128];  
bool balancepauze = 0;
bool resetid=0;

int balancecells;

uint16_t MinCell_mV;
uint16_t MaxCell_mV;

int8_t celltemp[32];


CAN_frame TEST = {.FD = false,
  .ext_ID = false,
  .DLC = 8,
  .ID = 0x080,
  .data = {0xC7, 0x10, 0x00, 0x00, 0x20, 0x00, 0x00, 0x8F}};

void print_units(char* header, int value, char* units) {
  Serial.print(header);
  Serial.print(value);
  Serial.print(units);
}


void printCanFrame(CAN_frame msg){
  Serial.print(" RX ");
  Serial.print(msg.ID, HEX);
  Serial.print(" ");
  Serial.print(msg.DLC);
  Serial.print(" ");
  for (int i = 0; i < msg.DLC; ++i) {
    Serial.print(msg.data.u8[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
}

void update_values_battery() { /* This function puts fake values onto the parameters sent towards the inverter */

  datalayer.system.status.battery_allows_contactor_closing = true;

  datalayer.battery.info.number_of_cells = 96;

  datalayer.battery.status.soh_pptt = 9900;  // 99.00%

  //datalayer.battery.status.voltage_dV = 3700;  // 370.0V , value set in startup in .ino file, editable via webUI

  datalayer.battery.status.current_dA = 0;  // 0 A

  datalayer.battery.info.total_capacity_Wh = 9000;  // 30kWh

  datalayer.battery.status.remaining_capacity_Wh = 8000;  // 15kWh

  MinCell_mV=datalayer.battery.status.cell_voltages_mV[0];
  MaxCell_mV=datalayer.battery.status.cell_voltages_mV[0];
  for (int idx = 1; idx < datalayer.battery.info.number_of_cells; idx++) {
    if(datalayer.battery.status.cell_voltages_mV[idx]<MinCell_mV) {
      MinCell_mV=datalayer.battery.status.cell_voltages_mV[idx];
    }
    if(datalayer.battery.status.cell_voltages_mV[idx]>MaxCell_mV) {
      MaxCell_mV=datalayer.battery.status.cell_voltages_mV[idx];
    }
  }   



//datalayer_extended.bmwphev.needbalanc
  if (MinCell_mV <= MaxCell_mV - 10) {
     datalayer_extended.bmwphev.balancestatus=1;
     if (MinCell_mV > 3900) {
       datalayer_extended.bmwphev.balance_mv=MinCell_mV+10;
     }
     else {
       datalayer_extended.bmwphev.balance_mv=3910;
       }
  }
  else
    {
    datalayer_extended.bmwphev.balancestatus=0;
    datalayer_extended.bmwphev.balance_mv=4200;
  }
  datalayer.battery.status.temperature_min_dC=celltemp[0]*10;
  datalayer.battery.status.temperature_max_dC=celltemp[0]*10;

  for (int idx = 1; idx < 18; idx++) {
    if(celltemp[idx]*10<datalayer.battery.status.temperature_min_dC) {
      datalayer.battery.status.temperature_min_dC=celltemp[idx]*10;
    }
    if(celltemp[idx]*10>datalayer.battery.status.temperature_max_dC) {
      datalayer.battery.status.temperature_max_dC=celltemp[idx]*10;
    }
  }   



  datalayer.battery.status.real_soc = map(MinCell_mV, 3100, 4100, 1000, 9000); //3.1v = 10%, 4.1v=90%
  datalayer.battery.status.cell_max_voltage_mV = MaxCell_mV;
  datalayer.battery.status.cell_min_voltage_mV = MinCell_mV;

  datalayer.battery.status.active_power_W = 0;  // 0W


  datalayer.battery.status.max_discharge_power_W = 5000;  // 5kW

  datalayer.battery.status.max_charge_power_W = 5000;  // 5kW

  //Fake that we get CAN messages
//  datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;

  /*Finally print out values to serial if configured to do so*/
#ifdef DEBUG_VIA_USB
  Serial.println("FAKE Values going to inverter");
  print_units("SOH%: ", (datalayer.battery.status.soh_pptt * 0.01), "% ");
  print_units(", SOC%: ", (datalayer.battery.status.reported_soc * 0.01), "% ");
  print_units(", Voltage: ", (datalayer.battery.status.voltage_dV * 0.1), "V ");
  print_units(", Max discharge power: ", datalayer.battery.status.max_discharge_power_W, "W ");
  print_units(", Max charge power: ", datalayer.battery.status.max_charge_power_W, "W ");
  print_units(", Max temp: ", (datalayer.battery.status.temperature_max_dC * 0.1), "°C ");
  print_units(", Min temp: ", (datalayer.battery.status.temperature_min_dC * 0.1), "°C ");
  print_units(", Max cell voltage: ", datalayer.battery.status.cell_max_voltage_mV, "mV ");
  print_units(", Min cell voltage: ", datalayer.battery.status.cell_min_voltage_mV, "mV ");
  Serial.println("");
#endif
}

void decodetemp(CAN_frame &msg)
{
  int CMU = (msg.ID & 0x00F) ;
  for (int g = 0; g < 3; g++)
    {
    celltemp[g+CMU*3] = msg.data.u8[g] - 40;
    }
}

void decodecandata(int CMU, int Id, CAN_frame &msg, bool Ign){
  if (Id==0) {
//      modules[CMU].error = msg.data.u8[0] + (msg.data.u8[1] << 8) + (msg.data.u8[2] << 16) + (msg.data.u8[3] << 24);
//      modules[CMU].balstat = (msg.data.u8[5]<< 8) + msg.data.u8[4];
/*      Serial.print("Error: ");
    Serial.println(modules[CMU].error);
    Serial.print("Bal state: ");
    Serial.println(modules[CMU].balstat);*/
    }
  int BASE=(CMU-1)*16+(Id-1)*3;
    //transmit_can(&TEST, can_config.battery);
  //printCanFrame(msg);
  if (0<Id & Id<6)
    {
    datalayer.battery.status.cell_voltages_mV[BASE]    = uint16_t(msg.data.u8[0] + (msg.data.u8[1] & 0x3F) * 256) ;
    datalayer.battery.status.cell_voltages_mV[BASE+1]  = uint16_t(msg.data.u8[2] + (msg.data.u8[3] & 0x3F) * 256) ;
    datalayer.battery.status.cell_voltages_mV[BASE+2]  = uint16_t(msg.data.u8[4] + (msg.data.u8[5] & 0x3F) * 256) ;
    }
  if (Id==6)
    {
    datalayer.battery.status.cell_voltages_mV[BASE]    = uint16_t(msg.data.u8[0] + (msg.data.u8[1] & 0x3F) * 256) ;
    }
}


void decodecan(CAN_frame &msg)
{
  int Id = (msg.ID & 0x0F0);
  int CMU = (msg.ID & 0x00F) + 1;

  switch (Id)
  {
    case 0x000:
      Id = 0;  
      break;   
    case 0x020:
      Id = 1;
      break;
    case 0x030:
      Id = 2;
      break;

    case 0x040:
      Id = 3;
      break;

    case 0x050:
      Id = 4;
      break;

    case 0x060:
      Id = 5;
      break;

    case 0x070:
      Id = 6;
      break;
  }
  if (CMU < 14 && Id < 7)
    {
/*    Serial.print(CMU);
    Serial.print(",");
    Serial.print(Id);
    Serial.println();*/
    boolean BalIgnore=0;
      decodecandata(CMU,Id, msg, BalIgnore);
    }
}

void receive_can_battery(CAN_frame rx_frame) {
  datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
  // All CAN messages recieved will be logged via serial
  unsigned long currentMillis = millis();
  // Send 100ms CAN Message
/*  if (currentMillis - previousMillis10 >= INTERVAL_100_MS*3) {
    previousMillis10 = currentMillis;
    Serial.print(millis());  // Example printout, time, ID, length, data: 7553  1DB  8  FF C0 B9 EA 0 0 2 5D
    Serial.print(" RX ");
    Serial.print(rx_frame.ID, HEX);
    Serial.print(" ");
    Serial.print(rx_frame.DLC);
    Serial.print(" ");
    for (int i = 0; i < rx_frame.DLC; ++i) {
      Serial.print(rx_frame.data.u8[i], HEX);
      Serial.print(" ");
    }
    Serial.println("");
  }

*/

  //ID not assigned//
  if (rx_frame.ID == 0xF0) {
    Unassigned++;
    Serial.print(millis());
    if ((rx_frame.ID & 0x80000000) == 0x80000000)  // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rx_frame.ID & 0x1FFFFFFF), rx_frame.DLC);
    else
      sprintf(msgString, ",0x%.3lX,false,%1d", rx_frame.ID, rx_frame.DLC);

    Serial.print(msgString);

    if ((rx_frame.ID & 0x40000000) == 0x40000000) {  // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for (byte i = 0; i < rx_frame.DLC; i++) {
        sprintf(msgString, ", 0x%.2X", rx_frame.data.u8[i]);
        DMC[i] = rx_frame.data.u8[i];
        Serial.print(msgString);
      }
    }

    Serial.println();
    for (byte i = 0; i < 8; i++) {
      Serial.print(DMC[i], HEX);
      Serial.print("|");
    }
    Serial.println();
  }


  if (rx_frame.ID > 0x99 && rx_frame.ID < 0x180) {
      decodecan(rx_frame);  
  }
  if ((rx_frame.ID & 0xFF0) == 0x180) {
      decodetemp(rx_frame);  
  }
}



void findUnassigned() {
  Unassigned = 0;
  //check for found unassigned CSC
  TEST.ID = 0x0A0;  //broadcast to all CSC
  TEST.DLC = 8;
  TEST.ext_ID = 0;
  TEST.data.u8[0] = 0x37;
  TEST.data.u8[1] = 0xFF;
  TEST.data.u8[2] = 0xFF;
  TEST.data.u8[3] = 0xFF;
  TEST.data.u8[4] = 0xFF;
  TEST.data.u8[5] = 0xFF;
  TEST.data.u8[6] = 0xFF;
  TEST.data.u8[7] = 0xFF;

      transmit_can(&TEST, can_config.battery);
//  Can0.write(msg);
}

void assignID() {
  Serial.print("ASSIGN ID: ");
  Serial.println(NextID);

  TEST.ID = 0x0A0;  //broadcast to all CSC
  TEST.DLC = 8;
  TEST.ext_ID = 0;
  TEST.data.u8[0] = 0x12;
  TEST.data.u8[1] = 0xAB;
  TEST.data.u8[2] = DMC[0];
  TEST.data.u8[3] = DMC[1];
  TEST.data.u8[4] = DMC[2];
  TEST.data.u8[5] = DMC[3];
  TEST.data.u8[6] = 0xFF;
  TEST.data.u8[7] = 0xFF;

  transmit_can(&TEST, can_config.battery);

  delay(30);

  TEST.data.u8[1] = 0xBA;
  TEST.data.u8[2] = DMC[4];
  TEST.data.u8[3] = DMC[5];
  TEST.data.u8[4] = DMC[6];
  TEST.data.u8[5] = DMC[7];

 transmit_can(&TEST, can_config.battery);

  delay(10);
  TEST.data.u8[0] = 0x5B;
  TEST.data.u8[1] = NextID;
  transmit_can(&TEST, can_config.battery);

  delay(10);
  TEST.data.u8[0] = 0x37;
  TEST.data.u8[1] = NextID;
  transmit_can(&TEST, can_config.battery);

  NextID++;

  findUnassigned();
}

void resetIDdebug() 
    {

    //Rest all possible Ids
    for (int ID = 0; ID < 15; ID++) {
      TEST.ID = 0x0A0;  //broadcast to all CSC
      TEST.data.u8[0] = 0xA1;
      TEST.data.u8[1] = ID;
      TEST.data.u8[2] = 0xFF;
      TEST.data.u8[3] = 0xFF;
      TEST.data.u8[4] = 0xFF;
      TEST.data.u8[5] = 0xFF;
      TEST.data.u8[6] = 0xFF;
      TEST.data.u8[7] = 0xFF;
      transmit_can(&TEST, can_config.battery);
      delay(2);
    }

    TEST.ID = 0x0A0;  //broadcast to all CSC
    TEST.data.u8[0] = 0x37;
    TEST.data.u8[1] = 0xFF;
    TEST.data.u8[2] = 0xFF;
    TEST.data.u8[3] = 0xFF;
    TEST.data.u8[4] = 0xFF;
    TEST.data.u8[5] = 0xFF;
    TEST.data.u8[6] = 0xFF;
    TEST.data.u8[7] = 0xFF;

    transmit_can(&TEST, can_config.battery);
}

uint8_t getcheck(CAN_frame &msg, int id) {
  unsigned char canmes[11];
  int meslen = msg.DLC + 1;  //remove one for crc and add two for id bytes
  canmes[1] = msg.ID;
  canmes[0] = msg.ID >> 8;

  for (int i = 0; i < (msg.DLC - 1); i++) {
    canmes[i + 2] = msg.data.u8[i];
  }
  
/*    Serial.println();
    for (int i = 0; i <  meslen; i++)
    {
    Serial.print(canmes[i], HEX);
    Serial.print("|");
    }
  */
  return (crc8.get_crc8(canmes, meslen, finalxor[id]));
}

void send_can_battery() {

  if (Unassigned) {
    Unassigned=0;
    assignID();
  }

  if (!resetid) {
//    resetIDdebug();
    resetid=1;
    Serial.println("Reset ID debug");
  }
  
  unsigned long currentMillis = millis();
  // Send 100ms CAN Message
  if (currentMillis - previousMillis5s >= INTERVAL_10_MS) {
    previousMillis5s = currentMillis;
    // Put fake messages here incase you want to test sending CAN


    if (nextmes == 6) {
      mescycle++;
      nextmes = 0;
      if (testcycle < 4) {
        testcycle++;
      }

      if (mescycle == 0xF) {
          mescycle = 0;
      }
    }

    TEST.ID = 0x080 | (nextmes);


    if(!datalayer_extended.bmwphev.balancestatus) {
      TEST.data.u8[0] = 0xC7;
      TEST.data.u8[1] = 0x10;
    }
    else {
      TEST.data.u8[0] = lowByte(datalayer_extended.bmwphev.balance_mv);
      TEST.data.u8[1] = highByte(datalayer_extended.bmwphev.balance_mv);
    }

    TEST.data.u8[2] = 0x00;  //balancing bits
    TEST.data.u8[3] = 0x00;  //balancing bits

  if (testcycle < 3) {
    TEST.data.u8[4] = 0x20;
    TEST.data.u8[5] = 0x00;
  } else {
    if(!datalayer_extended.bmwphev.balancestatus) {
      TEST.data.u8[4] = 0x40;
    }
    else {
      TEST.data.u8[4] = 0x48;
    }
    TEST.data.u8[5] = 0x01;
  }

  TEST.data.u8[6] = mescycle << 4;
  if (testcycle == 2) {
    TEST.data.u8[6] = TEST.data.u8[6] + 0x04;
  }

  TEST.data.u8[7] = getcheck(TEST, nextmes);
  transmit_can(&TEST, can_config.battery);
  nextmes++;
  }
}

void setup_battery(void) {  // Performs one time setup at startup
#ifdef DEBUG_VIA_USB
  Serial.println("Test mode with fake battery selected");

#endif
  crc8.begin();

  strncpy(datalayer.system.info.battery_protocol, "BMW PHEV MODULE", 63);
  datalayer.system.info.battery_protocol[63] = '\0';

  datalayer.battery.info.max_design_voltage_dV =
    4040;  // 404.4V, over this, charging is not possible (goes into forced discharge)
  datalayer.battery.info.min_design_voltage_dV = 2450;  // 245.0V under this, discharging further is disabled

  datalayer.system.status.battery_allows_contactor_closing = true;

  for (int i = 0; i < 96; ++i) {
    datalayer.battery.status.cell_voltages_mV[i] = 1000 ;
  }
//  for (int i = 80; i < 96; ++i) {
//    datalayer.battery.status.cell_voltages_mV[i] = 3600+i ;
//  }

}

#endif
