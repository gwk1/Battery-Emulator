#include "../include.h"
#ifdef SMA_CAN
#include "../datalayer/datalayer.h"


Serial2.begin(57600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);




static int16_t discharge_current = 0;
static int16_t charge_current = 0;
static int16_t temperature_average = 0;
static uint16_t ampere_hours_remaining = 0;


void run_kostal_byd(){
}

void update_values_can_inverter() {  //This function maps all the values fetched from battery CAN to the correct CAN messages
  //Calculate values
  charge_current =
      ((datalayer.battery.status.max_charge_power_W * 10) /
       datalayer.battery.status.voltage_dV);  //Charge power in W , max volt in V+1decimal (P=UI, solve for I)
  //The above calculation results in (30 000*10)/3700=81A
  charge_current = (charge_current * 10);  //Value needs a decimal before getting sent to inverter (81.0A)
  if (charge_current > datalayer.battery.info.max_charge_amp_dA) {
    charge_current =
        datalayer.battery.info
            .max_charge_amp_dA;  //Cap the value to the max allowed Amp. Some inverters cannot handle large values.
  }

  discharge_current =
      ((datalayer.battery.status.max_discharge_power_W * 10) /
       datalayer.battery.status.voltage_dV);  //Charge power in W , max volt in V+1decimal (P=UI, solve for I)
  //The above calculation results in (30 000*10)/3700=81A
  discharge_current = (discharge_current * 10);  //Value needs a decimal before getting sent to inverter (81.0A)
  if (discharge_current > datalayer.battery.info.max_discharge_amp_dA) {
    discharge_current =
        datalayer.battery.info
            .max_discharge_amp_dA;  //Cap the value to the max allowed Amp. Some inverters cannot handle large values.
  }

  temperature_average =
      ((datalayer.battery.status.temperature_max_dC + datalayer.battery.status.temperature_min_dC) / 2);

  ampere_hours_remaining = ((datalayer.battery.status.remaining_capacity_Wh / datalayer.battery.status.voltage_dV) *
                            100);  //(WH[10000] * V+1[3600])*100 = 270 (27.0Ah)

  //Map values to CAN messages
  //Maxvoltage (eg 400.0V = 4000 , 16bits long)
  SMA_358.data.u8[0] = (datalayer.battery.info.max_design_voltage_dV >> 8);
  SMA_358.data.u8[1] = (datalayer.battery.info.max_design_voltage_dV & 0x00FF);
  //Minvoltage (eg 300.0V = 3000 , 16bits long)
  SMA_358.data.u8[2] = (datalayer.battery.info.min_design_voltage_dV >>
                        8);  //Minvoltage behaves strange on SMA, cuts out at 56% of the set value?
  SMA_358.data.u8[3] = (datalayer.battery.info.min_design_voltage_dV & 0x00FF);
  //Discharge limited current, 500 = 50A, (0.1, A)
  SMA_358.data.u8[4] = (discharge_current >> 8);
  SMA_358.data.u8[5] = (discharge_current & 0x00FF);
  //Charge limited current, 125 =12.5A (0.1, A)
  SMA_358.data.u8[6] = (charge_current >> 8);
  SMA_358.data.u8[7] = (charge_current & 0x00FF);

  //SOC (100.00%)
  SMA_3D8.data.u8[0] = (datalayer.battery.status.reported_soc >> 8);
  SMA_3D8.data.u8[1] = (datalayer.battery.status.reported_soc & 0x00FF);
  //StateOfHealth (100.00%)
  SMA_3D8.data.u8[2] = (datalayer.battery.status.soh_pptt >> 8);
  SMA_3D8.data.u8[3] = (datalayer.battery.status.soh_pptt & 0x00FF);
  //State of charge (AH, 0.1)
  SMA_3D8.data.u8[4] = (ampere_hours_remaining >> 8);
  SMA_3D8.data.u8[5] = (ampere_hours_remaining & 0x00FF);

  //Voltage (370.0)
  SMA_4D8.data.u8[0] = (datalayer.battery.status.voltage_dV >> 8);
  SMA_4D8.data.u8[1] = (datalayer.battery.status.voltage_dV & 0x00FF);
  //Current (TODO: signed OK?)
  SMA_4D8.data.u8[2] = (datalayer.battery.status.current_dA >> 8);
  SMA_4D8.data.u8[3] = (datalayer.battery.status.current_dA & 0x00FF);
  //Temperature average
  SMA_4D8.data.u8[4] = (temperature_average >> 8);
  SMA_4D8.data.u8[5] = (temperature_average & 0x00FF);
  //Battery ready
  if (datalayer.battery.status.bms_status == ACTIVE) {
    SMA_4D8.data.u8[6] = READY_STATE;
  } else {
    SMA_4D8.data.u8[6] = STOP_STATE;
  }

  //Error bits
  /*
  //SMA_158.data.u8[0] = //bit12 Fault high temperature, bit34Battery cellundervoltage, bit56 Battery cell overvoltage, bit78 batterysystemdefect
  //TODO: add all error bits. Sending message with all 0xAA until that.


*/
}

void receive_can_inverter(CAN_frame_t rx_frame) {
  switch (rx_frame.MsgID) {
    case 0x360:  //Message originating from SMA inverter - Voltage and current
      //Frame0-1 Voltage
      //Frame2-3 Current
      break;
    case 0x420:  //Message originating from SMA inverter - Timestamp
      //Frame0-3 Timestamp
      break;
    case 0x3E0:  //Message originating from SMA inverter - ?
      break;
    case 0x5E0:  //Message originating from SMA inverter - String
      break;
    case 0x560:  //Message originating from SMA inverter - Init
      break;
    default:
      break;
  }
}

void send_can_inverter() {
  unsigned long currentMillis = millis();

  // Send CAN Message every 100ms
  if (currentMillis - previousMillis100ms >= INTERVAL_100_MS) {
    previousMillis100ms = currentMillis;

    ESP32Can.CANWriteFrame(&SMA_558);
    ESP32Can.CANWriteFrame(&SMA_598);
    ESP32Can.CANWriteFrame(&SMA_5D8);
    ESP32Can.CANWriteFrame(&SMA_618_1);  // TODO, should these 3x
    ESP32Can.CANWriteFrame(&SMA_618_2);  // be sent as batch?
    ESP32Can.CANWriteFrame(&SMA_618_3);  // or alternate on each send?
    ESP32Can.CANWriteFrame(&SMA_358);
    ESP32Can.CANWriteFrame(&SMA_3D8);
    ESP32Can.CANWriteFrame(&SMA_458);
    ESP32Can.CANWriteFrame(&SMA_518);
    ESP32Can.CANWriteFrame(&SMA_4D8);
    ESP32Can.CANWriteFrame(&SMA_158);
  }
}
#endif
