#include "JK-BMS-CAN.h"
#include "../battery/BATTERIES.h"
#include "../communication/can/comm_can.h"
#include "../communication/nvm/comm_nvm.h"
#include "../datalayer/datalayer.h"
#include "../devboard/utils/events.h"
#include "../devboard/utils/logging.h"

void JkBmsCan::update_values() {

  datalayer.battery.status.real_soc = soc_percent * 100;  // pptt (0-10000)

  datalayer.battery.status.remaining_capacity_Wh = static_cast<uint32_t>(
      (static_cast<double>(datalayer.battery.status.real_soc) / 10000) * datalayer.battery.info.total_capacity_Wh);

  datalayer.battery.status.voltage_dV = pack_voltage_dV;
  // JK BMS convention: positive=discharge, negative=charge
  // Datalayer convention: positive=charge, negative=discharge — invert
  datalayer.battery.status.current_dA = -pack_current_dA;

  // Charge power ramping
  bool charge_blocked = (alm_charge_overcurrent > 0) || (alm_overtemperature > 0) || (alm_pack_overvoltage > 0) ||
                         (alm_cell_overvoltage > 0);
  if (charge_blocked) {
    datalayer.battery.status.max_charge_power_W = 0;
  } else if (datalayer.battery.status.real_soc > 9900) {
    datalayer.battery.status.max_charge_power_W = MAX_CHARGE_POWER_WHEN_TOPBALANCING_W;
  } else if (datalayer.battery.status.real_soc > RAMPDOWN_SOC) {
    datalayer.battery.status.max_charge_power_W =
        datalayer.battery.status.override_charge_power_W *
        (1 - (datalayer.battery.status.real_soc - RAMPDOWN_SOC) / (10000.0 - RAMPDOWN_SOC));
  } else {
    datalayer.battery.status.max_charge_power_W = datalayer.battery.status.override_charge_power_W;
  }

  // Discharge power
  bool discharge_blocked = (alm_discharge_overcurrent > 0) || (alm_undertemperature > 0) ||
                            (alm_pack_undervoltage > 0) || (alm_cell_undervoltage > 0);
  if (discharge_blocked) {
    datalayer.battery.status.max_discharge_power_W = 0;
  } else {
    datalayer.battery.status.max_discharge_power_W = datalayer.battery.status.override_discharge_power_W;
  }

  datalayer.battery.status.temperature_max_dC = temperature_max_dC;
  datalayer.battery.status.temperature_min_dC = temperature_min_dC;

  datalayer.battery.status.cell_max_voltage_mV = cell_max_voltage_mV;
  datalayer.battery.status.cell_min_voltage_mV = cell_min_voltage_mV;

  // BMS status — any alarm level ≥1 (serious) is a fault
  datalayer.battery.status.real_bms_status = has_any_alarm ? BMS_FAULT : BMS_ACTIVE;

  // Alarm events
  if (alm_cell_overvoltage || alm_pack_overvoltage) {
    set_event(EVENT_BATTERY_OVERVOLTAGE, 0);
  } else {
    clear_event(EVENT_BATTERY_OVERVOLTAGE);
  }
  if (alm_cell_undervoltage || alm_pack_undervoltage) {
    set_event(EVENT_BATTERY_UNDERVOLTAGE, 0);
  } else {
    clear_event(EVENT_BATTERY_UNDERVOLTAGE);
  }
  if (alm_overtemperature) {
    set_event(EVENT_BATTERY_OVERHEAT, 0);
  } else {
    clear_event(EVENT_BATTERY_OVERHEAT);
  }
  if (alm_undertemperature) {
    set_event(EVENT_BATTERY_FROZEN, 0);
  } else {
    clear_event(EVENT_BATTERY_FROZEN);
  }
  if (alm_charge_overcurrent) {
    set_event(EVENT_CHARGE_LIMIT_EXCEEDED, 0);
  } else {
    clear_event(EVENT_CHARGE_LIMIT_EXCEEDED);
  }
  if (alm_discharge_overcurrent) {
    set_event(EVENT_DISCHARGE_LIMIT_EXCEEDED, 0);
  } else {
    clear_event(EVENT_DISCHARGE_LIMIT_EXCEEDED);
  }
}

void JkBmsCan::handle_incoming_can_frame(CAN_frame rx_frame) {

  datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;

  switch (rx_frame.ID) {
    case CAN_ID_BATT_ST:  // 0x02F4 — Battery status (20ms cycle)
      // All multi-byte fields are LITTLE-ENDIAN per spec
      // Bytes[0:1] LE uint16: BattVolt × 0.1V
      pack_voltage_dV = rx_frame.data.u8[0] | (rx_frame.data.u8[1] << 8);
      // Bytes[2:3] LE uint16: BattCurr × 0.1A, offset -400
      //   actual_A = raw × 0.1 − 400  →  in dA: raw − 4000
      //   Positive = discharge, negative = charge (JK convention)
      {
        uint16_t raw_current = rx_frame.data.u8[2] | (rx_frame.data.u8[3] << 8);
        pack_current_dA = static_cast<int16_t>(raw_current) - 4000;
      }
      // Byte[4]: SOC 0-100%
      soc_percent = rx_frame.data.u8[4];
      // Byte[5]: unused
      // Bytes[6:7] LE uint16: DischgTime in hours
      discharge_time_h = rx_frame.data.u8[6] | (rx_frame.data.u8[7] << 8);
      if (enhanced_logging) {
        logging.printf("[JKNAT] 0x02F4 V=%u.%uV I=%d.%dA SOC=%u%% DischT=%uh\n",
                       pack_voltage_dV / 10, pack_voltage_dV % 10,
                       pack_current_dA / 10, abs(pack_current_dA) % 10,
                       soc_percent, discharge_time_h);
      }
      break;

    case CAN_ID_CELL_VOLT:  // 0x04F4 — Cell voltage extremes (100ms cycle)
      cell_max_voltage_mV = rx_frame.data.u8[0] | (rx_frame.data.u8[1] << 8);
      cell_max_voltage_num = rx_frame.data.u8[2];
      cell_min_voltage_mV = rx_frame.data.u8[3] | (rx_frame.data.u8[4] << 8);
      cell_min_voltage_num = rx_frame.data.u8[5];
      if (enhanced_logging) {
        logging.printf("[JKNAT] 0x04F4 MaxCell=%umV(#%u) MinCell=%umV(#%u) Delta=%umV\n",
                       cell_max_voltage_mV, cell_max_voltage_num,
                       cell_min_voltage_mV, cell_min_voltage_num,
                       cell_max_voltage_mV - cell_min_voltage_mV);
      }
      break;

    case CAN_ID_CELL_TEMP:  // 0x05F4 — Cell temperature (100ms cycle)
      temperature_max_dC = (static_cast<int16_t>(rx_frame.data.u8[0]) - 50) * 10;
      temperature_max_num = rx_frame.data.u8[1];
      temperature_min_dC = (static_cast<int16_t>(rx_frame.data.u8[2]) - 50) * 10;
      temperature_min_num = rx_frame.data.u8[3];
      temperature_avg_dC = (static_cast<int16_t>(rx_frame.data.u8[4]) - 50) * 10;
      if (enhanced_logging) {
        logging.printf("[JKNAT] 0x05F4 MaxT=%d.%dC(#%u) MinT=%d.%dC(#%u) AvgT=%d.%dC\n",
                       temperature_max_dC / 10, abs(temperature_max_dC) % 10, temperature_max_num,
                       temperature_min_dC / 10, abs(temperature_min_dC) % 10, temperature_min_num,
                       temperature_avg_dC / 10, abs(temperature_avg_dC) % 10);
      }
      break;

    case CAN_ID_ALM_INFO:  // 0x07F4 — Alarm info (event-triggered, 100ms when active)
    {
      uint32_t alm = rx_frame.data.u8[0] | (rx_frame.data.u8[1] << 8) |
                     (rx_frame.data.u8[2] << 16) | (rx_frame.data.u8[3] << 24);
      alm_cell_overvoltage = (alm >> 0) & 0x03;
      alm_cell_undervoltage = (alm >> 2) & 0x03;
      alm_pack_overvoltage = (alm >> 4) & 0x03;
      alm_pack_undervoltage = (alm >> 6) & 0x03;
      alm_cell_imbalance = (alm >> 8) & 0x03;
      alm_discharge_overcurrent = (alm >> 10) & 0x03;
      alm_charge_overcurrent = (alm >> 12) & 0x03;
      alm_overtemperature = (alm >> 14) & 0x03;
      alm_undertemperature = (alm >> 16) & 0x03;
      alm_temp_difference = (alm >> 18) & 0x03;
      alm_soc_low = (alm >> 20) & 0x03;
      alm_insulation_low = (alm >> 22) & 0x03;
      alm_hv_interlock = (alm >> 24) & 0x03;
      alm_ext_comm_fail = (alm >> 26) & 0x03;
      alm_int_comm_fail = (alm >> 28) & 0x03;
      has_any_alarm = (alm != 0);
      if (enhanced_logging) {
        logging.printf("[JKNAT] 0x07F4 ALM=0x%08X%s%s%s%s%s%s%s%s\n", alm,
                       alm_cell_overvoltage ? " CellOV" : "", alm_cell_undervoltage ? " CellUV" : "",
                       alm_pack_overvoltage ? " PackOV" : "", alm_pack_undervoltage ? " PackUV" : "",
                       alm_charge_overcurrent ? " ChgOC" : "", alm_discharge_overcurrent ? " DisOC" : "",
                       alm_overtemperature ? " OT" : "", alm_undertemperature ? " UT" : "");
      }
      break;
    }

    default:
      break;
  }
}

void JkBmsCan::transmit_can(unsigned long currentMillis) {
  // JK BMS native CAN protocol: BMS is the transmitter.
  // No periodic requests needed from our side.
  // The BMS broadcasts status frames autonomously.
}

void JkBmsCan::setup(void) {
  strncpy(datalayer.system.info.battery_protocol, Name, 63);
  datalayer.system.info.battery_protocol[63] = '\0';
  datalayer.battery.info.max_design_voltage_dV = user_selected_max_pack_voltage_dV;
  datalayer.battery.info.min_design_voltage_dV = user_selected_min_pack_voltage_dV;
  datalayer.battery.info.max_cell_voltage_mV = user_selected_max_cell_voltage_mV;
  datalayer.battery.info.min_cell_voltage_mV = user_selected_min_cell_voltage_mV;
  datalayer.battery.info.chemistry = battery_chemistry_enum::LFP;
  datalayer.battery.status.soh_pptt = 9900;  // JK CAN doesn't report SOH; assume 99%
  datalayer.system.status.battery_allows_contactor_closing = true;

  // Load enhanced logging setting
  {
    BatteryEmulatorSettingsStore settings(true);
    enhanced_logging = settings.getBool("JKLOG", false);
  }

  if (enhanced_logging) {
    logging.printf("[JKNAT] Enhanced logging enabled\n");
  }
}