#include "JK-BMS-PB-CAN.h"
#include "../battery/BATTERIES.h"
#include "../communication/can/comm_can.h"
#include "../communication/nvm/comm_nvm.h"
#include "../datalayer/datalayer.h"
#include "../devboard/utils/events.h"
#include "../devboard/utils/logging.h"

void JkBmsPbCan::update_values() {

  datalayer.battery.status.real_soc = soc_percent * 100;  // pptt (0-10000)
  datalayer.battery.status.soh_pptt = soh_percent * 100;

  datalayer.battery.status.remaining_capacity_Wh = static_cast<uint32_t>(
      (static_cast<double>(datalayer.battery.status.real_soc) / 10000) * datalayer.battery.info.total_capacity_Wh);

  datalayer.battery.status.voltage_dV = pack_voltage_dV;
  datalayer.battery.status.current_dA = pack_current_dA;

  // Use BMS-reported limits directly — the BMS knows best
  if (charge_current_limit_dA > 0 && pack_voltage_dV > 0) {
    datalayer.battery.status.max_charge_power_W =
        (static_cast<uint32_t>(charge_current_limit_dA) * pack_voltage_dV) / 100;
  } else {
    datalayer.battery.status.max_charge_power_W = 0;
  }

  if (discharge_current_limit_dA > 0 && pack_voltage_dV > 0) {
    datalayer.battery.status.max_discharge_power_W =
        (static_cast<uint32_t>(discharge_current_limit_dA) * pack_voltage_dV) / 100;
  } else {
    datalayer.battery.status.max_discharge_power_W = 0;
  }

  // Use BMS voltage limits to update design voltage dynamically
  if (charge_voltage_limit_dV > 0) {
    datalayer.battery.info.max_design_voltage_dV = charge_voltage_limit_dV;
  }
  if (discharge_voltage_limit_dV > 0) {
    datalayer.battery.info.min_design_voltage_dV = discharge_voltage_limit_dV;
  }

  // Temperature — PB CAN only sends average, use for both min/max
  datalayer.battery.status.temperature_max_dC = pack_temperature_dC;
  datalayer.battery.status.temperature_min_dC = pack_temperature_dC;

  // Block charge/discharge on active protections
  if (prot_overvoltage || prot_charge_overcurrent || prot_overtemperature) {
    datalayer.battery.status.max_charge_power_W = 0;
  }
  if (prot_undervoltage || prot_discharge_overcurrent || prot_undertemperature) {
    datalayer.battery.status.max_discharge_power_W = 0;
  }

  // Force charge override — BMS is requesting immediate charge (critically low SOC)
  // Override protection blocks and ensure a minimum charge power so the inverter acts
  if (force_charge_requested && datalayer.battery.status.max_charge_power_W == 0 && pack_voltage_dV > 0) {
    // Allow at least 500W for emergency recovery charging
    datalayer.battery.status.max_charge_power_W = 500;
  }

  datalayer.battery.status.real_bms_status = has_any_protection ? BMS_FAULT : BMS_ACTIVE;

  // Alarm events
  if (prot_overvoltage || warn_overvoltage) {
    set_event(EVENT_BATTERY_OVERVOLTAGE, 0);
  } else {
    clear_event(EVENT_BATTERY_OVERVOLTAGE);
  }
  if (prot_undervoltage || warn_undervoltage) {
    set_event(EVENT_BATTERY_UNDERVOLTAGE, 0);
  } else {
    clear_event(EVENT_BATTERY_UNDERVOLTAGE);
  }
  if (prot_overtemperature || warn_overtemperature) {
    set_event(EVENT_BATTERY_OVERHEAT, 0);
  } else {
    clear_event(EVENT_BATTERY_OVERHEAT);
  }
  if (prot_undertemperature || warn_undertemperature) {
    set_event(EVENT_BATTERY_FROZEN, 0);
  } else {
    clear_event(EVENT_BATTERY_FROZEN);
  }
  if (prot_charge_overcurrent || warn_charge_overcurrent) {
    set_event(EVENT_CHARGE_LIMIT_EXCEEDED, 0);
  } else {
    clear_event(EVENT_CHARGE_LIMIT_EXCEEDED);
  }
  if (prot_discharge_overcurrent || warn_discharge_overcurrent) {
    set_event(EVENT_DISCHARGE_LIMIT_EXCEEDED, 0);
  } else {
    clear_event(EVENT_DISCHARGE_LIMIT_EXCEEDED);
  }
}

void JkBmsPbCan::handle_incoming_can_frame(CAN_frame rx_frame) {

  datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;

  switch (rx_frame.ID) {
    case CAN_ID_LIMITS:  // 0x351 — Charge/discharge limits (1000ms)
      // All fields little-endian
      charge_voltage_limit_dV = rx_frame.data.u8[0] | (rx_frame.data.u8[1] << 8);
      charge_current_limit_dA = static_cast<int16_t>(rx_frame.data.u8[2] | (rx_frame.data.u8[3] << 8));
      discharge_current_limit_dA = static_cast<int16_t>(rx_frame.data.u8[4] | (rx_frame.data.u8[5] << 8));
      discharge_voltage_limit_dV = rx_frame.data.u8[6] | (rx_frame.data.u8[7] << 8);
      if (enhanced_logging) {
        logging.printf("[JKPB] 0x351 CVL=%u.%uV CCL=%d.%dA DCL=%d.%dA DVL=%u.%uV\n",
                       charge_voltage_limit_dV / 10, charge_voltage_limit_dV % 10,
                       charge_current_limit_dA / 10, abs(charge_current_limit_dA) % 10,
                       discharge_current_limit_dA / 10, abs(discharge_current_limit_dA) % 10,
                       discharge_voltage_limit_dV / 10, discharge_voltage_limit_dV % 10);
      }
      break;

    case CAN_ID_SOC_SOH:  // 0x355 — SOC/SOH (1000ms)
      soc_percent = rx_frame.data.u8[0] | (rx_frame.data.u8[1] << 8);
      soh_percent = rx_frame.data.u8[2] | (rx_frame.data.u8[3] << 8);
      if (enhanced_logging) {
        logging.printf("[JKPB] 0x355 SOC=%u%% SOH=%u%%\n", soc_percent, soh_percent);
      }
      break;

    case CAN_ID_TELEMETRY:  // 0x356 — Voltage/Current/Temp (1000ms)
    {
      // Voltage is 0.01V units — convert to 0.1V (dV)
      uint16_t raw_voltage_cV = rx_frame.data.u8[0] | (rx_frame.data.u8[1] << 8);
      pack_voltage_dV = raw_voltage_cV / 10;
      // Current is 0.1A units (positive=charge, matches datalayer convention)
      pack_current_dA = static_cast<int16_t>(rx_frame.data.u8[2] | (rx_frame.data.u8[3] << 8));
      // Temperature is 0.1°C units
      pack_temperature_dC = static_cast<int16_t>(rx_frame.data.u8[4] | (rx_frame.data.u8[5] << 8));
      if (enhanced_logging) {
        logging.printf("[JKPB] 0x356 V=%u.%02uV I=%d.%dA T=%d.%dC\n",
                       raw_voltage_cV / 100, raw_voltage_cV % 100,
                       pack_current_dA / 10, abs(pack_current_dA) % 10,
                       pack_temperature_dC / 10, abs(pack_temperature_dC) % 10);
      }
      break;
    }

    case CAN_ID_PROTECTION:  // 0x359 — Protection & alarm flags (1000ms)
    {
      uint8_t prot_flags = rx_frame.data.u8[0];
      system_fault = rx_frame.data.u8[1];
      uint8_t warn_flags = rx_frame.data.u8[2];

      prot_discharge_overcurrent = (prot_flags >> 1) & 0x01;
      prot_charge_overcurrent = (prot_flags >> 2) & 0x01;
      prot_undervoltage = (prot_flags >> 3) & 0x01;
      prot_overvoltage = (prot_flags >> 4) & 0x01;
      prot_undertemperature = (prot_flags >> 5) & 0x01;
      prot_overtemperature = (prot_flags >> 6) & 0x01;

      warn_discharge_overcurrent = (warn_flags >> 1) & 0x01;
      warn_charge_overcurrent = (warn_flags >> 2) & 0x01;
      warn_undervoltage = (warn_flags >> 3) & 0x01;
      warn_overvoltage = (warn_flags >> 4) & 0x01;
      warn_undertemperature = (warn_flags >> 5) & 0x01;
      warn_overtemperature = (warn_flags >> 6) & 0x01;

      has_any_protection = (prot_flags != 0) || (system_fault != 0);
      if (enhanced_logging) {
        logging.printf("[JKPB] 0x359 PROT=0x%02X FAULT=0x%02X WARN=0x%02X%s%s%s%s%s%s\n",
                       prot_flags, system_fault, warn_flags,
                       prot_overvoltage ? " OVP" : "", prot_undervoltage ? " UVP" : "",
                       prot_charge_overcurrent ? " COP" : "", prot_discharge_overcurrent ? " DOP" : "",
                       prot_overtemperature ? " COT" : "", prot_undertemperature ? " CUT" : "");
      }
      break;
    }

    case CAN_ID_MANUFACTURER:  // 0x35E — Manufacturer name (informational)
      if (enhanced_logging) {
        char mfr[9] = {0};
        memcpy(mfr, rx_frame.data.u8, 8);
        logging.printf("[JKPB] 0x35E Manufacturer: %s\n", mfr);
      }
      break;

    case CAN_ID_BMS_INFO:  // 0x35C — BMS info + force charge/discharge flags
      // Byte 4: Force flags per Pylontech LV protocol
      //   Bit 3 (0x08): Force charge request (BMS SOC critically low)
      //   Bit 2 (0x04): Force charge request II
      //   Bit 1 (0x02): Force discharge request
      //   Bit 5 (0x20): Charge enable
      //   Bit 6 (0x40): Discharge enable
      force_charge_requested = (rx_frame.data.u8[4] & 0x08) || (rx_frame.data.u8[4] & 0x04);
      force_discharge_requested = (rx_frame.data.u8[4] & 0x02);
      if (enhanced_logging) {
        logging.printf("[JKPB] 0x35C Model=0x%02X HW=0x%02X FW=0x%02X Flags=0x%02X ForceChg=%d ForceDis=%d\n",
                       rx_frame.data.u8[0], rx_frame.data.u8[1], rx_frame.data.u8[2],
                       rx_frame.data.u8[4], force_charge_requested, force_discharge_requested);
      }
      break;

    default:
      break;
  }
}

void JkBmsPbCan::transmit_can(unsigned long currentMillis) {
  // JK BMS PB broadcasts autonomously in Pylontech emulation mode.
  // No periodic requests needed from our side.
}

void JkBmsPbCan::setup(void) {
  strncpy(datalayer.system.info.battery_protocol, Name, 63);
  datalayer.system.info.battery_protocol[63] = '\0';
  datalayer.battery.info.chemistry = battery_chemistry_enum::LFP;
  // Let BMS-reported CVL/DVL override these dynamically via 0x351
  datalayer.battery.info.max_design_voltage_dV = user_selected_max_pack_voltage_dV;
  datalayer.battery.info.min_design_voltage_dV = user_selected_min_pack_voltage_dV;
  datalayer.battery.info.max_cell_voltage_mV = user_selected_max_cell_voltage_mV;
  datalayer.battery.info.min_cell_voltage_mV = user_selected_min_cell_voltage_mV;
  datalayer.system.status.battery_allows_contactor_closing = true;

  // Load enhanced logging setting (RAII: constructor opens, destructor closes)
  {
    BatteryEmulatorSettingsStore settings(true);  // read-only
    enhanced_logging = settings.getBool("JKLOG", false);
  }

  if (enhanced_logging) {
    logging.printf("[JKPB] Enhanced logging enabled\n");
  }
}
