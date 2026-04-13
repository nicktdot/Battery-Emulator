#ifndef JK_BMS_CAN_H
#define JK_BMS_CAN_H

#include "../system_settings.h"
#include "CanBattery.h"

/* JK BMS native CAN protocol (Young Power Technology)
 * CAN 2.0A standard frame, 250 kbps, LITTLE-ENDIAN byte order
 * Source address (SA) = 0xF4
 * CAN_ID = (FUNC << 8) | SA
 * Current sign: positive = discharge, negative = charge
 *
 * 0x02F4  BATT_ST    Battery status (voltage, current, SOC, discharge time)  20ms
 * 0x04F4  CELL_VOLT  Max/min cell voltage + cell position                   100ms
 * 0x05F4  CELL_TEMP  Max/min/avg cell temperature + position                100ms
 * 0x07F4  ALM_INFO   Alarm flags (2-bit levels, event-triggered)            100ms when active
 */

class JkBmsCan : public CanBattery {
 public:
  JkBmsCan() : CanBattery(CAN_Speed::CAN_SPEED_250KBPS) {}

  virtual void setup(void);
  virtual void handle_incoming_can_frame(CAN_frame rx_frame);
  virtual void update_values();
  virtual void transmit_can(unsigned long currentMillis);
  static constexpr const char* Name = "JK BMS CAN (B/BD Native)";

 private:
  /* Power ramping constants */
  static const int MAX_CHARGE_POWER_WHEN_TOPBALANCING_W = 500;
  static const int RAMPDOWN_SOC = 9000;  // 90.00%

  /* JK CAN protocol IDs per spec */
  static const uint32_t CAN_ID_BATT_ST = 0x02F4;
  static const uint32_t CAN_ID_CELL_VOLT = 0x04F4;
  static const uint32_t CAN_ID_CELL_TEMP = 0x05F4;
  static const uint32_t CAN_ID_ALM_INFO = 0x07F4;

  /* Decoded from 0x02F4 BATT_ST */
  uint16_t pack_voltage_dV = 0;           // 0.1V resolution
  int16_t pack_current_dA = 0;            // 0.1A, positive=discharge, negative=charge
  uint8_t soc_percent = 0;                // 0-100%
  uint16_t discharge_time_h = 0;          // hours

  /* Decoded from 0x04F4 CELL_VOLT */
  uint16_t cell_max_voltage_mV = 0;       // max cell voltage mV
  uint8_t cell_max_voltage_num = 0;       // cell position (1-based)
  uint16_t cell_min_voltage_mV = 0;       // min cell voltage mV
  uint8_t cell_min_voltage_num = 0;       // cell position (1-based)

  /* Decoded from 0x05F4 CELL_TEMP (°C with -50 offset, stored as deci-°C) */
  int16_t temperature_max_dC = 0;
  uint8_t temperature_max_num = 0;        // cell position (1-based)
  int16_t temperature_min_dC = 0;
  uint8_t temperature_min_num = 0;        // cell position (1-based)
  int16_t temperature_avg_dC = 0;

  /* Decoded from 0x07F4 ALM_INFO — 2-bit alarm levels (0=none,1=serious,2=major,3=general) */
  uint8_t alm_cell_overvoltage = 0;       // bits 0-1
  uint8_t alm_cell_undervoltage = 0;      // bits 2-3
  uint8_t alm_pack_overvoltage = 0;       // bits 4-5
  uint8_t alm_pack_undervoltage = 0;      // bits 6-7
  uint8_t alm_cell_imbalance = 0;         // bits 8-9
  uint8_t alm_discharge_overcurrent = 0;  // bits 10-11
  uint8_t alm_charge_overcurrent = 0;     // bits 12-13
  uint8_t alm_overtemperature = 0;        // bits 14-15
  uint8_t alm_undertemperature = 0;       // bits 16-17
  uint8_t alm_temp_difference = 0;        // bits 18-19
  uint8_t alm_soc_low = 0;               // bits 20-21
  uint8_t alm_insulation_low = 0;         // bits 22-23
  uint8_t alm_hv_interlock = 0;           // bits 24-25
  uint8_t alm_ext_comm_fail = 0;          // bits 26-27
  uint8_t alm_int_comm_fail = 0;          // bits 28-29
  bool has_any_alarm = false;

  /* Enhanced logging — enabled via web UI checkbox */
  bool enhanced_logging = false;
};

#endif
