#ifndef JK_BMS_PB_CAN_H
#define JK_BMS_PB_CAN_H

#include "../system_settings.h"
#include "CanBattery.h"

/* JK BMS PB Series CAN protocol (Pylontech LV emulation)
 * CAN 2.0A standard frame, 500 kbps, LITTLE-ENDIAN
 * JK BMS PB is set to "Pylontech" mode in the JK app
 * Current sign: positive = charge, negative = discharge
 *
 * 0x351  CVL/CCL/DCL/DVL  Charge/discharge voltage & current limits  1000ms
 * 0x355  SOC/SOH          State of charge & health                   1000ms
 * 0x356  Telemetry        Pack voltage, current, temperature         1000ms
 * 0x359  Protection       Protection & alarm bit flags               1000ms
 * 0x35E  Manufacturer     ASCII manufacturer name ("PYLON")          1000ms
 * 0x35C  BMS Info         Model, HW/FW version, force flags          1000ms
 */

class JkBmsPbCan : public CanBattery {
 public:
  JkBmsPbCan() : CanBattery(CAN_Speed::CAN_SPEED_500KBPS) {}

  virtual void setup(void);
  virtual void handle_incoming_can_frame(CAN_frame rx_frame);
  virtual void update_values();
  virtual void transmit_can(unsigned long currentMillis);
  static constexpr const char* Name = "JK BMS PB CAN";

 private:
  /* CAN IDs per Pylontech LV protocol */
  static const uint32_t CAN_ID_LIMITS = 0x351;
  static const uint32_t CAN_ID_SOC_SOH = 0x355;
  static const uint32_t CAN_ID_TELEMETRY = 0x356;
  static const uint32_t CAN_ID_PROTECTION = 0x359;
  static const uint32_t CAN_ID_MANUFACTURER = 0x35E;
  static const uint32_t CAN_ID_BMS_INFO = 0x35C;

  /* Decoded from 0x351 — Limits */
  uint16_t charge_voltage_limit_dV = 0;     // CVL, 0.1V
  int16_t charge_current_limit_dA = 0;      // CCL, 0.1A
  int16_t discharge_current_limit_dA = 0;   // DCL, 0.1A
  uint16_t discharge_voltage_limit_dV = 0;  // DVL, 0.1V

  /* Decoded from 0x355 — SOC/SOH */
  uint16_t soc_percent = 0;  // 0-100%
  uint16_t soh_percent = 0;  // 0-100%

  /* Decoded from 0x356 — Telemetry */
  uint16_t pack_voltage_dV = 0;     // 0.1V (converted from 0.01V raw)
  int16_t pack_current_dA = 0;      // 0.1A, positive=charge
  int16_t pack_temperature_dC = 0;  // 0.1°C

  /* Decoded from 0x359 — Protection flags */
  // Byte 0: Protection status (active protections)
  bool prot_discharge_overcurrent = false;  // bit 1
  bool prot_charge_overcurrent = false;     // bit 2
  bool prot_undervoltage = false;           // bit 3
  bool prot_overvoltage = false;            // bit 4
  bool prot_undertemperature = false;       // bit 5
  bool prot_overtemperature = false;        // bit 6
  // Byte 1: System/module fault
  uint8_t system_fault = 0;
  // Byte 2: Alarm/warning flags (same layout as byte 0)
  bool warn_discharge_overcurrent = false;
  bool warn_charge_overcurrent = false;
  bool warn_undervoltage = false;
  bool warn_overvoltage = false;
  bool warn_undertemperature = false;
  bool warn_overtemperature = false;

  bool has_any_protection = false;

  /* Decoded from 0x35C — BMS Info / Force charge flags */
  bool force_charge_requested = false;     // BMS requests immediate charging (critically low SOC)
  bool force_discharge_requested = false;  // BMS requests immediate discharging

  /* Enhanced logging — enabled via web UI checkbox */
  bool enhanced_logging = false;
};

#endif
