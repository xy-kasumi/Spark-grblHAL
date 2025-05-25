// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * Driver for Spark EDM.
 *
 * M503: Energize
 * M505: De-energize
 * G38.2, G38.3: Probe using current sensing. De-energize (same as M505) on
 * contact or not-found completion.
 */
#if EDM_ENABLE

#include "driver.h"
#include "grbl/core_handlers.h"
#include "grbl/grbl.h"
#include "i2c.h"
#include "platform.h"

#include <stdio.h>

#define EDM_MCODE_START_TNEG 503
#define EDM_MCODE_START_TPOS 504
#define EDM_MCODE_STOP 505
#define EDM_MCODE_READ 550

// You can change this if you somehow want to use Aux8 for different purposes.
// If you change this, you also need to updard board configuration header.
#define PIN_FUNCTION_PULSER_GATE Output_Aux8

// See https://github.com/xy-kasumi/Spark/blob/main/docs/user-PULSER.md for
// register map
#define PULSER_ADDR 0x3b

static const uint8_t REG_POLARITY = 0x01;
static const uint8_t REG_PULSE_CURRENT = 0x02;
static const uint8_t REG_TEMPERATURE = 0x03;
static const uint8_t REG_PULSE_DUR = 0x04;
static const uint8_t REG_MAX_DUTY = 0x05;
static const uint8_t REG_CKP_N_PULSE = 0x10;

// Simple status flag for debugging initialization error.
// This will be read by M-code.
// 0: OK
// 255: initial value
// You can use whatever other numbers to indicate failure modes.
static volatile uint8_t edm_init_status = 255;

static bool edm_gate_port_found = false;
static uint8_t edm_gate_port;

static volatile uint32_t edm_timer_cnt = 0;
static volatile uint32_t edm_poll_cnt = 0;
static volatile bool edm_has_current = false;
static volatile uint64_t last_poll_tick_us;  // hal.get_micros() time

////////////////////////////////////////////////////////////////////////////////
// M-code handlers

static on_probe_completed_ptr other_probe_completed;
static user_mcode_ptrs_t other_mcode_ptrs;

static void exec_mcode_read() {
  bool succ;

  uint8_t buf[1];
  i2c_transfer_t tx;
  tx.address = PULSER_ADDR;
  tx.word_addr = 0x03;
  tx.word_addr_bytes = 1;
  tx.count = 1;
  tx.data = buf;
  tx.no_block = false;
  if (i2c_transfer(&tx, true)) {
    succ = true;
  } else {
    succ = false;
  }
  uint8_t temp = buf[0];

  char resp[100];
  size_t ofs = 0;
  ofs += snprintf(resp + ofs, sizeof(resp) - ofs, "[EDM:stat=%d,",
                  edm_init_status);
  if (succ) {
    ofs += snprintf(resp + ofs, sizeof(resp) - ofs, "i2c=ok,temp=%d", temp);
  } else {
    ofs += snprintf(resp + ofs, sizeof(resp) - ofs, "i2c=fail");
  }
  ofs += snprintf(resp + ofs, sizeof(resp) - ofs, ",loops=%ld,polls=%ld",
                  edm_timer_cnt, edm_poll_cnt);

  ofs += snprintf(resp + ofs, sizeof(resp) - ofs, "]" ASCII_EOL);
  hal.stream.write(resp);
}

// blocking I2C register write.
// returns true if write was successful
static bool write_reg(uint8_t reg_addr, uint8_t val) {
  uint8_t buf[6];
  i2c_transfer_t tx;
  tx.address = PULSER_ADDR;
  tx.word_addr = reg_addr;
  tx.word_addr_bytes = 1;
  tx.count = 1;
  tx.data = &val;
  tx.no_block = false;
  return i2c_transfer(&tx, false);
}

inline static void init_gate() {
  GPIO_InitTypeDef init = {
      .Pin = 1 << PULSER_GATE_PIN,
      .Speed = GPIO_SPEED_FREQ_MEDIUM,
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
  };
  HAL_GPIO_Init(PULSER_GATE_PORT, &init);
}

inline static void set_gate(bool on) {
  DIGITAL_OUT(PULSER_GATE_PORT, 1 << PULSER_GATE_PIN, on);
}

// must not be called when edm_init_status != 0
static void exec_mcode_start_tneg() {
  bool all_ok = true;
  all_ok &= write_reg(REG_PULSE_CURRENT, 1);
  all_ok &= write_reg(REG_PULSE_DUR, 25);
  all_ok &= write_reg(REG_MAX_DUTY, 25);
  all_ok &= write_reg(REG_POLARITY, 2);  // 2: T- W+
  if (!all_ok) {
    system_raise_alarm(Alarm_SelftestFailed);
    return;
  }
  set_gate(true);
}

// must not be called when edm_init_status != 0
static void exec_mcode_stop() {
  set_gate(false);

  bool ok = write_reg(REG_POLARITY, 0);  // OFF
  if (!ok) {
    system_raise_alarm(Alarm_SelftestFailed);
    return;
  }
}

static inline bool is_edm_mcode(user_mcode_t m) {
  return (m == EDM_MCODE_READ || m == EDM_MCODE_START_TNEG ||
          m == EDM_MCODE_START_TPOS || m == EDM_MCODE_STOP);
}

static user_mcode_type_t mcode_check(user_mcode_t m) {
  if (!is_edm_mcode(m)) {
    return other_mcode_ptrs.check ? other_mcode_ptrs.check(m)
                                  : UserMCode_Unsupported;
  }

  return UserMCode_Normal;
}

static status_code_t mcode_validate(parser_block_t* block) {
  user_mcode_t code = block->user_mcode;

  if (!is_edm_mcode(code)) {
    return other_mcode_ptrs.validate ? other_mcode_ptrs.validate(block)
                                     : Status_Unhandled;
  }

  if (code == EDM_MCODE_READ) {
    return Status_OK;
  } else {
    if (edm_init_status != 0) {
      return Status_SelfTestFailed;
    }
    return Status_OK;
  }
}

static void mcode_execute(uint_fast16_t state, parser_block_t* block) {
  user_mcode_t code = block->user_mcode;
  if (!is_edm_mcode(code)) {
    if (other_mcode_ptrs.execute) {
      other_mcode_ptrs.execute(state, block);
    }
    return;
  }

  if (code == EDM_MCODE_READ) {
    exec_mcode_read();
  } else if (code == EDM_MCODE_START_TNEG) {
    exec_mcode_start_tneg();
  } else if (code == EDM_MCODE_START_TPOS) {
    // TODO: implement
  } else if (code == EDM_MCODE_STOP) {
    exec_mcode_stop();
  }
}

static void edm_probe_completed() {
  // stop discharge to minimize work damage.
  set_gate(false);

  bool ok = write_reg(REG_POLARITY, 0);  // OFF
  if (!ok) {
    system_raise_alarm(Alarm_SelftestFailed);
    return;
  }

  //
  if (other_probe_completed) {
    other_probe_completed();
  }
}

////////////////////////////////////////////////////////////////////////////////
// EDM Probe

void edm_probe_configure(bool is_probe_away, bool probing) {
  // Maybe nothing is needed here?
  // \param is_probe_away true if probing away from the workpiece, false
  // otherwise. When probing away the signal must be inverted in the
  // probe_get_state_ptr() implementation.
  // \param probing true if probe cycle is active, false otherwise.
}

void edm_probe_connected_toggle() {
  // do nothing; no notion of "toggle"
}

// NOTE: Called from stepping interrupt, must run extremely quickly.
probe_state_t edm_probe_get_state() {
  probe_state_t state = {0};
  state.triggered = edm_has_current;
  state.connected = true;  // always connected
  return state;
}

////////////////////////////////////////////////////////////////////////////////
// Timer

on_execute_realtime_ptr other_realtime;

static void edm_realtime(sys_state_t s) {
  if (other_realtime) {
    other_realtime(s);
  }

  // Limit to 1ms polling rate.
  edm_timer_cnt++;
  uint64_t t_curr = hal.get_micros();
  if (t_curr - last_poll_tick_us < 1000) {
    return;
  }
  last_poll_tick_us = t_curr;

  // Do I2C poll
  uint8_t buf[6];
  i2c_transfer_t tx;
  tx.address = PULSER_ADDR;
  tx.word_addr = REG_CKP_N_PULSE;
  tx.word_addr_bytes = 1;
  tx.count = 6;
  tx.data = buf;
  tx.no_block = false;
  if (!i2c_transfer(&tx, true)) {
    return;
  }

  uint8_t r_pulse = buf[3];
  uint8_t r_short = buf[4];
  uint8_t r_open = buf[5];
  edm_has_current = (r_pulse > 0 || r_short > 0);

  edm_poll_cnt++;
}

////////////////////////////////////////////////////////////////////////////////
// Plugin Reporting

static on_report_options_ptr other_reports;

static void edm_report_options(bool newopt) {
  if (other_reports) {
    other_reports(newopt);
  }

  if (!newopt) {
    report_plugin("EDM", "0.0");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Plugin Init

// returns true if found. Stores result into data (uint8_t*)
static bool port_search_cb(xbar_t* properties, uint8_t port, void* data) {
  uint8_t* result_ptr = (uint8_t*)data;

  if (properties->function == PIN_FUNCTION_PULSER_GATE) {
    *result_ptr = port;
    return true;
  }
  return false;
}

void edm_init() {
  // Register report printer.
  other_reports = grbl.on_report_options;
  grbl.on_report_options = edm_report_options;

  // Register M-code handler by appending to the call chain.
  memcpy(&other_mcode_ptrs, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
  grbl.user_mcode.check = mcode_check;
  grbl.user_mcode.validate = mcode_validate;
  grbl.user_mcode.execute = mcode_execute;

  i2c_start();

  init_gate();
  set_gate(false);  // ensure it's off

  // Register EDM virtual probe to HAL.
  hal.probe.configure = edm_probe_configure;
  hal.probe.connected_toggle = edm_probe_connected_toggle;
  hal.probe.get_state = edm_probe_get_state;

  // Register probe completed handler.
  other_probe_completed = grbl.on_probe_completed;
  grbl.on_probe_completed = edm_probe_completed;

  // Register PULSER polling as rate-limited "realtime" process.
  // This is better than hal.timers based approach.
  // Doing I2C in timer will cause TMC2209 init to fail, as they're bit-banging
  // serial comm is very sensitive to timing.
  other_realtime = grbl.on_execute_realtime;
  grbl.on_execute_realtime = edm_realtime;

  // Mark as OK.
  edm_init_status = 0;
}

#endif /** EDM_ENABLE */
