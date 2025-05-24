// SPDX-License-Identifier: AGPL-3.0-or-later
#if EDM_ENABLE

#include "driver.h"
#include "grbl/core_handlers.h"
#include "grbl/grbl.h"
#include "i2c.h"
#include "platform.h"

#include <stdio.h>

#define EDM_ADDR 0x3b
#define EDM_REG_FIRST 0x00  // first register to read
#define EDM_REG_COUNT 3     // how many bytes

static const uint8_t REG_CKP_N_PULSE = 0x10;

#define EDM_MCODE_READ 550

static volatile uint32_t edm_timer_cnt = 0;
static volatile uint32_t edm_poll_cnt = 0;
static volatile bool edm_has_current = false;
static volatile uint64_t last_poll_tick_us;  // hal.get_micros() time

////////////////////////////////////////////////////////////////////////////////
// M-code handlers

static user_mcode_ptrs_t other_mcode_ptrs;

static user_mcode_type_t mcode_check(user_mcode_t m) {
  if (m != EDM_MCODE_READ) {
    return other_mcode_ptrs.check ? other_mcode_ptrs.check(m)
                                  : UserMCode_Unsupported;
  }

  return UserMCode_Normal;
}

static status_code_t mcode_validate(parser_block_t* block) {
  if (block->user_mcode != EDM_MCODE_READ) {
    return other_mcode_ptrs.validate ? other_mcode_ptrs.validate(block)
                                     : Status_Unhandled;
  }

  return Status_OK;
}

static void mcode_execute(uint_fast16_t state, parser_block_t* block) {
  if (block->user_mcode != EDM_MCODE_READ) {
    if (other_mcode_ptrs.execute) {
      other_mcode_ptrs.execute(state, block);
    }
    return;
  }

  bool succ;

  uint8_t buf[1];
  i2c_transfer_t tx;
  tx.address = EDM_ADDR;
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
  ofs += snprintf(resp + ofs, sizeof(resp) - ofs, "[EDM:");
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
  tx.address = EDM_ADDR;
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

void edm_init() {
  i2c_start();

  // Register EDM virtual probe to HAL.
  hal.probe.configure = edm_probe_configure;
  hal.probe.connected_toggle = edm_probe_connected_toggle;
  hal.probe.get_state = edm_probe_get_state;

  // Register PULSER polling as rate-limited "realtime" process.
  // This is better than hal.timers based approach.
  // Doing I2C in timer will cause TMC2209 init to fail, as they're bit-banging
  // serial comm is very sensitive to timing.
  other_realtime = grbl.on_execute_realtime;
  grbl.on_execute_realtime = edm_realtime;

  // Register M-code handler by appending to the call chain.
  memcpy(&other_mcode_ptrs, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
  grbl.user_mcode.check = mcode_check;
  grbl.user_mcode.validate = mcode_validate;
  grbl.user_mcode.execute = mcode_execute;

  // Register report printer.
  other_reports = grbl.on_report_options;
  grbl.on_report_options = edm_report_options;
}

#endif /** EDM_ENABLE */
