// SPDX-License-Identifier: AGPL-3.0-or-later
#include "driver.h"  // hal.*, driver macros

#if EDM_ENABLE

#include "grbl/core_handlers.h"
#include "grbl/grbl.h"
#include "i2c.h"
#include "platform.h"

#include <stdio.h>

#define EDM_ADDR 0x3b       // <-- change to suit
#define EDM_REG_FIRST 0x00  // first register to read
#define EDM_REG_COUNT 3     // how many bytes

#define EDM_MCODE_READ 550

static volatile uint8_t edm_timer_status = 255;  // 0: success, 255: unknown
static volatile uint32_t edm_timer_cnt = 0;

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
  ofs += snprintf(resp + ofs, sizeof(resp) - ofs, ",status=%d,loops=%ld",
                  edm_timer_status, edm_timer_cnt);

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
  state.triggered = false;  // TODO: implement
  state.connected = true;   // always connected
  return state;
}

////////////////////////////////////////////////////////////////////////////////
// Timer

static void edm_poll_isr(void* context) {
  //   uint8_t buf[EDM_REG_COUNT];
  // if (i2c_read(EDM_I2C, EDM_ADDR, EDM_REG_FIRST, buf, EDM_REG_COUNT))
  // memcpy(edm_snapshot, buf, EDM_REG_COUNT);  // update the live copy
  //}
  edm_timer_cnt++;
}

void edm_start_timer() {
  // 1 tick = 1us = 1000 ns
  hal_timer_t timer = hal.timer.claim((timer_cap_t){.periodic = 1}, 1000);
  if (!timer) {
    edm_timer_status = 1;
    return;
  }

  timer_cfg_t cfg = {
      .single_shot = false,
      .timeout_callback = edm_poll_isr,
  };
  if (!hal.timer.configure(timer, &cfg)) {
    edm_timer_status = 2;
    return;
  }
  // 1ms = 1000 tick
  if (!hal.timer.start(timer, 1000)) {
    edm_timer_status = 3;
    return;
  }
  // succesfully started
  edm_timer_status = 0;
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

  // Register timer (can fail).
  edm_start_timer();

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
