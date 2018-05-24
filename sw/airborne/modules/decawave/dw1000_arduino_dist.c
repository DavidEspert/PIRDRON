/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/decawave/dw1000_arduino.c"
 * @author Gautier Hattenberger
 * Driver to get ranging data from Decawave DW1000 modules connected to Arduino
 */

#include "modules/decawave/dw1000_arduino.h"

#include "std.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/abi.h"
#include "modules/decawave/trilateration.h"
#include "subsystems/gps.h"
#include "state.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Number of anchors
 *
 * using standard trilateration algorithm, only 3 anchors are required/supported
 * at the moment.
 * More advanced multilateration algorithms might allow more anchors in the future
 */
#ifndef DW1000_NB_ANCHORS
#define DW1000_NB_ANCHORS 3
#endif

/** default offset, applied to final result not to individual distances */
#ifndef DW1000_OFFSET
#define DW1000_OFFSET { 0.f, 0.f, 0.f }
#endif

/** default scale factor, applied to final result not to individual distances */
#ifndef DW1000_SCALE
#define DW1000_SCALE { 1.f, 1.f, 1.f }
#endif

/** default initial heading correction between anchors frame and global frame */
#ifndef DW1000_INITIAL_HEADING
#define DW1000_INITIAL_HEADING 0.f
#endif

/** default timeout (in ms) */
#ifndef DW1000_TIMEOUT
#define DW1000_TIMEOUT 500
#endif

/** frame sync byte */
#define DW_STX 0xFE

/** Parsing states */
#define DW_WAIT_STX 0
#define DW_GET_DATA 1
#define DW_GET_CK 2
#define DW_NB_DATA 6

/** DW1000 Noise */
#ifndef NOISE_X
#define NOISE_X 0.1
#endif

#ifndef NOISE_Y
#define NOISE_Y 0.1
#endif

#ifndef NOISE_Z
#define NOISE_Z 0.1
#endif

/** Landing direction */

float landing_direction;

/** DW1000 positionning system structure */
struct DW1000 {
  uint8_t buf[DW_NB_DATA];    ///< incoming data buffer
  uint8_t idx;                ///< buffer index
  uint8_t ck;                 ///< checksum
  uint8_t state;              ///< parser state
  float initial_heading;      ///< initial heading correction
  struct Anchor anchors[DW1000_NB_ANCHORS];   ///<anchors data
  struct EnuCoor_f pos;       ///< local pos in anchors frame
  struct EnuCoor_f raw_pos;   ///< unscaled local pos in anchors frame
  struct GpsState gps_dw1000; ///< "fake" gps structure
  struct LtpDef_i ltp_def;    ///< ltp reference
  bool updated;               ///< new anchor data available
};

static struct DW1000 dw1000;


/** Utility function to get float from buffer */
static inline float float_from_buf(uint8_t* b) {
  float f;
  memcpy((uint8_t*)(&f), b, sizeof(float));
  return f;
}

/** Utility function to get uint16_t from buffer */
static inline uint16_t uint16_from_buf(uint8_t* b) {
  uint16_t u16;
  memcpy ((uint8_t*)(&u16), b, sizeof(uint16_t));
  return u16;
}

/** Utility function to fill anchor from buffer */
static void fill_anchor(struct DW1000 *dw) {
  for (int i = 0; i < DW1000_NB_ANCHORS; i++) {
    uint16_t id = uint16_from_buf(dw->buf);
    if (dw->anchors[i].id == id) {
      dw->anchors[i].distance = float_from_buf(dw->buf + 2);
      dw->anchors[i].time = get_sys_time_float();
      dw->updated = true;
      break;
    }
  }
}

/** Data parsing function */
static void dw1000_arduino_parse(struct DW1000 *dw, uint8_t c)
{
  switch (dw->state) {

    case DW_WAIT_STX:
      /* Waiting Synchro */
      if (c == DW_STX) {
        dw->idx = 0;
        dw->ck = 0;
        dw->state = DW_GET_DATA;
      }
      break;

    case DW_GET_DATA:
      /* Read Bytes */
      dw->buf[dw->idx++] = c;
      dw->ck += c;
      if (dw->idx == DW_NB_DATA) {
        dw->state = DW_GET_CK;
      }
      break;

    case DW_GET_CK:
      /* Checksum */
      if (dw->ck == c) {
        fill_anchor(dw);
      }
      dw->state = DW_WAIT_STX;
      break;
  }
}


static void dw1000_send_pos(struct DW1000 *dw)
{
  uint32_t now_ts = get_sys_time_usec();
  //Send POSITION_ESTIMATE type message
  AbiSendMsgPOSITION_ESTIMATE(GPS_DW1000_ID, now_ts, dw->pos.x, dw->pos.y, dw->pos.z,NOISE_X,NOISE_Y,NOISE_Z);
}
  

void dw1000_reset_heading_ref(void)
{
  // store current heading as ref and stop periodic call
  dw1000.initial_heading = stateGetNedToBodyEulers_f()->psi;
  dw1000_arduino_dist_dw1000_reset_heading_ref_status = MODULES_STOP;
}

/// init arrays from airframe file
static const uint16_t ids[] = DW1000_ANCHORS_IDS;
static const float pos_x[] = DW1000_ANCHORS_POS_X;
static const float pos_y[] = DW1000_ANCHORS_POS_Y;
static const float pos_z[] = DW1000_ANCHORS_POS_Z;
static const float offset[] = DW1000_OFFSET;
static const float scale[] = DW1000_SCALE;

static void scale_distances(struct Anchor *anchors)
{
  anchors[0].distance = scale[0] * (anchors[0].distance - offset[0]);
  anchors[1].distance = scale[1] * (anchors[1].distance - offset[1]);
  anchors[2].distance = scale[2] * (anchors[2].distance - offset[2]);
}

void dw1000_arduino_compute_ld(){
  float x_td = WaypointX(WP_TD);
  float y_td = WaypointY(WP_TD);
  float x_af = WaypointX(WP_AF);
  float y_af = WaypointY(WP_AF);
  if (x_td<=x_af){landing_direction = atan((y_td-y_af)/(x_td-x_af)) + 3.1415/2;}
  if (x_td>x_af){landing_direction = atan((y_td-y_af)/(x_td-x_af)) - 3.1415/2;}
}

void dw1000_arduino_init()
{
  // init DW1000 structure
  dw1000.idx = 0;
  dw1000.ck = 0;
  dw1000.state = DW_WAIT_STX;
  dw1000.initial_heading = DW1000_INITIAL_HEADING;
  dw1000.pos.x = 0.f;
  dw1000.pos.y = 0.f;
  dw1000.pos.z = 0.f;
  dw1000.updated = false;
  for (int i = 0; i < DW1000_NB_ANCHORS; i++) {
    dw1000.anchors[i].distance = 0.f;
    dw1000.anchors[i].time = 0.f;
    dw1000.anchors[i].id = ids[i];
    dw1000.anchors[i].pos.x = pos_x[i];
    dw1000.anchors[i].pos.y = pos_y[i];
    dw1000.anchors[i].pos.z = pos_z[i];
  }

  // gps structure init
  dw1000.gps_dw1000.fix = GPS_FIX_NONE;
  dw1000.gps_dw1000.pdop = 0;
  dw1000.gps_dw1000.sacc = 0;
  dw1000.gps_dw1000.pacc = 0;
  dw1000.gps_dw1000.cacc = 0;
  dw1000.gps_dw1000.comp_id = GPS_DW1000_ID;

  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;
  ltp_def_from_lla_i(&dw1000.ltp_def, &llh_nav0);

  // init trilateration algorithm
  trilateration_init(dw1000.anchors);
  
  //init Landing direction
  dw1000_arduino_compute_ld(); 
}

void dw1000_arduino_periodic()
{
  // Check for timeout
  dw1000_arduino_compute_ld();
  gps_periodic_check(&(dw1000.gps_dw1000));
}

void dw1000_arduino_report()
{
  float buf[9];
  buf[0] = dw1000.anchors[0].distance;
  buf[1] = dw1000.anchors[1].distance;
  buf[2] = dw1000.anchors[2].distance;
  buf[3] = dw1000.raw_pos.x;
  buf[4] = dw1000.raw_pos.y;
  buf[5] = dw1000.raw_pos.z;
  buf[6] = dw1000.pos.x;
  buf[7] = dw1000.pos.y;
  buf[8] = dw1000.pos.z;
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 9, buf);
}

/** check timeout for each anchor
 * @return true if one has reach timeout
 */
static bool check_anchor_timeout(struct DW1000 *dw)
{
  const float now = get_sys_time_float();
  const float timeout = (float)DW1000_TIMEOUT / 1000.;
  for (int i = 0; i < DW1000_NB_ANCHORS; i++) {
    if (now - dw->anchors[i].time > timeout) {
      return true;
    }
  }
  return false;
}

void dw1000_arduino_event()
{
  // Look for data on serial link and send to parser
  while (uart_char_available(&DW1000_ARDUINO_DEV)) {
    uint8_t ch = uart_getch(&DW1000_ARDUINO_DEV);
    dw1000_arduino_parse(&dw1000, ch);
    // process if new data
    if (dw1000.updated) {
	// apply scale and neutral corrections
        scale_distances(dw1000.anchors);
      // if no timeout on anchors, run trilateration algorithm
      if (check_anchor_timeout(&dw1000) == false &&
          trilateration_compute(dw1000.anchors, &dw1000.pos) == 0) {
	//send position
	dw1000_send_pos(&dw1000);
      }
      dw1000.updated = false;
    }
  }
}


