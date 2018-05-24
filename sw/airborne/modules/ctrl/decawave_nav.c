/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/ctrl/object_tracking.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Control a rotorcraft heading to track an object detected by a camera
 */

#include <stdio.h>
#include "modules/ctrl/decawave_nav.h"

#include "firmwares/fixedwing/nav.h"
#include "subsystems/abi.h"
#include "generated/airframe.h"
#include "firmwares/fixedwing/autopilot_static.h"
//#include "generated/modules.h"

// ABI message binding ID
#ifndef DECAWAVE_NAV_ID
#define DECAWAVE_NAV_ID ABI_BROADCAST
#endif

// Send debug message
#ifndef DECAWAVE_NAV_DEBUG
#define DECAWAVE_NAV_DEBUG FALSE
#endif

#if OBJECT_TRACKING_DEBUG
#include "subsystems/datalink/downlink.h"
#include "pprzlink/messages.h"
#include "mcu_periph/uart.h"
#endif

#ifndef DECAWAVE_NAV_KP
#define DECAWAVE_NAV_KP 1.5
#endif

#ifndef DECAWAVE_NAV_KI
#define DECAWAVE_NAV_KI 0.5
#endif

#ifndef DECAWAVE_NAV_KD
#define DECAWAVE_NAV_KD 1.
#endif

#ifndef DECAWAVE_NAV_MAXSHIFT
#define DECAWAVE_NAV_MAXSHIFT 30
#endif

#ifndef DECAWAVE_NAV_MAXSOMME
#define DECAWAVE_NAV_MAXSOMME 30
#endif

abi_event decawave_ev;
static const float nav_dt = 1.f / NAVIGATION_FREQUENCY;
static float ac_x = 0.;
static float previous_x = 0.;
static float somme = 0.;
static float ac_y,ac_z;
float kp,ki,kd;
float maxshift, maxsomme;

// callback on follow target message
static void get_pos(uint8_t sender_id __attribute__((unused)), uint32_t id __attribute__((unused)), float x , float y, float z, float noise_x __attribute__((unused)), float noise_y __attribute__((unused)), float noise_z __attribute__((unused)))
{
	ac_x = x;
	ac_y = y;
	ac_z = z;
}

void decawave_nav_init(void)
{
  // Bind to camera message
  AbiBindMsgPOSITION_ESTIMATE(DECAWAVE_NAV_ID, &decawave_ev, get_pos);
  kp = DECAWAVE_NAV_KP;
  ki = DECAWAVE_NAV_KI;
  kd = DECAWAVE_NAV_KD;
  maxshift = DECAWAVE_NAV_MAXSHIFT;
  maxsomme = DECAWAVE_NAV_MAXSOMME;
}

void decawave_nav_reset(void)
{
  somme = 0.;
  nav_shift = 0.;
}

void decawave_nav_run(void)
{
  somme+=ac_x*nav_dt;
  if (somme < -maxsomme){nav_shift = -maxsomme;}
  if (somme > maxsomme ){nav_shift = maxsomme;}
  //Shift calculation
  nav_shift = -(kp*ac_x + ki*somme + kd*(ac_x - previous_x)/nav_dt);
  previous_x = ac_x;
  if (nav_shift < -maxshift ){nav_shift = -maxshift;}
  if (nav_shift > maxshift ){nav_shift = maxshift;}
}

