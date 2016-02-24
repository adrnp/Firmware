/****************************************************************************
 *
f *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file hunt_params.c
 *
 * Parameters for tweaking the hunt mode.
 *
 * @author Adrien Perkins	<adrienp@stanford.edu>
 */

#include <nuttx/config.h>

#include <systemlib/param/param.h>


/*
 * Hunt mode parameters, accessible via MAVLink
 *
 */

/**
 * Yaw Step.
 *
 * The angle in [deg] through which to step to keep Jager rotating in
 * correct direction.
 *
 * @unit degrees
 * @min 0.0
 * @group Hunt Params
 */
PARAM_DEFINE_FLOAT(HUNT_YAW_STEP, 120.0f);

/**
 * Start Latitude.
 *
 * The latitude in decimal degrees of the starting location for hunting.
 *
 * @unit degrees
 * @min -180.0
 * @group Hunt Params
 */
PARAM_DEFINE_FLOAT(HUNT_STRT_LAT,  37.423703f);

/**
 * Start Longitude.
 *
 * The longitude in decimal degrees of the starting location for hunting.
 *
 * @unit degrees
 * @min -180.0
 * @group Hunt Params
 */
PARAM_DEFINE_FLOAT(HUNT_STRT_LON, -122.176883f);

/**
 * Start Altitude.
 *
 * The altitude in m AMSL of the starting location for hunting.
 *
 * @unit m
 * @min 0.0
 * @group Hunt Params
 */
PARAM_DEFINE_FLOAT(HUNT_STRT_ALT, 40.0f);

/**
 * Start Heading.
 *
 * The heading for the vehicle at the starting point.
 *
 * @unit degree
 * @min 0.0
 * @max 359.9
 * @group Hunt Params
 */
PARAM_DEFINE_FLOAT(HUNT_STRT_HDG, 270.0f);

