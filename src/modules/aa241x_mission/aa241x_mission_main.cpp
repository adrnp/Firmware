/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file aa241x_mission_main.cpp
 *
 * Mission logic for Stanford's Spring 2015 AA241x class.  This handles
 * all safety checks, mission checks, runs the mission (fire propagation),
 * and handles all picture and water dropping requests.
 *
 * @author Adrien Perkins	<adrienp@stanford.edu>
 *
 */
#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <algorithm>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <uORB/uORB.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <mathlib/mathlib.h>
//#include <mavlink/mavlink_log.h>
#include <drivers/drv_tone_alarm.h>

#include "LakeFire.h"
#include "fires.h"


/**
 * aa241x_mission app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int aa241x_mission_main(int argc, char *argv[]);


namespace aa241x_mission
{

LakeFire	*g_aa241x_mission = nullptr;
}

LakeFire::LakeFire() :
	_task_should_exit(false),
	_task_running(false),
	_control_task(-1),
	_mavlink_fd(-1),
	_buzzer(-1),
	_vcontrol_mode_sub(-1),
	_global_pos_sub(-1),
	_local_pos_sub(-1),
	_vehicle_status_sub(-1),
	_params_sub(-1),
	_pic_request_sub(-1),
	_water_drop_request_sub(-1),
	_local_data_sub(-1),
	_battery_status_sub(-1),
	_mission_status_pub(nullptr),
	_new_fire_pub(nullptr),
	_fire_prop_pub(nullptr),
	_pic_result_pub(nullptr),
	_water_drop_result_pub(nullptr),
	_cgrid_pub(nullptr),
	_mission_start_time(-1),
	_last_propagation_time(-1),
	_mission_start_battery(0),
	_in_mission(false),
	_can_start(true),
	_early_termination(false),
	_mission_failed(false),
	_score(0.0f),
	_unattended_count(0.0f),
	_cross_min(false),
	_last_picture(0),
	_new_fire_count(0),
	_wind_direction(WIND_OTHER),
	_grid{{0}},
	_grid_mask{{false}}
{
	_vcontrol_mode = {};
	_global_pos = {};
	_local_pos = {};
	_vehicle_status = {};
	_pic_request = {};
	_water_drop_request = {};
	_batt_stat = {};

	_parameter_handles.min_alt = param_find("AAMIS_ALT_MIN");
	_parameter_handles.max_alt = param_find("AAMIS_ALT_MAX");
	_parameter_handles.auto_alt = param_find("AAMIS_ALT_AUTO");
	_parameter_handles.cell_width = param_find("AAMIS_CELL_W");
	_parameter_handles.duration = param_find("AAMIS_DURATION");
	_parameter_handles.max_radius = param_find("AAMIS_RAD_MAX");
	_parameter_handles.timestep = param_find("AAMIS_TSTEP");
	_parameter_handles.std = param_find("AAMIS_STD");
	_parameter_handles.t_pic = param_find("AAMIS_TPIC");
	_parameter_handles.min_fov = param_find("AAMIS_FOV_MIN");
	_parameter_handles.max_fov = param_find("AAMIS_FOV_MAX");
	_parameter_handles.index = param_find("AA_MIS_INDEX");
	_parameter_handles.water_weight = param_find("AA_WATER_WGHT");
	_parameter_handles.weight_per_drop = param_find("AAMIS_WGHT_DRP");
	_parameter_handles.ctr_lat = param_find("AAMIS_CTR_LAT");
	_parameter_handles.ctr_lon = param_find("AAMIS_CTR_LON");
	_parameter_handles.ctr_alt = param_find("AAMIS_CTR_ALT");
	_parameter_handles.max_discharge = param_find("AAMIS_BATT_MAX");
	_parameter_handles.team_num = param_find("AA_TEAM");

	parameters_update();

	build_grid_mask();

	_water_drops_remaining = int(_parameters.water_weight/_parameters.weight_per_drop);
	_propagations_remaining = int(_parameters.duration*60.0f/_parameters.timestep);

}

LakeFire::~LakeFire() {
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	aa241x_mission::g_aa241x_mission = nullptr;
}


aa241x_picture_result_s
LakeFire::take_picture()
{
	aa241x_picture_result_s pic_result = {};

	pic_result.center_n = _pic_request.pos_N; // _local_pos.x;
	pic_result.center_e = _pic_request.pos_E; // _local_pos.y;
	pic_result.center_d = _pic_request.pos_D; // _local_pos.z;

	hrt_abstime curr_time = _pic_request.time_us; // hrt_absolute_time();
	pic_result.time_us = curr_time;
	float time_diff = (curr_time - _last_picture)/1000000.0f;

	/* check mission logic */
	if (!_in_mission || time_diff <= _parameters.t_pic) {
		/* do not take a picture */
		pic_result.pic_taken = false;
		return pic_result;
	}

	/* check position */
	float d = pic_result.center_d;
	float n = pic_result.center_n;
	float e = pic_result.center_e;
	if (-d < _parameters.min_alt || -d > _parameters.max_alt || (n*n + e*e) >= _parameters.max_radius*_parameters.max_radius) {
		pic_result.pic_taken = false;
		return pic_result;
	}

	pic_result.pic_taken = true;

	float pic_d = get_fov_d(-pic_result.center_d);
	pic_result.pic_d = pic_d;

	/* populate the i, j and state vectors */
	get_fire_info(&pic_result);

	/* set the last picture time to this time */
	_last_picture = curr_time;

	return pic_result;
}

aa241x_water_drop_result_s
LakeFire::drop_water()
{
	aa241x_water_drop_result_s water_drop;

	water_drop.time_us = _water_drop_request.time_us; // hrt_absolute_time();
	float n = _water_drop_request.pos_N; // _local_pos.x;
	float e = _water_drop_request.pos_E; // _local_pos.y;
	float d = _water_drop_request.pos_D; // _local_pos.z;

	/* mission logic checks */
	if (!_in_mission || _water_drops_remaining <= 0) {
		water_drop.success = false;
		return water_drop;
	}

	/* position checks */
	if (-d < _parameters.min_alt || -d > _parameters.max_alt || (n*n + e*e) >= _parameters.max_radius*_parameters.max_radius) {
		water_drop.success = false;
		return water_drop;
	}

	int i = n2i(n);
	int j = e2j(e);

	// TODO: need to check to make sure these are a valid (i,j) pair
	/* water this grid cell */
	_grid[i][j] = WATER;

	water_drop.success = true;
	water_drop.i = i;
	water_drop.j = j;

	/* reduce the number of water drops remaining */
	_water_drops_remaining--;

	/* publish the info */
	publish_water_drop(water_drop);

	return water_drop;
}

void
LakeFire::build_grid_mask()
{
	float hw = _parameters.cell_width/2.0f;
	math::Vector<2> center;

	math::Vector<2> left = math::Vector<2>(-hw, -hw);
	math::Vector<2> right = math::Vector<2>(hw, hw);
	math::Vector<2> top = math::Vector<2>(-hw, hw);
	math::Vector<2> bottom = math::Vector<2>(hw, -hw);

	math::Vector<2> sides[4] = {left, right, top, bottom};

	float r2 = _parameters.max_radius * _parameters.max_radius;
	bool valid = true;

	for (int i = 0; i < GRID_WIDTH; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {
			valid = true;

			// get center coord
			center = ij2ne((float) i, (float) j);

			// if center isn't valid, definite not a valid cell
			if (center.length_squared() >= r2) {
				continue;
			}

			// check each of the sides
			for (int k = 0; k < 4; k++) {
				if ((center + sides[k]).length_squared() > r2) {
					valid = false;
					break;
				}
			}

			// set mask to true if an inbound cell
			if (valid) {
				_grid_mask[i][j] = true;
			}
		}
	}
}

float
LakeFire::get_fov_d(const float &alt)
{
	return _parameters.min_fov + (alt - _parameters.min_alt)*(_parameters.max_fov - _parameters.min_fov)/(_parameters.max_alt - _parameters.min_alt);
}

int
LakeFire::n2i(const float &n)
{
	int i = GRID_CENTER - (int) roundf(n/_parameters.cell_width);

	if (i < 0) i = 0;
	if (i >= GRID_WIDTH) i = GRID_WIDTH - 1;

	return i;
}

int
LakeFire::e2j(const float &e)
{
	int j = GRID_CENTER + (int) roundf(e/_parameters.cell_width);

	if (j < 0) j = 0;
	if (j >= GRID_WIDTH) j = GRID_WIDTH - 1;

	return j;
}

math::Vector<2>
LakeFire::ij2ne(const float &i, const float &j)
{
	math::Vector<2> ne;

	ne(0) = (GRID_CENTER - i)*_parameters.cell_width;
	ne(1) = (j - GRID_CENTER)*_parameters.cell_width;

	return ne;

}

void
LakeFire::get_fire_info(aa241x_picture_result_s *pic_result)
{

	int i_view[9] = {0};
	int j_view[9] = {0};
	int state[9] = {0};

	/* explicit clean up (should not be necessary) */
	pic_result->num_cells = 0;

	float pic_r = pic_result->pic_d/2.0f;
	float center_n = pic_result->center_n;
	float center_e = pic_result->center_e;

	int i_min = n2i(center_n + pic_r);
	int i_max = n2i(center_n - pic_r);

	int j_min = e2j(center_e - pic_r);
	int j_max = e2j(center_e + pic_r);

	float d2 = (1.0f/4.0f) * (pic_result->pic_d) * (pic_result->pic_d);

	math::Vector<2> center;
	center(0) = center_n;
	center(1) = center_e;

	int loc = 0;
	for (int i = i_min; i <= i_max; i++) {
		for (int j = j_min; j <= j_max; j++) {

			/*check to ensure the grid cell is in bounds */
			if (!_grid_mask[i][j]) continue;

			/* check to ensure center of cell is within fov */
			if ((ij2ne(i,j) - center).length_squared() <= d2) {
				i_view[loc] = i;
				j_view[loc] = j;
				state[loc] = _grid[i][j];
				loc++;
			}

		}
	}

	pic_result->num_cells = loc;
	memcpy(&pic_result->i, &i_view, sizeof(i_view));
	memcpy(&pic_result->j, &j_view, sizeof(j_view));
	memcpy(&pic_result->state, &state, sizeof(state));

	return;
}

int
LakeFire::parameters_update()
{
	param_get(_parameter_handles.min_alt, &(_parameters.min_alt));
	param_get(_parameter_handles.max_alt, &(_parameters.max_alt));
	param_get(_parameter_handles.min_fov, &(_parameters.min_fov));
	param_get(_parameter_handles.max_fov, &(_parameters.max_fov));
	param_get(_parameter_handles.auto_alt, &(_parameters.auto_alt));
	param_get(_parameter_handles.cell_width, &(_parameters.cell_width));
	param_get(_parameter_handles.duration, &(_parameters.duration));
	param_get(_parameter_handles.max_radius, &(_parameters.max_radius));
	param_get(_parameter_handles.timestep, &(_parameters.timestep));
	param_get(_parameter_handles.std, &(_parameters.std));
	param_get(_parameter_handles.t_pic, &(_parameters.t_pic));
	param_get(_parameter_handles.index, &(_parameters.index));
	param_get(_parameter_handles.water_weight, &(_parameters.water_weight));
	param_get(_parameter_handles.weight_per_drop, &(_parameters.weight_per_drop));
	param_get(_parameter_handles.ctr_lat, &(_parameters.ctr_lat));
	param_get(_parameter_handles.ctr_lon, &(_parameters.ctr_lon));
	param_get(_parameter_handles.ctr_alt, &(_parameters.ctr_alt));
	param_get(_parameter_handles.max_discharge, &(_parameters.max_discharge));
	param_get(_parameter_handles.team_num, &(_parameters.team_num));

	return OK;
}


void
LakeFire::vehicle_control_mode_update()
{
	/* Check if vehicle control mode has changed */
	bool vcontrol_mode_updated;
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
LakeFire::global_pos_update()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
LakeFire::local_pos_update()
{
	/* check if there is a new local position */
	bool local_pos_updated;
	orb_check(_local_pos_sub, &local_pos_updated);

	if (local_pos_updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}
}

void
LakeFire::vehicle_status_update()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
	}
}

void
LakeFire::local_data_update()
{
	/* check if there is new status information */
	bool local_data_updated;
	orb_check(_local_data_sub, &local_data_updated);

	if (local_data_updated) {
		orb_copy(ORB_ID(aa241x_local_data), _local_data_sub, &_local_data);
	}
}

void
LakeFire::battery_status_update()
{
	/* check if there is new status information */
	bool battery_status_updated;
	orb_check(_battery_status_sub, &battery_status_updated);

	if (battery_status_updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_batt_stat);
	}
}


void
LakeFire::handle_picture_request()
{
	/* copy the picture request */
	orb_copy(ORB_ID(aa241x_picture_request), _pic_request_sub, &_pic_request);

	/* do the taking of the picture */
	aa241x_picture_result_s pic_result = take_picture();

	/* only publish the result if the picture was successful,
	 * this ensures that there is no erasing of a picture result before
	 * it can be used in the case of someone spaming the take picture function */
	// TODO: maybe always publish the result...
	if (pic_result.pic_taken) {
		publish_picture_result(pic_result);
	}
}

void
LakeFire::handle_water_drop_request()
{
	/* copy the water drop request */
	orb_copy(ORB_ID(aa241x_water_drop_request), _water_drop_request_sub, &_water_drop_request);

	/* do the dropping of water */
	aa241x_water_drop_result_s water_drop_res = drop_water();

	/* only publish the result if the was drop was successful,
	 * this ensures that there is no erasing of a water drop result before
	 * it can be used */
	// TODO: maybe always publish the result...
	if (water_drop_res.success) {
		publish_water_drop(water_drop_res);
	}
}

void
LakeFire::publish_mission_status()
{
	aa241x_mission_status_s mis_stat;
	mis_stat.can_start = _can_start;
	mis_stat.in_mission = _in_mission;
	mis_stat.score = _score;
	mis_stat.wind_direction = (int) _wind_direction;
	mis_stat.mission_index = _parameters.index;

	if (_in_mission) {
		mis_stat.mission_time = (hrt_absolute_time() - _mission_start_time)/(1000000.0f*60.0f);
		mis_stat.battery_used = _batt_stat.discharged_mah - _mission_start_battery;
	} else {
		/*
		// don't necessarily need to set to 0?
		mis_stat.mission_time = 0.0f;
		mis_stat.battery_used = 0.0f;
		*/
	}

	/* publish the mission status */
	if (_mission_status_pub != nullptr) {
		orb_publish(ORB_ID(aa241x_mission_status), _mission_status_pub, &mis_stat);
	} else {
		_mission_status_pub = orb_advertise(ORB_ID(aa241x_mission_status), &mis_stat);
	}
}

void
LakeFire::publish_condensed_grid()
{
	/*// IF 16 x 16 GRID
	uint16_t fire_row[16] = {0};
	uint16_t water_row[16] = {0};

	for (int i = 0; i < GRID_WIDTH; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {
			if (_grid[i][j] == ON_FIRE) {
				fire_row[i] |= 1 << (16 - j);
				water_row[i] |= 0 << (16 - j);
			} else if (_grid[i][j] == WATER) {
				water_row[i] |= 1 << (16 - j);
				fire_row[i] |= 0 << (16 - j);
			} else {
				fire_row[i] |= 0 << (16 - j);
				water_row[i] |= 0 << (16 - j);
			}
		}
	} */

	aa241x_cgrid_s cgrid;

	// general grid
	cgrid.time_us = hrt_absolute_time();
	memset(&cgrid.cells, 0, sizeof(cgrid.cells));

	int row_id = 0;
	int shift_id = 30;

	for (int i = 0; i < GRID_WIDTH; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {
			if (_grid_mask[i][j]) {
				if (shift_id < 0) {
					row_id++;
					shift_id = 30;
				}

				if (_grid[i][j] == ON_FIRE) {
					cgrid.cells[row_id] |= 1 << shift_id;
				} else if (_grid[i][j] == WATER) {
					cgrid.cells[row_id] |= 2 << shift_id;
				} else {
					cgrid.cells[row_id] |= 0 << shift_id;
				}

				shift_id -= 2;
			}
		}
	}

	/* publish the mission status */
	if (_cgrid_pub != nullptr) {
		orb_publish(ORB_ID(aa241x_cgrid), _cgrid_pub, &cgrid);
	} else {
		_cgrid_pub = orb_advertise(ORB_ID(aa241x_cgrid), &cgrid);
	}
}

void
LakeFire::publish_new_fire(const std::vector<int> &i_new, const std::vector<int> &j_new)
{
	/* 
	// THIS ALL DOESN'T WORK ANYWAY
	std::vector<int> i_fire = i_new;
	std::vector<int> j_fire = j_new;

	aa241x_new_fire_s new_fire;
	new_fire.mission_time = _last_propagation_time;
	new_fire.num_new = i_fire.size();
	new_fire.i_new = &i_fire[0];
	new_fire.j_new = &j_fire[0];

	// publish the new fire cells 
	if (_new_fire_pub != nullptr) {
		orb_publish(ORB_ID(aa241x_new_fire), _new_fire_pub, &new_fire);
	} else {
		_new_fire_pub = orb_advertise(ORB_ID(aa241x_new_fire), &new_fire);
	}
	*/
}

void
LakeFire::publish_fire_prop()
{
	aa241x_fire_prop_s fire_prop;
	fire_prop.time_us = _last_propagation_time;
	fire_prop.num_new = _new_fire_count;
	fire_prop.props_remaining = _propagations_remaining;

	/* publish the info on this fire prop step */
	if (_fire_prop_pub != nullptr) {
		orb_publish(ORB_ID(aa241x_fire_prop), _fire_prop_pub, &fire_prop);
	} else {
		_fire_prop_pub = orb_advertise(ORB_ID(aa241x_fire_prop), &fire_prop);
	}
}

void
LakeFire::publish_picture_result(const aa241x_picture_result_s &pic_result)
{
	/* publish the picture result */
	if (_pic_result_pub != nullptr) {
		orb_publish(ORB_ID(aa241x_picture_result), _pic_result_pub, &pic_result);
	} else {
		_pic_result_pub = orb_advertise(ORB_ID(aa241x_picture_result), &pic_result);
	}
}

void
LakeFire::publish_water_drop(const aa241x_water_drop_result_s &water_drop_result)
{
	/* publish the picture result */
	if (_water_drop_result_pub != nullptr) {
		orb_publish(ORB_ID(aa241x_water_drop_result), _water_drop_result_pub, &water_drop_result);
	} else {
		_water_drop_result_pub = orb_advertise(ORB_ID(aa241x_water_drop_result), &water_drop_result);
	}
}


float
LakeFire::generate_normal_random(const float &mean)
{
	// TODO: it is never actually using the spare, would need to make params global
	bool have_spare = false;

	/* get uniform random values on interval (0,1) */
	float rand1 = (float) rand() / ((float) MAX_RAND);
	float rand2 = (float) rand() / ((float) MAX_RAND);

	float R;
	float theta;

	/* generate 2 random numbers at a time, but only use 1, check for leftover from previous request */
	if (have_spare) {
		have_spare = false;
		return (_parameters.std * sqrtf(R) * sinf(theta)) + mean;
	}

	have_spare = true;
	R = -2*logf(rand1);
	theta = rand2 * 2.0f * (float) M_PI;

	/* return Box-Muller transform for normal random */
	return (_parameters.std * sqrtf(R) * cosf(theta)) + mean;
}


void
LakeFire::get_prop_coords(int *i_prop, int *j_prop, const int &prop_dir)
{

	/* adjust new fire cell coords based on propagation direction */
	switch (WIND_DIRECTION(prop_dir)) {
	case NORTH:
		(*i_prop)--;
		break;
	case NORTH_EAST:
		(*i_prop)--;
		(*j_prop)++;
		break;
	case EAST:
		(*j_prop)++;
		break;
	case SOUTH_EAST:
		(*i_prop)++;
		(*j_prop)++;
		break;
	case SOUTH:
		(*i_prop)++;
		break;
	case SOUTH_WEST:
		(*i_prop)++;
		(*j_prop)--;
		break;
	case WEST:
		(*j_prop)--;
		break;
	case NORTH_WEST:
		(*i_prop)--;
		(*j_prop)--;
		break;
	default:
		break;
	}
}

void
LakeFire::calculate_unattended_score()
{
	int8_t temp_grid[GRID_WIDTH][GRID_WIDTH] = {{0}};

	// initialize the random seed
	srand(seed_start[_parameters.index]);

	// load up of the initial fire cells to the temp grid
	int i_s;
	int j_s;
	for (int i = 0; i < NUM_STARTS; i++) {
		i_s = i_start[_parameters.index][i];
		j_s = j_start[_parameters.index][i];

		/* check to see if reached the end of valid locations */
		if (i_s < 0 || j_s < 0) {
			break;
		}

		/* set this starting grid on fire */
		temp_grid[i_s][j_s] = ON_FIRE;
	}

	int temp_props_remaining = int(_parameters.duration*60.0f/_parameters.timestep);

	while (temp_props_remaining > 0) {
		propagate_temp_fire(temp_grid);
		temp_props_remaining--;
	}

	// print out the grid
	/*
	printf("\n");
	printf("Printing temp grid: \n");
	for (int i = 0; i < GRID_WIDTH; i++) {
		for (int j = 0;j < GRID_WIDTH; j++) {
			printf("%d ", temp_grid[i][j]);
		}
		printf("\n");
	}
	printf("\n");
	*/

	// count the number of cells on fire
	int count = 0;
	int8_t cell_val;

	for (int i = 0; i < GRID_WIDTH; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {

			/* check for fire in cell */
			cell_val = temp_grid[i][j];
			if (cell_val == ON_FIRE) {
				count++;
			}
		}
	}

	_unattended_count = count;
}


void
LakeFire::propagate_temp_fire(int8_t temp_grid[GRID_WIDTH][GRID_WIDTH])
{
	float prop_dir;
	int8_t cell_val;
	int i_prop;
	int j_prop;

	std::vector<int> i_new;
	std::vector<int> j_new;

	int count = 0;

	for (int i = 0; i < GRID_WIDTH; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {

			prop_dir = roundf(generate_normal_random(_wind_direction));

			/* make sure this cell isn't a new fire cell */
			if (std::find(i_new.begin(), i_new.end(), i) != i_new.end() && std::find(j_new.begin(), j_new.end(), j) != j_new.end()) {
				continue;
			}

			/* check for fire in cell, continue if no fire in cell */
			cell_val = temp_grid[i][j];
			if (cell_val != ON_FIRE) continue;
			count++;

			/* wrap the propagation direction to be within (0,8) */
			if (prop_dir < 0) prop_dir += 8;
			if (prop_dir > 7) prop_dir -= 8;

			i_prop = i;
			j_prop = j;

			get_prop_coords(&i_prop, &j_prop, (int) prop_dir);

			/* check to make sure new fire cell is a valid location */
			if (i_prop >= GRID_WIDTH || i_prop < 0 || j_prop >= GRID_WIDTH || j_prop < 0) continue;

			/* check that the new cell is in bounds */
			if (!_grid_mask[i_prop][j_prop]) continue;

			/* check for new fire cell value */
			cell_val = temp_grid[i_prop][j_prop];
			if (cell_val == OPEN_LAND) {
				/* add fire to this cell */
				temp_grid[i_prop][j_prop] = ON_FIRE;
				i_new.push_back(i_prop);
				j_new.push_back(j_prop);
			}
		}
	}

	/* clear vectors after having published the info */
	i_new.clear();
	j_new.clear();
}


void
LakeFire::initialize_mission()
{

	if (_parameters.index >= NUM_FIRES) {
		// TODO: throw a system warning
		_can_start = false;
		mavlink_log_info(_mavlink_fd, "#audio: AA241x invalid mission index");
		return;
	}

	// send message that mission has started
	mavlink_log_info(_mavlink_fd, "#audio: AA241x mission started");

	// trigger the buzzer audio for mission start
	/*
	switch(_parameters.team_num){
	case 1:

		break;
	case 2:

		break;
	case 3:
		ioctl(_buzzer, TONE_SET_ALARM, TONE_TRAINER_BATTLE_TUNE);
		break;
	case 4:

		break;
	default:

		break;
	} */

	_in_mission = true;
	_can_start = false;	// don't allow restarting of a mission
	_mission_start_time = hrt_absolute_time();
	_last_propagation_time = hrt_absolute_time();
	_mission_start_battery = _batt_stat.discharged_mah;

	// TODO: need to ensure that the wind direction is valid
	_wind_direction = WIND_DIRECTION(fire_wind_dir[_parameters.index]);

	int i_s;
	int j_s;
	for (int i = 0; i < NUM_STARTS; i++) {
		i_s = i_start[_parameters.index][i];
		j_s = j_start[_parameters.index][i];

		/* check to see if reached the end of valid locations */
		if (i_s < 0 || j_s < 0) {
			break;
		}

		/* set this starting grid on fire */
		_grid[i_s][j_s] = ON_FIRE;
	}

	// calculate the unattended score
	calculate_unattended_score();

	// reinitialize the random seed
	srand(seed_start[_parameters.index]);
}


void
LakeFire::propagate_fire()
{
	float prop_dir;
	int8_t cell_val;
	int i_prop;
	int j_prop;

	std::vector<int> i_new;
	std::vector<int> j_new;

	int count = 0;
	_new_fire_count = 0;

	for (int i = 0; i < GRID_WIDTH; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {

			prop_dir = roundf(generate_normal_random(_wind_direction));

			/* make sure this cell isn't a new fire cell */
			if (std::find(i_new.begin(), i_new.end(), i) != i_new.end() && std::find(j_new.begin(), j_new.end(), j) != j_new.end()) {
				continue;
			}

			/* check for fire in cell, continue if no fire in cell */
			cell_val = _grid[i][j];
			if (cell_val != ON_FIRE) continue;
			count++;

			/* wrap the propagation direction to be within (0,8) */
			if (prop_dir < 0) prop_dir += 8;
			if (prop_dir > 7) prop_dir -= 8;

			i_prop = i;
			j_prop = j;

			get_prop_coords(&i_prop, &j_prop, (int) prop_dir);

			/* check to make sure new fire cell is a valid location */
			if (i_prop >= GRID_WIDTH || i_prop < 0 || j_prop >= GRID_WIDTH || j_prop < 0) continue;

			/* check that the new cell is in bounds */
			if (!_grid_mask[i_prop][j_prop]) continue;

			/* check for new fire cell value */
			cell_val = _grid[i_prop][j_prop];
			if (cell_val == OPEN_LAND) {
				/* add fire to this cell */
				_grid[i_prop][j_prop] = ON_FIRE;
				_new_fire_count++;		// increase new fire tally
				i_new.push_back(i_prop);
				j_new.push_back(j_prop);
			}
		}
	}

	/* publish the new fire cells */
	//publish_new_fire(i_new, j_new);

	/* clear vectors after having published the info */
	i_new.clear();
	j_new.clear();

}

void
LakeFire::calculate_score()
{
	int count = 0;
	int8_t cell_val;

	for (int i = 0; i < GRID_WIDTH; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {

			/* check for fire in cell */
			cell_val = _grid[i][j];
			if (cell_val == ON_FIRE) {
				count++;
			}
		}
	}

	_score = (1.0f - (float) count / _unattended_count)*100.0f;
}

void
LakeFire::task_main_trampoline(int argc, char **argv)
{
	 aa241x_mission::g_aa241x_mission->task_main();
	//aa241x_mission::g_aa241x_mission->prop_testing(); // DEBUG
	// aa241x_mission::g_aa241x_mission->testing(); // DEBUG
	// aa241x_mission::g_aa241x_mission->sim_testing(); // DEBUG
}

void
LakeFire::print_grid() {
	printf("\n");
	for (int i = 0; i < GRID_WIDTH; i++) {
		for (int j = 0;j < GRID_WIDTH; j++) {
			printf("%d ", _grid[i][j]);
		}
		printf("\n");
	}
	printf("\n");
}

void
LakeFire::print_mask() {
	printf("\n");
	for (int i = 0; i < GRID_WIDTH; i++) {
		for (int j = 0;j < GRID_WIDTH; j++) {
			printf("%d ", _grid_mask[i][j]);
		}
		printf("\n");
	}
	printf("\n");
}

void
LakeFire::testing()
{
	parameters_update();

	printf("min alt: %0.5f\n", (double) _parameters.min_alt);
	printf("max alt: %0.5f\n", (double) _parameters.max_alt);
	printf("min fov: %0.5f\n", (double) _parameters.min_fov);
	printf("max fov: %0.5f\n", (double) _parameters.max_fov);
	printf("alt auto: %0.5f\n", (double) _parameters.auto_alt);

	printf("cell width: %0.5f\n", (double) _parameters.cell_width);
	printf("duration: %0.5f\n", (double) _parameters.duration);
	printf("max radius: %0.5f\n", (double) _parameters.max_radius);
	printf("timestep: %0.5f\n", (double) _parameters.timestep);
	printf("std: %0.5f\n", (double) _parameters.std);
	printf("t pic: %0.5f\n", (double) _parameters.t_pic);
	printf("index: %i\n", _parameters.index);
	printf("h2o wght: %0.5f\n", (double) _parameters.water_weight);
	printf("wght / drop: %0.5f\n", (double) _parameters.weight_per_drop);

	printf("ctr lat: %0.5f\n", (double) _parameters.ctr_lat);
	printf("ctr lon: %0.5f\n", (double) _parameters.ctr_lon);
	printf("ctr alt: %0.5f\n", (double) _parameters.ctr_alt);

	float fov = 0.0f;
	for (int i=31; i<122; i += 5) {
		fov = get_fov_d((float) i);


		printf("%f -> %0.5f\n", (double) i, (double) fov);
	}


	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
	_exit(0);
}

void
LakeFire::prop_testing()
{
	const int nrolls=10000;  // number of experiments
	const int nstars=100;    // maximum number of stars to distribute
	int mean = 5;

	int p[10]={};

	for (int i=0; i<nrolls; ++i) {
		float number = roundf(generate_normal_random(mean));

		if ((number>=0.0f)&&(number<10.0f)) ++p[int(number)];
	}

	printf("normal_distribution (5):\n");

	for (int i=0; i<10; ++i) {
		printf("%i: ", i);
		int num_stars = p[i]*nstars/nrolls;
		for (int j= 0; j < num_stars; j++) {
			printf("*");
		}
		printf("\n");
	}

	printf("grid mask\n");
	print_mask();

	initialize_mission();
	print_grid();

	for (int i = 0; i < _propagations_remaining; i++) {
		propagate_fire();
		// usleep(500000);
	}

	print_grid();

	printf("\nUnattended count: %f\n", (double) _unattended_count);

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
	_exit(0);
}


void
LakeFire::task_main()
{
	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	/* open connection to mavlink logging */
	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	/* open buzzer */
	_buzzer = open(TONEALARM0_DEVICE_PATH, O_WRONLY);

	/*
	 * do subscriptions
	 */
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_water_drop_request_sub = orb_subscribe(ORB_ID(aa241x_water_drop_request));
	_pic_request_sub = orb_subscribe(ORB_ID(aa241x_picture_request));
	_local_data_sub = orb_subscribe(ORB_ID(aa241x_local_data));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	/* rate limit vehicle status updates and local data updates to 5Hz */
	orb_set_interval(_vcontrol_mode_sub, 200);
	orb_set_interval(_local_data_sub, 200);

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_control_mode_update();
	global_pos_update();
	local_pos_update();
	vehicle_status_update();
	local_data_update();
	battery_status_update();

	/* wakeup source(s) */
	struct pollfd fds[9];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _vcontrol_mode_sub;
	fds[1].events = POLLIN;
	fds[2].fd = _global_pos_sub;
	fds[2].events = POLLIN;
	fds[3].fd = _local_pos_sub;
	fds[3].events = POLLIN;
	fds[4].fd = _vehicle_status_sub;
	fds[4].events = POLLIN;
	fds[5].fd = _pic_request_sub;
	fds[5].events = POLLIN;
	fds[6].fd = _water_drop_request_sub;
	fds[6].events = POLLIN;
	fds[7].fd = _local_data_sub;
	fds[7].events = POLLIN;
	fds[8].fd = _battery_status_sub;
	fds[8].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* vehicle control mode updated */
		if (fds[1].revents & POLLIN) {
			vehicle_control_mode_update();
		}

		/* global position updated */
		if (fds[2].revents & POLLIN) {
			global_pos_update();
		}

		/* local position updated */
		if (fds[3].revents & POLLIN) {
			local_pos_update();
		}

		/* vehicle status updated */
		if (fds[4].revents & POLLIN) {
			vehicle_status_update();
		}

		/* picture request */
		if (fds[5].revents & POLLIN) {
			handle_picture_request();
		}

		/* water drop request */
		if (fds[6].revents & POLLIN) {
			handle_water_drop_request();
		}

		/* local data updated */
		if (fds[7].revents & POLLIN) {
			local_data_update();
		}

		/* battery status updated */
		if (fds[8].revents & POLLIN) {
			battery_status_update();
		}

		/* check auto start requirements */
		if (_can_start && !_in_mission) {

			if (!_vehicle_status.gps_failure && -_local_data.D_gps >= _parameters.auto_alt && !_vcontrol_mode.flag_control_auto_enabled) {
				// not allowed to start is above auto alt and not in auto mode
				_can_start = false;
				mavlink_log_info(_mavlink_fd, "#audio: AA241x mission start conditions violated");
			}
		}

		/* check mission start requirements */
		if (_can_start && !_in_mission) {

			if (-_local_data.D_gps >= _parameters.auto_alt && _vcontrol_mode.flag_control_auto_enabled) {
				/* start the mission once have crossed over the minimum altitude */
				initialize_mission();
			}
		}

		/* ensure abiding by mission rules */
		if (_in_mission) {

			/* check to see if we have crossed min alt for first time */
			if (!_cross_min && -_local_data.D_gps >= _parameters.min_alt) {
				_cross_min = true;
			}

			// ---- Check soft terminations first ---- //

			/* check auto requirements */
			if (!_vcontrol_mode.flag_control_auto_enabled) {
				// end mission and set score to 0 if switch to manual mode
				_in_mission = false;
				_early_termination = true;
				mavlink_log_info(_mavlink_fd, "#audio: AA241x mission termination: control mode violation");
			}

			/* check min altitude requirements (with 10m buffer) only if plane has gotten above it already */
			if (_cross_min && -_local_data.D_gps <= (_parameters.min_alt - 10.0f)) {
				// end mission, but let fire propagate for rest of time
				_in_mission = false;
				_early_termination = true;
				mavlink_log_info(_mavlink_fd, "#audio: AA241x mission termination: below min alt");
			}

			/* check battery requirements */
			if ((_batt_stat.discharged_mah - _mission_start_battery) > _parameters.max_discharge) {
				_in_mission = false;
				_early_termination = true;
				mavlink_log_info(_mavlink_fd, "#audio: AA241x mission termination: max battery discharge reached");
			}

			// propagate through the rest of the fire as needed
			if (_early_termination) {

				/* propagate the rest of the time */
				while (_propagations_remaining > 0) {
					propagate_fire();
					_propagations_remaining--;
					publish_fire_prop();
				}

				publish_condensed_grid();
				calculate_score();
			}

			// ---- Check hard failures ---- //

			/* check strict requirements (max alt and radius) (with 10 and 5 m buffers, respectively) */
			float r2 = _local_data.N*_local_data.N + _local_data.E*_local_data.E;
			float max_r2 = (_parameters.max_radius + 5.0f)*(_parameters.max_radius + 5.0f); // with additional 5 meter buffer
			if (-_local_data.D_gps >= (_parameters.max_alt + 10.0f) || r2 > max_r2) {
				// end mission and set score to 0 if violate max altitude
				_in_mission = false;
				_mission_failed = true;
				_score = 0.0f;
				mavlink_log_info(_mavlink_fd, "#audio: AA241x mission failed: boundary violation");
			}


			// --- Time related checks --- //

			/* get the current time needed for further calculations */
			hrt_abstime current_time = hrt_absolute_time();

			/* ensure we are still in mission and check if timestep has advanced */
			if (_in_mission && (current_time - _last_propagation_time) >= _parameters.timestep*1E6f) {
				_last_propagation_time = current_time;

				/* propagate the fire for this next timestep */
				propagate_fire();
				_propagations_remaining--;
				publish_fire_prop();
				publish_condensed_grid();

				/* calculate the new score */
				calculate_score();
			}

			/* check to see if mission time has elapsed */
			if (_in_mission && (current_time - _mission_start_time)/1000000.0f >= _parameters.duration*60.0f) {
				_in_mission = false;
				_can_start = false;
				mavlink_log_info(_mavlink_fd, "#audio: AA241x mission completed");

				/* make sure that have done all fire propagations */
				if (_propagations_remaining > 0) {
					// TODO: check to make sure there is only ever 1 left here
					propagate_fire();
				}

				// calculate the final score
				calculate_score();
			}


		}

		/* publish the mission status as the last thing to do each loop */
		publish_mission_status();

	}

	warnx("exiting.\n");

	// close the buzzer connection
	close(_buzzer);

	_control_task = -1;
	_task_running = false;
	_exit(0);
}

int
LakeFire::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("aa241x_mission",
			SCHED_DEFAULT,
			SCHED_PRIORITY_DEFAULT + 20,
			1800,
			(main_t)&LakeFire::task_main_trampoline,
			nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}


int aa241x_mission_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: aa241x_mission_control {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (aa241x_mission::g_aa241x_mission != nullptr)
			errx(1, "already running");

		aa241x_mission::g_aa241x_mission = new LakeFire;

		if (aa241x_mission::g_aa241x_mission == nullptr)
			errx(1, "alloc failed");

		if (OK != aa241x_mission::g_aa241x_mission->start()) {
			delete aa241x_mission::g_aa241x_mission;
			aa241x_mission::g_aa241x_mission = nullptr;
			err(1, "start failed");
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		while (aa241x_mission::g_aa241x_mission == nullptr || !aa241x_mission::g_aa241x_mission->task_running()) {
			usleep(50000);
			printf(".");
			fflush(stdout);
		}
		printf("\n");

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (aa241x_mission::g_aa241x_mission == nullptr)
			errx(1, "not running");

		delete aa241x_mission::g_aa241x_mission;
		aa241x_mission::g_aa241x_mission = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (aa241x_mission::g_aa241x_mission) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
