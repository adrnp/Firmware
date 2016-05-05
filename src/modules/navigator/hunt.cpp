/*
 * hunt.cpp
 *
 *  Created on: Aug 5, 2014
 *      Author: Adrien Perkins <adrienp@stanford.edu>
 *
 *      Navigation mode for hunting down signal
 *      allows for executing navigation commands generated from offboard in real time
 */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>
#include <mathlib/mathlib.h>

#include <mavlink/mavlink_log.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>

#include "navigator.h"
#include "hunt.h"

Hunt::Hunt(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
/* hunt state/logic */
	_started(false),
	_current_cmd_id(0),
    _hunt_state(hunt_state_s::HUNT_STATE_OFF),

/* params */
	_param_yaw_increment(this, "HUNT_YAW_STEP", false),
	_param_start_lat(this, "HUNT_STRT_LAT", false),
	_param_start_lon(this, "HUNT_STRT_LON", false),
	_param_start_alt(this, "HUNT_STRT_ALT", false),
	_param_start_hdg(this, "HUNT_STRT_HDG", false),


/* subscriptions */
	_local_pos_sub(-1),

/* publications */
    _hunt_result_pub(nullptr),
    _hunt_state_pub(nullptr),

/* rotation handling */
	_current_rotation_direction(0),
	_end_rotation_angle(0),
	_total_rotation(0),
	_prev_yaw(0),
	_in_rotation(false),
	_allow_rotation_end(false),
    _start_rotation_angle(0),
    _new_total_rotation(0),

/* time */
	_temp_time(hrt_absolute_time()),
	_test_time(hrt_absolute_time()),
	_ref_timestamp(0)
{
	/* initialize structs */
	_local_pos = {};
	_ref_pos = {};
	_tracking_cmd = {};
	_hunt_result = {};
	_hunt_state_s = {};

	/* load initial params */
	updateParams();

	/* initial reset */
	on_inactive();
}

Hunt::~Hunt()
{
	// destructor, don't think need anything here

	// maybe close publisher...
}

void
Hunt::on_inactive()
{
	// called when the hunt mode is made inactive
	// need to reset some of the parameters

	// maybe need to think about doing a suspended mode instead of off
	// would make the _started parameter not needed?
	/* put the hunt into off mode */
    _hunt_state = hunt_state_s::HUNT_STATE_OFF;
}

void
Hunt::on_activation()
{
	// called when we hunt mode gets activated

	if (!_started) { // hunt not started, meaning this is the first time we have activated hunt
		// go to start position

        /* send a message that hunt mode has been activated */
        mavlink_log_info(_navigator->get_mavlink_fd(), "#audio: hunt started");

		// set the state to moving to first position
		// XXX: maybe don't need state start, and just set it to state move here
		// _hunt_state = HUNT_STATE_START;

		// make sure it is known we cannot use the current mission item for loiter
		// just needed for initial hover
		_navigator->set_can_loiter_at_sp(false);

		_started = true;

		move_to_start();

	} else {
		// need to check if we are at the last location
		// if not at the last commanded location, go to last commanded location

		// update our copy of the local position
		update_local_position();

		// update the reference position
		update_reference_position();

		// move to the starting location
		move_to_start();

		/*
		// create a mission item for the current location
		_mission_item.lat = _navigator->get_global_position()->lat;
		_mission_item.lon = _navigator->get_global_position()->lon;
		_mission_item.altitude = _navigator->get_global_position()->alt;
		_mission_item.altitude_is_relative = false;

		// XXX: FOR NOW JUST GO INTO WAITING MODE REGARDLESS OF PREVIOUS START MODE
		_hunt_state = HUNT_STATE_WAIT;
		set_waiting(); // trigger it loitering just for safety

		// broadcast the status change
		// TODO: make change the state a function, will better outline the state machine...
		report_status();
		*/

	}
}

void
Hunt::on_active()
{
	// TODO: this is where all the stuff runs

	// update our copy of the local position
	update_local_position();

	// update the reference position
	update_reference_position();

	// only check mission success when not in a waiting mode (to avoid always checking whether or not
	// we have reached the cmd when we are just waiting around)
	// TODO: use own is mission item reached.... trying to use the existing one may be the source of some problems
    if (_hunt_state == hunt_state_s::HUNT_STATE_MOVE && is_mission_item_reached()) {
		mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] travel completed");

		_test_time = hrt_absolute_time(); // update the test time, this is for a delay between

		// we finished with the cmd, so broadcast that
		report_cmd_finished();

		// change the state to waiting since we have reached the target
        _hunt_state = hunt_state_s::HUNT_STATE_WAIT;

		// have vehicle start waiting
		set_waiting();

		// reset whether or not we have reached the mission item
		reset_mission_item_reached();

		// state changed here, so report it
		report_status();

    } else if (_hunt_state == hunt_state_s::HUNT_STATE_ROTATE) {

		/* check to see if rotation finished */
		if (is_mission_item_reached()) {

			mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] rotation completed");
			_allow_rotation_end = false;
			_in_rotation = false;

			report_cmd_finished();

            _hunt_state = hunt_state_s::HUNT_STATE_WAIT;
			set_waiting();
			reset_mission_item_reached();
			report_cmd_finished();

		} else {	/* still rotating */


			if (get_next_cmd()) { /* termination of direction change requested */

				set_next_item();

			} else {	/* continue rotation */

				// need to calculate the yaw increment
				float currentYaw2pi = _wrap_2pi(_navigator->get_global_position()->yaw);
				float prevYaw2pi = _wrap_2pi(_prev_yaw);
				float dtheta = math::min(fabsf(prevYaw2pi - currentYaw2pi), _wrap_2pi((M_TWOPI_F - prevYaw2pi) + currentYaw2pi));
				_total_rotation += dtheta;
                //mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] total rotation: %2.3f deg", (double) math::degrees(_total_rotation));

                // new calculation of total rotation
                if (_current_rotation_direction > 0) {  // clockwise rotation

                    if (_start_rotation_angle < currentYaw2pi) {
                        _new_total_rotation = currentYaw2pi - _start_rotation_angle;
                    } else {
                        _new_total_rotation = (M_TWOPI_F - _start_rotation_angle) + currentYaw2pi;
                    }

                } else {    // counterclockwise rotation

                    if (_start_rotation_angle < currentYaw2pi) {
                        _new_total_rotation = _start_rotation_angle + (M_TWOPI_F - currentYaw2pi);
                    } else {
                        _new_total_rotation = _start_rotation_angle - currentYaw2pi;
                    }
                }
                mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] total rotation: %2.3f deg", (double) math::degrees(_new_total_rotation));

				// determine the total rotation threshold
				// TODO: this should be done elsewhere!!!
				float yaw_step = _param_yaw_increment.get();
				float threshold = M_TWOPI_F - math::radians(90.0f);
				if (yaw_step > 0) {
					threshold = M_TWOPI_F - math::radians(yaw_step);
				}

				// want to limit to 1 rotation, so only continue rotation if not going to complete 1 full rotation
				if (_total_rotation < threshold) {
                    continue_rotation();
				} else { // DEBUG
					// TESTING
					// _tracking_cmd.yaw_angle = -1.0;
					// start_rotation();
				}
			}
		}
	}



	// XXX: not sure if or or and is best here, depends on when hunt state is
	// changed to wait
    if (_hunt_state == hunt_state_s::HUNT_STATE_WAIT) {
		// we have reached the desired point or are waiting

		// ------------ DEBUG ------------------- //
		// just doing a rotation here
		// -------------------------------------- //
		// rotate();


		if (get_next_cmd()) {
			// new command has come from tracking
			set_next_item();

			// set next item will handle the state change to a none wait state
		}
	}

	// always report the status here, just so there is a constant new mavlink message
	report_status();

	// XXX: IMPORTANT
	// TODO: figure out how to broadcast done with the command

}

bool
Hunt::get_next_cmd()
{
	bool updated = false;
	orb_check(_navigator->get_hunt_mission_sub(), &updated);

	if (updated) {
		// copy over the command
		orb_copy(ORB_ID(tracking_cmd), _navigator->get_hunt_mission_sub(), &_tracking_cmd);
		return true;
	}

	return false;
}

void
Hunt::set_next_item()
{
	/* get pointer to the position setpoint from the navigator */
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* make sure we have the latest params */
	updateParams();

	/* just copy over what is the current setpoint to the previous one, since we will be setting a new current setpoint */
	set_previous_pos_setpoint();

	// update the current cmd id to be that of the new cmd
	_current_cmd_id = _tracking_cmd.cmd_id;

	// not sure about this
	// XXX: figure out difference between can loiter at sp and can't loiter
	// who looks at this... (will be setting it as true... others have it as false...)
	_navigator->set_can_loiter_at_sp(true);

	// XXX: IMPORTANT
	// TODO: should do a distance to home check on this to make sure we are not commanded to go insanely far

	// create a mission item from the tracking cmd
	switch (_tracking_cmd.cmd_type) {
    case tracking_cmd_s::HUNT_CMD_TRAVEL: {
		mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] traveling");
		/* change the hunt state to move */
        _hunt_state = hunt_state_s::HUNT_STATE_MOVE;

		/* get desired north and east distances of travel */
		set_mission_latlon();

		_mission_item.yaw = math::radians(270.0f);	// for now just go with point north

		// setting the altitude of the mission item for a move command
		_mission_item.altitude_is_relative = false;
		_mission_item.altitude = _tracking_cmd.altitude;

		break;
	}
    case tracking_cmd_s::HUNT_CMD_ROTATE: {

		// if we are already in a rotation state, will want to terminate that rotation first
        if (_hunt_state == hunt_state_s::HUNT_STATE_ROTATE) {
			end_rotation();
		}

		/* change the hunt state to rotate */
        _hunt_state = hunt_state_s::HUNT_STATE_ROTATE;
		mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] rotating");

		// do the management for starting a rotation
		start_rotation();

		break;
	}
    case tracking_cmd_s::HUNT_CMD_FINISH: {
		// change the hunt state to off
        _hunt_state = hunt_state_s::HUNT_STATE_OFF;

		set_waiting();

		// report the new cmd id
		report_cmd_id();

		// status must have changed if at this point, so report it
		report_status();

		// straight up return from here...
		return;
	}
	default:
		break;

	}

	// do all the general constant stuff (same for all mission items)
	_mission_item.loiter_radius = _navigator->get_loiter_radius();	// loiter radius and direction will be same, regardless of cmd type
	_mission_item.loiter_direction = 1;
	_mission_item.nav_cmd = NAV_CMD_WAYPOINT;						// nav cmd will be waypoint regardless of cmd type
	_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
	_mission_item.time_inside = 0.0f;
	_mission_item.pitch_min = 0.0f;
	_mission_item.autocontinue = false;
	_mission_item.origin = ORIGIN_TRACKING;

	// need to reset the mission item reached info
	// XXX: THIS MAY BE UNNECESSARY??
	reset_mission_item_reached();

	/* convert mission item to current position setpoint and make it valid */
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();

	// report the new cmd id
	report_cmd_id();

	// status must have changed if at this point, so report it
	report_status();

}


void
Hunt::set_waiting()
{
	mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] set to waiting");
	/* get pointer to the position setpoint from the navigator */
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* make sure we have the latest params */
	updateParams();

	/* just copy over what is the current setpoint to the previous one, since we will be setting a new current setpoint */
	set_previous_pos_setpoint();

	/* set loiter mission item */
	set_loiter_item(&_mission_item);

	/* update position setpoint triplet  */
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	// _navigator->set_can_loiter_at_sp(pos_sp_triplet->current.type == SETPOINT_TYPE_LOITER); // this is what mission does

	// XXX: really not sure...
	// _navigator->set_can_loiter_at_sp(true);

	_navigator->set_position_setpoint_triplet_updated();
}

void
Hunt::report_cmd_finished()
{
	_hunt_result.reached = true;
	_hunt_result.cmd_reached = _current_cmd_id;

	publish_hunt_result();
}

void
Hunt::report_cmd_id()
{
	// just make sure that reached and finished are both false
	_hunt_result.reached = false;
	_hunt_result.finished = false;
	_hunt_result.cmd_current = _current_cmd_id;
	publish_hunt_result();
}


void
Hunt::publish_hunt_result()
{
	/* lazily publish the mission result only once available */
    if (_hunt_result_pub == nullptr) {
        /* advertise and publish */
        _hunt_result_pub = orb_advertise(ORB_ID(hunt_result), &_hunt_result_pub);
	} else {
        /* publish mission result */
        orb_publish(ORB_ID(hunt_result), _hunt_result_pub, &_hunt_result_pub);
	}

	/* reset reached bool */
	_hunt_result.reached = false;
	_hunt_result.finished = false;
}

void
Hunt::report_status()
{
	// update the tracking status state to the current hunt state
	_hunt_state_s.hunt_mode_state = _hunt_state;

	// need to then publish the status change
	publish_status();
}


void
Hunt::publish_status()
{
	/* lazily publish the hunt state only once available */
    if (_hunt_state_pub == nullptr) {
        /* advertise and publish */
        _hunt_state_pub = orb_advertise(ORB_ID(hunt_state), &_hunt_state_s);
	} else {
        /* publish current hunt state */
        orb_publish(ORB_ID(hunt_state), _hunt_state_pub, &_hunt_state_s);
	}
}


void
Hunt::update_local_position()
{
    if (_local_pos_sub < 0) {
        _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
        orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	} else {
        orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}

}


void
Hunt::update_reference_position()
{
	// if we have a new local position, update the reference position
	if (_local_pos.ref_timestamp != _ref_timestamp) {

		/* update local projection reference */
		map_projection_init(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon);
		// NOTE: these reference lat and lon points should never change...

		_ref_timestamp = _local_pos.ref_timestamp;
	}
}


void
Hunt::set_mission_latlon()
{
	// calculate the desired noth and east positions in the local frame
	float north_desired = _tracking_cmd.north + _local_pos.x;
	float east_desired = _tracking_cmd.east + _local_pos.y;

	// now need to convert from the local frame to lat lon, will also directly set it
	// to the mission item while we are at it
	map_projection_reproject(&_ref_pos, north_desired, east_desired, &_mission_item.lat, &_mission_item.lon);
}


void
Hunt::rotate()
{
	/* change the hunt state to rotate */
    _hunt_state = hunt_state_s::HUNT_STATE_ROTATE;

	/* get pointer to the position setpoint from the navigator */
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	_current_rotation_direction = 1;

	/* we want to just sit in one spot */
	_mission_item.lat = _navigator->get_global_position()->lat;
	_mission_item.lon = _navigator->get_global_position()->lon;


	float yaw_step = _param_yaw_increment.get();
	if (yaw_step > 0) {
		_mission_item.yaw = _navigator->get_global_position()->yaw + (float) _current_rotation_direction * math::radians(yaw_step);
	} else {
		_mission_item.yaw = _navigator->get_global_position()->yaw + (float) _current_rotation_direction * math::radians(90.0f);
	}

	_mission_item.yaw = _wrap_pi(_mission_item.yaw);

	// setting the altitude of the mission item for a rotate command (just want to use current altitude)
	_mission_item.altitude_is_relative = false;
	_mission_item.altitude = _navigator->get_global_position()->alt;

	// loiter radius and direction will be same, regardless of cmd type
	_mission_item.loiter_radius = _navigator->get_loiter_radius();
	_mission_item.loiter_direction = 1;

	// nav cmd will be waypoint regardless of cmd type
	_mission_item.nav_cmd = NAV_CMD_WAYPOINT;

	// all other odds and ends of mission item will be the same regardless of cmd type
	_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
	_mission_item.time_inside = 0.0f;
	_mission_item.pitch_min = 0.0f;
	_mission_item.autocontinue = false;
	_mission_item.origin = ORIGIN_TRACKING;


	// need to reset the mission item reached info
	// XXX: THIS MAY BE UNNECESSARY??
	reset_mission_item_reached();

	/* convert mission item to current position setpoint and make it valid */
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();

	// report the new cmd id
	report_cmd_id();

	// status must have changed if at this point, so report it
	report_status();
}


void
Hunt::start_rotation()
{

	_in_rotation = true;
	_end_rotation_angle = _navigator->get_global_position()->yaw; // want to rotate 360 degrees
	_prev_yaw = _navigator->get_global_position()->yaw;
	_total_rotation = 0.0f;

    _start_rotation_angle = _wrap_2pi(_navigator->get_global_position()->yaw);
    _new_total_rotation = 0.0f;

	// get the data from the tracking command (which direction to rotate)
	_current_rotation_direction = (int) _tracking_cmd.yaw_angle;
	if (_current_rotation_direction == 0) {
		_current_rotation_direction = 1;
	}

	/* we want to just sit in one spot at the current altitude */
	_mission_item.lat = _navigator->get_global_position()->lat;
	_mission_item.lon = _navigator->get_global_position()->lon;
	_mission_item.altitude_is_relative = false;
	_mission_item.altitude = _navigator->get_global_position()->alt;

	// start the yaw rotation
	float yaw_step = _param_yaw_increment.get();
	if (yaw_step > 0) {
		_mission_item.yaw = _navigator->get_global_position()->yaw + (float) _current_rotation_direction * math::radians(yaw_step);
	} else {
		_mission_item.yaw = _navigator->get_global_position()->yaw + (float) _current_rotation_direction * math::radians(90.0f);
	}
	_mission_item.yaw = _wrap_pi(_mission_item.yaw);

}

void
Hunt::continue_rotation()
{
	_prev_yaw = _navigator->get_global_position()->yaw;

	/* get pointer to the position setpoint from the navigator */
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// do not need to update anything but the yaw on the mission item, want to keep everything else the same

	// continue rotation
	float yaw_step = _param_yaw_increment.get();
	if (yaw_step > 0) {
		_mission_item.yaw = _navigator->get_global_position()->yaw + (float) _current_rotation_direction * math::radians(yaw_step);
	} else {
		_mission_item.yaw = _navigator->get_global_position()->yaw + (float) _current_rotation_direction * math::radians(90.0f);
	}
	_mission_item.yaw = _wrap_pi(_mission_item.yaw);

	/* convert mission item to current position setpoint and make it valid */
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}


void
Hunt::end_rotation()
{
	_prev_yaw = _navigator->get_global_position()->yaw;

	/* get pointer to the position setpoint from the navigator */
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// do not need to update anything but the yaw on the mission item, want to keep everything else the same

	// set the target yaw to be the current heading (+ 5 deg for some margin)
	_mission_item.yaw = _navigator->get_global_position()->yaw + (float) _current_rotation_direction * math::radians(5.0f);

	/* convert mission item to current position setpoint and make it valid */
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}


void
Hunt::move_to_start()
{
	// set the hunt state to move (since we are moving)
    _hunt_state = hunt_state_s::HUNT_STATE_MOVE;

	/* get pointer to the position setpoint from the navigator */
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	float start_lat = _param_start_lat.get();
	float start_lon = _param_start_lon.get();
	float start_alt = _param_start_alt.get();
	float start_hdg = _param_start_hdg.get();

	_mission_item.lat = (double) start_lat;
	_mission_item.lon = (double) start_lon;
	_mission_item.altitude = start_alt;
	_mission_item.altitude_is_relative = false;

	_mission_item.yaw = math::radians(start_hdg);

	// loiter radius and direction will be same, regardless of cmd type
	_mission_item.loiter_radius = _navigator->get_loiter_radius();
	_mission_item.loiter_direction = 1;

	// nav cmd will be waypoint regardless of cmd type
	_mission_item.nav_cmd = NAV_CMD_WAYPOINT;

	// all other odds and ends of mission item will be the same regardless of cmd type
	_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
	_mission_item.time_inside = 0.0f;
	_mission_item.pitch_min = 0.0f;
	_mission_item.autocontinue = false;
	_mission_item.origin = ORIGIN_TRACKING;


	// need to reset the mission item reached info
	// XXX: THIS MAY BE UNNECESSARY??
	reset_mission_item_reached();

	/* convert mission item to current position setpoint and make it valid */
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();

	// report the new cmd id
	report_cmd_id();

	// status must have changed if at this point, so report it
	report_status();

}

