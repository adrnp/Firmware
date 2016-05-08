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
	_prev_yaw(0),
	_in_rotation(false),
	_allow_rotation_end(false),
    _start_rotation_angle(0),
    _total_rotation(0),

/* time */
	_temp_time(hrt_absolute_time()),
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

    // send status updates
    mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] moving to start");  // alert to ground


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
        report_state();
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

    switch (_hunt_state) {

    /* moving to a position */
    case hunt_state_s::HUNT_STATE_MOVE:

        /* check to see if we have reached out destination */
        if (is_mission_item_reached()) {

            reset_mission_item_reached();  // reset mission item status (since we want to move to next thing)

            // send status updates
            mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] travel completed");  // alert to ground

            report_cmd_finished();  // pushlish to topic (alert to odroid via mavlink)

            // update the state (now waiting for next command)
            _hunt_state = hunt_state_s::HUNT_STATE_WAIT;
            set_waiting();      // have vehicle start waiting
        }

        // keep letting it move...

        break;

    /* rotating at a given position */
    case hunt_state_s::HUNT_STATE_ROTATE:

        /* check to see if rotation finished */
        if (is_mission_item_reached()) {

            reset_mission_item_reached();   // reset mission item status

            // send status updates
            mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] rotation completed");

            report_cmd_finished();

            // update the rotation handling stuff
            // TODO: figure out if this is still needed???
            _allow_rotation_end = false;
            _in_rotation = false;

            // update the state (now waiting for the next command)
            _hunt_state = hunt_state_s::HUNT_STATE_WAIT;
            set_waiting();  // have vehicle start waiting

        } else {    /* still rotating */


            if (get_next_cmd()) {   /* termination of direction change requested */

                set_next_item();

            } else {	/* continue rotation */

                // update how much we have rotated by
                update_total_rotation();

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
                }
            }
        }

        break;

    /* waiting for the next command */
    case hunt_state_s::HUNT_STATE_WAIT:

        // check for next command and set it if present
        if (get_next_cmd()) {

            set_next_item();    // new command has come from tracking
        }

        break;
    }

	// always report the status here, just so there is a constant new mavlink message
    report_state();

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
    mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] new command received");

    /* get pointer to the position setpoint from the navigator */
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* make sure we have the latest params */
	updateParams();

	/* just copy over what is the current setpoint to the previous one, since we will be setting a new current setpoint */
	set_previous_pos_setpoint();

	// update the current cmd id to be that of the new cmd
    _current_cmd_id = 0;//_tracking_cmd.cmd_id;


	// XXX: IMPORTANT
	// TODO: should do a distance to home check on this to make sure we are not commanded to go insanely far

	// create a mission item from the tracking cmd
	switch (_tracking_cmd.cmd_type) {

    /* move commanded */
    case tracking_cmd_s::HUNT_CMD_TRAVEL:
		mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] traveling");

        _hunt_state = hunt_state_s::HUNT_STATE_MOVE;    // update state


        mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] cmnd (%d): N: %0.1f, E:%0.1f, A: %0.1f", _tracking_cmd.cmd_id, (double)_tracking_cmd.north, (double)_tracking_cmd.east, (double)_tracking_cmd.altitude);

        // update mission items

        set_mission_latlon();                       // set the lat/lon for mission item (from N,E)
        _mission_item.yaw = math::radians(270.0f);	// for now just go with point west
        _mission_item.altitude_is_relative = false;         // abs altitudes
        _mission_item.altitude = _tracking_cmd.altitude;

		break;

    /* rotation commanded */
    case tracking_cmd_s::HUNT_CMD_ROTATE:
        mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] rotating");

		// if we are already in a rotation state, will want to terminate that rotation first
        if (_hunt_state == hunt_state_s::HUNT_STATE_ROTATE) {
			end_rotation();
		}

        _hunt_state = hunt_state_s::HUNT_STATE_ROTATE;  // update state

        start_rotation();   // do the management for starting a rotation

		break;

    /* finishing commanded */
    case tracking_cmd_s::HUNT_CMD_FINISH:
        mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] finished");

        _hunt_state = hunt_state_s::HUNT_STATE_OFF;  // change the hunt state to off
        set_waiting();  // let the vehicle just sit here

        // report stuff, since returning from here
        report_cmd_id();  // report the new cmd id
        report_state();

        return;  // straight up return from here...

	default:
        mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] unknown command");

        // if don't know command, just set to waiting
        set_waiting();
        report_cmd_id();
        report_state();
        return;
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

    _navigator->set_can_loiter_at_sp(true); // allow the vehicle to loiter at this setpoint

	// need to reset the mission item reached info
	reset_mission_item_reached();

	/* convert mission item to current position setpoint and make it valid */
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();

    // publish the command and the state
    report_cmd_id();    // report the new cmd id
    report_state();     // status must have changed if at this point, so report it
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

	_navigator->set_position_setpoint_triplet_updated();
}


void
Hunt::set_mission_latlon()
{
    // calculate the desired noth and east positions in the local frame
    float north_desired = _tracking_cmd.north + _local_pos.x;
    float east_desired = _tracking_cmd.east + _local_pos.y;

    // default to moving to the starting position
    _mission_item.lat = (double) _param_start_lat.get();
    _mission_item.lon = (double) _param_start_lon.get();

    // now need to convert from the local frame to lat lon, will also directly set it
    // to the mission item while we are at it
    double next_lat;
    double next_lon;
    if (map_projection_reproject(&_ref_pos, north_desired, east_desired, &next_lat, &next_lon) == 0) {
        _mission_item.lat = next_lat;
        _mission_item.lon = next_lon;
    } else {
        // error
        mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] unable to do reprojection");
    }
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
        _hunt_result_pub = orb_advertise(ORB_ID(hunt_result), &_hunt_result);
	} else {
        /* publish mission result */
        orb_publish(ORB_ID(hunt_result), _hunt_result_pub, &_hunt_result);
	}

	/* reset reached bool */
	_hunt_result.reached = false;
	_hunt_result.finished = false;
}

void
Hunt::report_state()
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
Hunt::update_total_rotation()
{
    // new calculation of total rotation
    float currentYaw2pi = _wrap_2pi(_navigator->get_global_position()->yaw);
    if (_current_rotation_direction > 0) {  // clockwise rotation

        if (_start_rotation_angle < currentYaw2pi) {
            _total_rotation = currentYaw2pi - _start_rotation_angle;
        } else {
            _total_rotation = (M_TWOPI_F - _start_rotation_angle) + currentYaw2pi;
        }

    } else {    // counterclockwise rotation

        if (_start_rotation_angle < currentYaw2pi) {
            _total_rotation = _start_rotation_angle + (M_TWOPI_F - currentYaw2pi);
        } else {
            _total_rotation = _start_rotation_angle - currentYaw2pi;
        }
    }

    // DEBUG
    //mavlink_log_info(_navigator->get_mavlink_fd(), "[HUNT] total rotation: %2.3f deg", (double) math::degrees(_total_rotation));
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
    report_state();
}


void
Hunt::start_rotation()
{

	_in_rotation = true;
	_end_rotation_angle = _navigator->get_global_position()->yaw; // want to rotate 360 degrees
	_prev_yaw = _navigator->get_global_position()->yaw;
	_total_rotation = 0.0f;

    _start_rotation_angle = _wrap_2pi(_navigator->get_global_position()->yaw);
    _total_rotation = 0.0f;

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
    report_state();

}

