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

#include <mavlink/mavlink_log.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>

#include "navigator.h"
#include "hunt.h"

Hunt::Hunt(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_hunt_state(HUNT_STATE_OFF),
	_started(false),
	_current_cmd_id(-1),
	_tracking_cmd({0}),
	_hunt_result_pub(-1),
	_hunt_result({0}),
	_hunt_state_pub(-1),
	_hunt_state_s({0})
{
	/* load initial params */
	updateParams();

	/* initial reset */
	on_inactive();
}

Hunt::~Hunt()
{
	// destructor, don't think need anything here
}

void
Hunt::on_inactive()
{
	// called when the hunt mode is made inactive
	// need to reset some of the paramters

	// maybe need to think about doing a suspended mode instead of off
	// would make the _started parameter not needed?
	/* put the hunt into off mode */
	_hunt_state = HUNT_STATE_OFF;
}

void
Hunt::on_activation()
{
	// called when we hunt mode gets activiated

	if (!_started) { // hunt not started, meaning this is the first time we have activated hunt
		// go to start position

		// set the state to moving to first position
		// XXX: maybe don't need state start, and just set it to state move here
		// _hunt_state = HUNT_STATE_START;



	} else {
		// need to check if we are at the last location
		// if not at the last commanded location, go to last commanded location
	}

	// XXX: FOR NOW JUST GO INTO WAITING MODE REGARDLESS OF PREVIOUS START MODE
	_hunt_state = HUNT_STATE_WAIT;
	set_waiting(); // trigger it loitering just for safety

	// broadcast the status change
	// TODO: make change the state a function, will better outline the state machine...
	report_status();

	// set the created mission item to a position setpoint?
	// also handle some of the states??
	set_next_item();

}

void
Hunt::on_active()
{
	// TODO: this is where all the stuff runs

	if (is_mission_item_reached()) {

		// we finished with the cmd, so broadcast that
		report_cmd_finished();

		// should never not be the case here, but have this if statement just in case....
		if (_hunt_state != HUNT_STATE_WAIT) {
			_hunt_state = HUNT_STATE_WAIT;

			// have vehicle start waiting
			set_waiting();

			// reset whether or not we have reached the mission item
			reset_mission_item_reached();

			// state changed here, so report it
			report_status();
		}
	}



	// XXX: not sure if or or and is best here, depends on when hunt state is
	// changed to wait
	if (_hunt_state == HUNT_STATE_WAIT) {
		// we have reached the desired point or are waiting

		if (get_next_cmd()) {
			// new command has come from tracking
			set_next_item();

			// set next item will handle the state change to a none wait state
		} /*else {

			// if we were not in a waiting state, need to go into a waiting state
			// and need to tell the vehicle to loiter here (I think)
			if (_hunt_state != HUNT_STATE_WAIT) {
				// change our state to waiting
				_hunt_state = HUNT_STATE_WAIT;

				// put the vehicle into a waiting mode
				set_waiting();
			}
		}*/
	}

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

	double northDist = 0.0;
	double eastDist = 0.0;

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
	case HUNT_CMD_TRAVEL: {

		/* get desired north and east distances of travel */
		northDist = _tracking_cmd.north; // not south is just a negative north distance
		eastDist = _tracking_cmd.east;

		// compute and set the desired latitude and longitude from
		// the desired travel distances
		// XXX: east might be backwards...
		_mission_item.lat = _navigator->get_global_position()->lat + M_RAD_TO_DEG*(northDist/6378137.0);
		_mission_item.lon = _navigator->get_global_position()->lon + M_RAD_TO_DEG*(eastDist/6378137.0)/cos(_navigator->get_global_position()->lat);

		// TODO: maybe change the desired yaw to be in the direction assumed of the jammer
		_mission_item.yaw = 0.0; // want to keep facing north during moves so that when a rotation is called, we know we are starting from north

		// change the hunt state to move
		_hunt_state = HUNT_STATE_MOVE;


		break;
	}
	case HUNT_CMD_ROTATE: {
		// TODO: implement ability to rotate

		// change the hunt state to rotate
		_hunt_state = HUNT_STATE_ROTATE;


		break;
	}
	default:
		break;

	}

	// setting the altitude of the mission item will be the same regardless of cmd type
	_mission_item.altitude_is_relative = false;
	_mission_item.altitude = _tracking_cmd.altitude;

	// loiter radius and direction will be same, regardless of cmd type
	_mission_item.loiter_radius = _navigator->get_loiter_radius();
	_mission_item.loiter_direction = 1;

	// nav cmd will be waypoint regardless of cmd type
	_mission_item.nav_cmd = NAV_CMD_WAYPOINT;

	// all other odds and ends of mission item will be the same regardless of cmd type
	_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
	_mission_item.time_inside = 0.0f;
	_mission_item.pitch_min = 0.0f;
	_mission_item.autocontinue = true;
	_mission_item.origin = ORIGIN_TRACKING;


	// need to reset the mission item reached info
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
	_navigator->set_can_loiter_at_sp(true);

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
	_hunt_result.cmd_current = _current_cmd_id;
	publish_hunt_result();
}


void
Hunt::publish_hunt_result()
{
	/* lazily publish the mission result only once available */
	if (_hunt_result_pub > 0) {
		/* publish mission result */
		orb_publish(ORB_ID(hunt_result), _hunt_result_pub, &_hunt_result_pub);
	} else {
		/* advertise and publish */
		_hunt_result_pub = orb_advertise(ORB_ID(hunt_result), &_hunt_result_pub);
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
}


void
Hunt::publish_status()
{
	/* lazily publish the hunt state only once available */
	if (_hunt_state_pub > 0) {
		/* publish mission result */
		orb_publish(ORB_ID(tracking_status), _hunt_state_pub, &_hunt_state_s);
	} else {
		/* advertise and publish */
		_hunt_state_pub = orb_advertise(ORB_ID(tracking_status), &_hunt_state_s);
	}
}


