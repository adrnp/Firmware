/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file lbmp.cpp
 *
 * Locata Binary Message Protocol (LBMP) implementation.
 *
 * @author Adrien Perkins <adrienp@stanford.edu>
 *
 * @see Locata LBMP documentation (insert link)
 */


#include <assert.h>
#include <math.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/satellite_info.h>
#include <drivers/drv_hrt.h>

#include "lbmp.h"


#define LBMP_WAIT_BEFORE_READ	20		// ms, wait before reading to save read() calls

// some powers as defines
#define POW_2_7		128.0f
#define POW_2_10 	1024.0f
#define POW_2_15	32768.0f
#define POW_2_38	274877906944.0

/**** Trace macros, disable for production builds */
#define LBMP_TRACE_PARSER(s, ...)	{printf(s, ## __VA_ARGS__);}	/* decoding progress in parse_char() */


LBMP::LBMP(const int &fd, struct vehicle_gps_position_s *gps_position) :
    _fd(fd),
    _gps_position(gps_position),
    _sync_index(0)
    // TODO: add other constants
{
    _Crc32 = new CRC32();
    decode_init();
}

LBMP::~LBMP()
{
}

int
LBMP::configure(unsigned &baudrate)
{
    /* set baudrate first */
    if (GPS_Helper::set_baudrate(_fd, LBMP_BAUDRATE) != 0) {
        printf("unable to set baudrate\n");
        return -1;
    }

    baudrate = LBMP_BAUDRATE;

    return 0;
}


int
LBMP::receive(unsigned timeout)
{
    /* poll descriptor */
    pollfd fds[1];
    fds[0].fd = _fd;
    fds[0].events = POLLIN;

    uint8_t buf[128];
    //uint8_t buf[1];

    /* timeout additional to poll */
    uint64_t time_started = hrt_absolute_time();

    ssize_t count = 0;
    int handled = 0;

    printf("[LBMP] reading....\n");

    while (true) {

        /* poll for new data */
        int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), timeout);

        if (ret < 0) {
            /* something went wrong when polling */
            printf("[LBMP] polling error\n");
            return -1;

        } else if (ret == 0) {
            /* timeout */
            printf("[LBMP] timeout\n");
            return -1;

        } else if (ret > 0) {
            /* if we have new data from GPS, go handle it */
            if (fds[0].revents & POLLIN) {
                /*
                 * We are here because poll says there is some data, so this
                 * won't block even on a blocking device. But don't read immediately
                 * by 1-2 bytes, wait for some more data to save expensive read() calls.
                 * If more bytes are available, we'll go back to poll() again.
                 */
                usleep(LBMP_WAIT_BEFORE_READ * 1000);
                count = ::read(_fd, buf, sizeof(buf));

                //printf("[LBMP] read %d bytes\n", count);

                /* pass received bytes to the packet decoder */
                for (int i = 0; i < count; i++) {
                    //handled |= parse_char(buf[i]);
                    handled = parse_char(buf[i]);
                    if (handled > 0) {
                        printf("[LBMP] handled > 0\n");
                    }

                }

                /* new message completed, return */
                if (handled) {
                    printf("[LBMP] returning handled = %d\n", handled);
                    return handled;
                }
            }
        }

        /* abort after timeout if no useful packets received */
        if (time_started + timeout * 1000 < hrt_absolute_time()) {
            printf("[LBMP] eol timeout\n");
            return -1;
        }
    }
}

// initialize all the globals for a successful message decoding
void
LBMP::decode_init()
{
    printf("[LBMP] initializing decoding\n");

    /* set state to wait for sync */
    _decode_state = LBMP_DECODE_SYNC;
    _sync_index = 0;

    /* reinit the payload starting at the header */
    _payload_index = 0;
    _payload_length = LBMP_HEADER_LEN;

    /* initialize other elements */
    decode_block_reinit(LBMP_HEADER_LEN);
    decode_checksum_reinit();
}

void
LBMP::decode_block_reinit(const int size)
{
    /* reinit the block, filling buffer backwards */
    _block_index = size - 1;
}

void
LBMP::decode_checksum_reinit()
{
    /* reinit the checksum payload, checksum is 4 bytes */
    _checksum_index = LBMP_CHECKSUM_LEN - 1;
    _Crc32->init();
}


int  // 0 = decoding, 1 = message handled, 2 = "sat" message handled
LBMP::parse_char(const uint8_t c)
{
    int ret = 0;  // default to still decoding

    switch(_decode_state) {

    /* Expecting Sync message */
    case LBMP_DECODE_SYNC:
        //printf("A");
        //printf("%02X ", c);

        if (c == LBMP_SYNC[_sync_index]) {	// got a sync byte
            _sync_index++;
            printf("%02x ", c);
        } else {	// reset
            _sync_index = 0;
        }

        // check to see if we have finished the sync word
        if (_sync_index >= LBMP_SYNC_LEN) {
            _sync_index = 0;

            // change state to now decoding
            _decode_state = LBMP_DECODE_HEADER;

            // DEBUG
            printf("[LBMP] head\n");

            // need to add the 4 sync bytes to the crc calculation
            for (int i = 0; i < 4; i ++) {
                _Crc32->add_byte(LBMP_SYNC[i]);
            }
        }

        break;

    /* Expecting Header */
    case LBMP_DECODE_HEADER:

        printf("%02X ", c);

        ret = payload_rx_add_block(c);  // add to buffer
        _Crc32->add_byte(c);			// add to checksum


        if (ret > 0) {
            // extract the message id and the payload length, still not guaranteed as checksum needs to be checked
            _rx_msg_id = _buf.header.msgId;
            _payload_length = _buf.header.payloadLength;

            // DEBUG
            printf("[LBMP] head checksum\n");

            // move to the header checksum decode state
            _decode_state = LBMP_DECODE_HEADER_CHECKSUM;
        }

        ret = 0;
        break;

    /* Expecting Header Checksum */
    case LBMP_DECODE_HEADER_CHECKSUM:

        printf("%02X ", c);

        ret = checksum_rx_add(c);	// add to incoming checksum buffer

        // check if completed the checksum bytes
        if (ret > 0) {

            // retrieve the calculated checksum
            uint32_t calculated_checksum = _Crc32->get_crc32();

            /* check checksum */
            if (calculated_checksum != _chk_buf.checksum) {	// error
                /* wait for next message */
                ret = 0;
                //decode_init();

                // DEBUG
                printf("[LBMP] checksum failed\n");
                printf("[LBMP] checksum: %08x\ncalculated: %08x\n", _chk_buf.checksum, calculated_checksum);

                decode_init();

            } else {	// match

                /* reinit checksum for payload */
                decode_checksum_reinit();

                /* init payload block */
                switch (_rx_msg_id) {

                /* Getting Extended Nav or Nav Solution */
                case LBMP_MSG_ID_EXT_NAV_SOL:
                case LBMP_MSG_ID_NAV_SOL:
                    decode_block_reinit(LBMP_NAV_SOL_BLK_LEN);	// first block is solution block

                    _nav_decode_state = LBMP_NAV_DECODE_SOL_BLK;	// set the nav decode state
                    break;

                }

                // DEBUG
                printf("[LBMP] payload\n");

                _decode_state = LBMP_DECODE_PAYLOAD;
            }
        }

        ret = 0;
        break;

    /* Expecting Payload */
    case LBMP_DECODE_PAYLOAD:

        _Crc32->add_byte(c);	// add byte to checksum

        /* each message unfortunately needs to be handled differently */
        switch (_rx_msg_id) {
        case LBMP_MSG_ID_EXT_NAV_SOL:
        case LBMP_MSG_ID_NAV_SOL:
            ret = payload_rx_add_nav(c);	// add a nav (ext and normal) solution byte
            break;

        default:
            // if don't know the message, just go back to waiting for sync
            decode_init();
            break;
        }

        // check if we have completed the payload
        if (ret > 0) {
            _decode_state = LBMP_DECODE_FOOTER;
        }

        ret = 0;
        break;

    /* Expecting Footer (payload checksum) */
    case LBMP_DECODE_FOOTER:

        ret = checksum_rx_add(c);	// add to the checksum payload buffer

        // check if completed the footer
        if (ret > 0) {

            // retrieve the calculated checksum
            uint32_t calculated_checksum = _Crc32->get_crc32();

            /* check checksum */
            if (calculated_checksum != _chk_buf.checksum) {	// error
                ret = 0;
            } else {	// match
                ret = payload_rx_done();	// finish payload
            }

            /* reset */
            decode_init();
        }

        break;
    }

    return ret;
}


int  // -1 = error, 0 = continue, 1 = completed
LBMP::payload_rx_add_block(const uint8_t c)
{
    int ret = 0;

    _buf.raw[_block_index] = c;

    if (--_block_index < 0) {
        ret = 1;  // paylaod completed
    }

    return ret;
}

int  // -1 = error, 0 = continue, 1 = completed
LBMP::checksum_rx_add(const uint8_t c)
{
    int ret = 0;

    _chk_buf.raw[_checksum_index] = c;

    if (--_checksum_index < 0) {
        ret = 1;  // received entire checksum
    }

    return ret;
}


int  // -1 = error, 0 = continue, 1 = completed
LBMP::payload_rx_add_nav(const uint8_t c)
{

    int ret = 0;
    int signal_count = 0;

    switch (_nav_decode_state) {

    case LBMP_NAV_DECODE_SOL_BLK:

        ret = payload_rx_add_block(c);  // add the byte to the block

        // have completed the block
        if (ret > 0) {

            // parse the information from the solution block
            _gps_position->lat = (int32_t) ((double)_buf.nav_sol_blk.lat / POW_2_38 * M_RAD_TO_DEG * 1.0e7);
            _gps_position->lon = (int32_t) ((double)_buf.nav_sol_blk.lon / POW_2_38 * M_RAD_TO_DEG * 1.0e7);
            _gps_position->alt_ellipsoid = (int32_t) ((float)_buf.nav_sol_blk.height / POW_2_15 * 1.0e3f);
            _gps_position->alt = _gps_position->alt_ellipsoid + 23.0*1.0e3; // TODO: need to add geoid height as constant

            _gps_position->cog_rad = (float)_buf.nav_sol_blk.heading / POW_2_7 * M_DEG_TO_RAD_F;
            _gps_position->vel_m_s = (float)_buf.nav_sol_blk.heading / POW_2_10;

            _gps_position->eph = (float)_buf.nav_sol_blk.hdop / POW_2_7;
            _gps_position->epv = (float)_buf.nav_sol_blk.vdop / POW_2_7;
            _gps_position->s_variance_m_s = (float)_buf.nav_sol_blk.velQual3d / POW_2_10;
            _gps_position->c_variance_rad = (float)_buf.nav_sol_blk.headingQual / POW_2_7 * M_DEG_TO_RAD_F;

            _gps_position->vel_ned_valid = true;
            _gps_position->fix_type = 3;//(uint8_t)_buf.nav_sol_blk.solutionStatus;
            _gps_position->satellites_used = _buf.nav_sol_blk.signalsUsed;

            // TODO: parse time block information

            // DEBUG
            printf("[LBMP] decoded position: e.g. lat = %d\n", _gps_position->lat);

            // if looking at the extended nav solution (right now that's the only one we have coming in)
            _nav_decode_state = LBMP_NAV_DECODE_EXT_BLK;
            decode_block_reinit(LBMP_NAV_EXT_BLK_LEN);
        }

        ret = 0;
        break;

    case LBMP_NAV_DECODE_EXT_BLK:

        ret = payload_rx_add_block(c);  // add the byte to the block

        if (ret > 0) {
            // parse block
            _gps_position->vel_n_m_s = (float)_buf.nav_ext_sol_blk.velN / POW_2_15;
            _gps_position->vel_e_m_s = (float)_buf.nav_ext_sol_blk.velE / POW_2_15;
            _gps_position->vel_d_m_s = -(float)_buf.nav_ext_sol_blk.velU / POW_2_15;
            //_gps_position->vel_m_s = sqrt(_gps_position->vel_n_m_s*_gps_position->vel_n_m_s + _gps_position->vel_e_m_s*_gps_position->vel_e_m_s  + _gps_position->vel_d_m_s*_gps_position->vel_d_m_s );

            // move to the new state
            _nav_decode_state = LBMP_NAV_DECODE_COUNT;
        }

        ret = 0;
        break;

    case LBMP_NAV_DECODE_COUNT:

        signal_count = (int) c;

        // now will need to decode all the signal details
        _nav_decode_state = LBMP_NAV_DECODE_SIG_BLK;
        decode_block_reinit(signal_count*4);
        break;

    case LBMP_NAV_DECODE_SIG_BLK:

        // right now do nothing

        break;
    }

    /* true return var is set based on full payload length */
    if (++_payload_index >= _payload_length) {
        ret = 1;
    } else {
        ret = 0;
    }

    return ret;
}

int  // 0 = no message handled, 1 = message handled, 2 = "sat" info message handled
LBMP::payload_rx_done(void)
{
    int ret = 0;

    // for all messages, the data has already been peeled out...

    // TODO: just need to add timestamp and return 1
    // add all the timestamps
    _gps_position->timestamp_time		= hrt_absolute_time();
    _gps_position->timestamp_velocity 	= hrt_absolute_time();
    _gps_position->timestamp_variance 	= hrt_absolute_time();
    _gps_position->timestamp_position	= hrt_absolute_time();

    // DEBUG
    printf("[LBMP} payload completed\n");

    ret = 1;
    return ret;
}
