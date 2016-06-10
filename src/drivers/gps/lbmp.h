/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @author Adrien Perkins <adrienp@stanford.edu>
 */

#ifndef LBMP_H
#define LBMP_H

#include "devices/src/gps_helper.h"
#include "crc32.h"

/* baudrate - for now fixed */
#define LBMP_BAUDRATE 57600

/* Locata message IDs */
#define LBMP_MSG_ID_STD_MEAS 		0x1001
#define LBMP_MSG_ID_EXT_MEAS 		0x1003
#define LBMP_MSG_ID_CP_MEAS 		0x1007
#define LBMP_MSG_ID_LITE_STATUS 	0x2001
#define LBMP_MSG_ID_ROV_STATUS 		0x2003
#define LBMP_MSG_ID_TLOC_STATUS 	0x2005
#define LBMP_MSG_ID_TIME_PPS 		0x3001
#define LBMP_MSG_ID_GPS_IN 			0x5001
#define LBMP_MSG_ID_NAV_SOL 		0x6001
#define LBMP_MSG_ID_EXT_NAV_SOL 	0x6002
#define LBMP_MSG_ID_SELF_SURV 		0x7001
#define LBMP_MSG_ID_SIG_DET 		0x8001
#define LBMP_MSG_ID_NET_COORD 		0x9001
#define LBMP_MSG_ID_NET_HEALTH 		0x9002
#define LBMP_MSG_ID_NET_DATA 		0x9003
#define LBMP_MSG_ID_ALMANAC 		0xA001
#define LBMP_MSG_ID_ACK 			0xAC01
#define LBMP_MSG_ID_ETH_CONFIG 		0xB002
#define LBMP_MSG_ID_REQ_OUT 		0xC001
#define LBMP_MSG_ID_CUSTOM 			0xCC00
#define LBMP_MSG_ID_SYS_REBOOT		0xCC01


/* useful Locata element lengths - needed because filling buffer backwards */
#define LBMP_SYNC_LEN			4
#define LBMP_HEADER_LEN 		16
#define LBMP_CHECKSUM_LEN 		4
#define LBMP_NAV_SOL_BLK_LEN	195
#define LBMP_NAV_EXT_BLK_LEN	124
#define LBMP_NAV_SIG_BLK_LEN 	4

/* sync word */
const uint8_t LBMP_SYNC[4] = {0x4f, 0xb0, 0xa2, 0x95};


// TODO: put message definitions here...

/** the structures of the binary packets */
#pragma pack(push, 1)

// note all blocks are reversed, as data is sent as Big Endian and Nuttx is Little Endian
// therefore will full buffer up backwards

 /* truncated Header message REVERSED */
typedef struct {
    uint8_t reserved[7];
    uint16_t reqSeqNum;
    uint16_t seqNum;
    uint16_t payloadLength;
    uint16_t msgId;
    uint8_t version;

} lbmp_header_t;

/* high precision timestamp block REVERSED */
typedef struct {
    uint32_t towPs; /* ps since start of ms */
    uint32_t towMs; /* ms since start of week */
    uint16_t week;	/* week number (GPS-IS-200D) */
    uint8_t flags;	/* bitfield */
    uint8_t source; /* enum */
} lbmp_hpt_t;


/* low precision timestamp block REVERSED */
typedef struct {
    uint32_t towMs;	/* ms since start of week */
    uint16_t week;	/* week number (GPS-IS-200D) */
} lbmp_lpt_t;


/* nav solution block (nav and ext nav msgs) REVERSED */
typedef struct {
    uint8_t reserved;
    uint32_t vdopAngular;	/* note: all dop units are [-]/2^7 */
    uint32_t hdopAngular;
    uint32_t tdop;
    uint32_t pdop;
    uint32_t hdop;
    uint32_t vdop;
    uint32_t ndop;
    uint32_t edop;
    uint8_t yawQual;		/* yaw (azimuth) quality metric, RMS (95%), [degree] */
    uint8_t pitchQual;		/* pitch (elevation) */
    uint8_t rollQual;
    uint32_t timeQual;		/* time quality uncertainty metric, RMS (95%), [ns] */
    uint32_t headingQual;	/* heading (cog) quality metric, RMS (95%), [deg] / 2^7 */
    uint32_t velQualU;		/* velocity quality metric, RMS (95%), [m/s] / 2^10 */
    uint32_t velQualN;
    uint32_t velQualE;
    uint32_t velQual3d;
    uint32_t posQualU;		/* position quality metric, RMS (95%), [m] / 2^10 */
    uint32_t posQualN;
    uint32_t posQualE;
    uint32_t posQual3d;
    int32_t  clockDrift;	/* common clock drift detected in the measurements for this solution, [m/s] / 2^10 */
    int64_t clockOffset;	/* common clock error detected in the emasurements for this solution, [m] / 2^10 */
    int32_t yaw;			/* receiver attitde about up axis, [deg] / 2^10, +- 180.00 */
    int32_t pitch;			/* receiver attitde about east axis, [deg] / 2^10, +- 180.00 */
    int32_t roll;			/* receiver attitde about north axis, [deg] / 2^10, +- 180.00 */
    int32_t heading;		/* course overground, [deg] / 2^7, +- 180.00 */
    uint32_t sog;			/* speed over ground, [m/s] / 2^10 */
    int64_t z;				/* ECEF Z position coordinate, [m] / 2^15 */
    int64_t y;
    int64_t x;
    int64_t height;			/* Ellipsoidal height, WGS-84, [m] / 2^15 */
    int64_t lon;			/* Longitude, WGS-84 +East, [rad] / 2^36, +-pi */
    int64_t lat;			/* Latitude, WGS-84 +North, [rad] / 2^36, +-pi/2 */
    uint8_t litesUsed;		/* number of LocataLites used in the solution */
    uint8_t litesTracked;	/* number of LocataLites being tracked by the device */
    uint8_t litesKnown;		/* number of LocataLites known to the decive */
    uint8_t signalsUsed;	/* number of LocataLite signals used in the solution */
    uint8_t signalsTracked;
    uint8_t signalsKnown;
    lbmp_lpt_t timeLastExtInit;	/* timestamp of the last time the solution was initialized from an external source.  Only valid when solution flag indicates external source used. */
    uint32_t solutionStatus;	/* status flags pertaining to the solution, bitfield */
    uint16_t ambiguityStatus;	/* indicates what carrier cycle ambiguities have been resolved, bitfield */
    uint8_t initType;			/* indicates how carrier phase measurements were initialized, enum */
    uint8_t measType;			/* indicates measurement types used, bitfield */
    uint16_t algo;				/* indicates nav algo used, bitfield */
    uint16_t config;			/* indicates nav config requested, bitfield */
    uint8_t validity;			/* indicates nav solution success, or reason for failure, enum */
    uint16_t latency;			/* latency of the solution output with respoect to the time of solution, [ms] */
    lbmp_hpt_t timeSol;			/* time of the measurments used in the solution */
} lbmp_soln_blk_t;


/* extended navigation solution block REVERSED */
typedef struct {
    uint32_t latency4;	/* latency between detecting measurement boundary and the nav thread finishing a fix, [ms] / 2^10 */
    uint32_t latency3;	/* latency between detecting measurement boundary and the nav thread starting to execute, [ms] / 2^10 */
    uint32_t latency2;	/* latency between detecting measurement boundary and notfiyng nav thread meas available, [ms] / 2^10 */
    uint32_t latency1;	/* latency between detecting measurement boundary and starting to build meas data set, [ms] / 2^10 */
    uint8_t reserved;
    uint8_t iodt;		/* issue of data counter for the time components of the solution */
    uint8_t externPosOffsetValid;	/* offset from external position valid, [1,0] */
    int32_t externPosOffsetVert;	/* vertical offset from external position, [m] / 2^10 */
    int32_t externPosOffsetHoriz;
    int32_t externPosOffset3d;
    int32_t angAccelQual;		/* angular acel quality, [deg/s/s] / 2^20 */
    int32_t yawAccel;			/* yaw acceleration, [deg/s/s] / 2^10 */
    int32_t pitchAccel;
    int32_t rollAccel;
    int32_t angleRateQual;		/* angular rate quality, [deg/s] / 2^20 */
    int32_t yawRate;			/* yaw rate, [deg/s] / 2^10 */
    int32_t pitchRate;
    int32_t rollRate;
    int32_t orientationQual;	/* oreitnation quality metric, [deg] / 2^20 */
    int32_t accelQual;			/* acceleration quality metric, [m/s/s] / 2^20 */
    int32_t vertAccel;			/* vertical acceleration, [m/s/s] / 2^15 */
    int32_t northingAccel;
    int32_t eastingAccel;
    int32_t velU;			/* vertical velocity, [m/s] / 2^15 */
    int32_t velN;
    int32_t velE;
    uint32_t ambiguityUncertainty;	/* overall qualit5y of estimated ambiguities (95%), [cycles] / 2^10 */
    uint32_t ambiguityRatio;		/* ratio of the best set of ambiguities to the next best set, [-] / 2^20 */
    uint32_t ambiguitySuccessRate;	/* theoretical ambiguity success rate based on model assumptions (related to ADOP), [%] / 2^7 */
    uint32_t adop;					/* ambiguitiy diluation of precision, [-] / 2^20 */
    uint32_t unitVariance;			/* sum of squares of weighted residuals divided by redundancy, [-] / 2^20 */
    uint8_t itersUsed;				/* number of iterations the solution ran before converging */
    uint32_t vertWeight;			/* relevant if bit-7 in sol flag is set, will indicate the vertial weighting number applied computing the 3D solution */
    uint32_t tropoScaleFactor;		/* scaling factor to adjust the recommended tropospheric model to minimize residual error */
} lbmp_ext_blk_t;


/* signals used block REVERSED */
typedef struct {
    uint32_t quality;
    uint8_t used;
    uint8_t txChannelNum;
    uint8_t liteId;
    uint8_t netId;
} lbmp_sig_blk_t;



/* messages and payload buffer */
typedef union {
    lbmp_header_t header;
    lbmp_soln_blk_t nav_sol_blk;
    lbmp_ext_blk_t nav_ext_sol_blk;
    lbmp_sig_blk_t mav_sig_used_element;
    uint8_t raw[];
} lbmp_buf_t;


/* checksum buffer */
typedef union {
    uint32_t checksum;
    uint8_t raw[];
} checksum_t;


#pragma pack(pop)


// some enums

/* decoding state */
typedef enum {
    LBMP_DECODE_SYNC = 0,
    LBMP_DECODE_HEADER,
    LBMP_DECODE_HEADER_CHECKSUM,
    LBMP_DECODE_PAYLOAD,
    LBMP_DECODE_FOOTER
} lbmp_decode_state_t;


/* nav message decoding state - since message is made up of blocks, decoding one block at a time */
typedef enum {
    LBMP_NAV_DECODE_SOL_BLK = 0,
    LBMP_NAV_DECODE_EXT_BLK,
    LBMP_NAV_DECODE_COUNT,
    LBMP_NAV_DECODE_SIG_BLK
} lbmp_nav_decode_state_t;

/* the different decode stages that can be initialized */
typedef enum {
    LBMP_STAGE_PAYLOAD = 0,
    LBMP_STAGE_BLOCK,
    LBMP_STAGE_CHECKSUM
} lbmp_decode_stage_t;






class GPSDriverLBMP : public GPSHelper
{
public:
    GPSDriverLBMP(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position);
    virtual ~GPSDriverLBMP();
    int				receive(unsigned timeout);
    int				configure(unsigned &baudrate, OutputMode output_mode);

    /**
     * Parse the binary LBMP packet
     */
    int         parse_char(const uint8_t c);

private:

    /**
     * Reset the parse state machine for a fresh start
     */
    void		decode_init();							// new message
    void		decode_block_reinit(const int size);	// new block
    void		decode_checksum_reinit();				// new checksum


    /**
     * Start payload rx
     */
    int			payload_rx_init(void);

    /**
     * Add payload rx byte
     */
    int 		payload_rx_add_block(const uint8_t c);
    int 		payload_rx_add_nav(const uint8_t c);
    int 		checksum_rx_add(const uint8_t c);

    /**
     * Finish payload rx
     */
    int			payload_rx_done(void);


    int			_fd;
    struct vehicle_gps_position_s *_gps_position;

    /* states */
    lbmp_decode_state_t		_decode_state;
    lbmp_nav_decode_state_t _nav_decode_state;

    /* indices */
    uint16_t		_payload_index;
    int16_t			_block_index;
    int16_t			_checksum_index;
    uint8_t			_sync_index;

    /* lengths */
    uint16_t		_payload_length;

    /* payload information */
    uint16_t		_rx_msg_id;

    /* buffers */
    lbmp_buf_t		_buf;
    checksum_t 		_chk_buf;

    /* checksum handling */
    CRC32*			_Crc32;

};

#endif // LBMP_H
