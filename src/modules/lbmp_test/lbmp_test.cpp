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
 * @file lbmp_main.cpp
 *
 * Testing script for the locata protocol.
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
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h> 		/*< for conversion from local x,y to lat, lon */

#include <drivers/gps/lbmp.h>
#include <drivers/gps/gps_helper.h>

#include <platforms/px4_defines.h>



extern "C" __EXPORT int lbmp_test_main(int argc, char *argv[]);



vehicle_gps_position_s _report_gps_pos;


// the main script
int lbmp_test_main(int argc, char **argv) {

    printf("starting...\n");

    // make sure this is all empty
    memset(&_report_gps_pos, 0, sizeof(_report_gps_pos));


    char* uart_name = (char*) "/dev/ttyS6";
    unsigned timeout = 1000;

    // retrieve the user input (serial port)
    for (int i = 1; i < argc; i++) {

        /* UART device ID */
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
            if (argc > i + 1) {
                uart_name = argv[i + 1];
            }
        }

        /* timeout interval */
        if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--timeout") == 0) {
            if (argc > i + 1) {
                timeout = (unsigned) atoi(argv[i + 1]);
            }
        }
    }

    // connect to the serial port
    unsigned baudrate = 115200;
    int serial_fd = ::open(uart_name, O_RDWR);

    if (serial_fd < 0) {
        printf("faled to open serial port\n");
        return -1;
    }

    // create the LBMP parser
    GPS_Helper *_Helper = new LBMP(serial_fd, &_report_gps_pos);

    int helper_ret;
    if (_Helper->configure(baudrate) == 0) {
        printf("configuration successful\n");

        // do 1 receive

       helper_ret = _Helper->receive(timeout);

       if (helper_ret <= 0) {
           printf("returned error\n");
       } else {
           printf("returned data received\n");
       }

    } else {
        printf("configuration error\n");
    }

    return -1;
}
