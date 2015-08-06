/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/actuator_armed.h>

#include <px4_config.h>
#include <nuttx/sched.h>
 
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

static int error_counter = 0;
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

__EXPORT int px4_simple_app_main(int argc, char *argv[]);


/**
 * Mainloop of daemon.
 */
int px4_daemon_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}


int px4_simple_app_main(int argc, char *argv[])
{
if (argc < 2) {
		usage("missing command");
		return 1;
	}
 
	if (!strcmp(argv[1], "start")) {
 
		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}
 
		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("daemon",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_DEFAULT,
					     2000,
					     px4_daemon_thread_main,
					     (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
		return 0;
	}
 
	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}
 
	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");
 
		} else {
			warnx("\tnot started\n");
		}
 
		return 0;
	}
 
	usage("unrecognized command");
	return 1;
}

int px4_daemon_thread_main(int argc, char *argv[]){

		while (!thread_should_exit) {
		printf("Hello Sky!\n");

		/* subscribe to sensor_combined topic */
		int cmd_sub_fd = orb_subscribe(ORB_ID(vehicle_command));
		orb_set_interval(cmd_sub_fd, 1000);

		/* advertise attitude topic */
		/*struct actuator_armed_s cmd_armed;
		memset(&cmd_armed, 0, sizeof(cmd_armed));
		orb_advert_t cmd_armed_pub_fd = orb_advertise(ORB_ID(actuator_armed), &cmd_armed);*/

		struct actuator_armed_s cmd_p;
		memset(&cmd_p, 0, sizeof(cmd_p));
		orb_advert_t cmd_p_pub_fd = orb_advertise(ORB_ID(vehicle_command), &cmd_p);

		struct vehicle_command_s cmd;
		/* copy sensors raw data into local buffer */
		orb_copy(ORB_ID(vehicle_command), cmd_sub_fd, &cmd);

		if(error_counter >= 2){

		cmd.command = VEHICLE_CMD_DO_SET_MODE;
		cmd.param1 = 1.0;

		printf("[px4_simple_app] parameter:\t%8.4f\t%8.4f\t%8.4f\n",
		       (double)cmd.param1,
	    	   (double)cmd.param2,
	    	   (double)cmd.command);

		/* set att and publish this information for other apps */
		/*cmd_armed.  = cmd.param1;
		cmd_armed.  = cmd.param2;
		cmd_armed.  = cmd.command;*/
		/*orb_publish(ORB_ID(actuator_armed), cmd_armed_pub_fd, &cmd_armed);*/
		orb_publish(ORB_ID(vehicle_command), cmd_p_pub_fd, &cmd_p);

	}else
	{
		printf("not armed\n");
		error_counter++;
	}
	

	/* there could be more file descriptors here, in the form like:*/
	/* if (fds[1..n].revents & POLLIN) {}*/
		sleep(10);
	}
		return 0;
}
