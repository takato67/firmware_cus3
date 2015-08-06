#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	printf("Hello Sky!\n");

	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	orb_set_interval(sensor_sub_fd, 1000);

	/* advertise attitude topic */
	/*struct vehicle_attitude_s att;*/
	//memset(&att, 0, sizeof(att));
	/*orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);*/

	/* one could wait for multiple topics with this technique, just using one here */
	struct pollfd fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 5; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = poll(fds, 1, 1000);
	 
		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			printf("[px4_simple_app] Got no data within a second\n");
		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[px4_simple_app] ERROR return value from poll(): %d\n"
					, poll_ret);
			}
			error_counter++;
		} else {
	 
			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				printf("[px4_simple_app] Accelerometer:\t%8.4f\t%8.4f\t%8.4f\n",
					(double)raw.adc_voltage_v[0],
					(double)raw.adc_voltage_v[1],
					(double)raw.adc_voltage_v[2],
					(double)raw.adc_voltage_v[3],
					(double)raw.adc_voltage_v[4],
					(double)raw.adc_voltage_v[5],
					(double)raw.adc_voltage_v[6],
					(double)raw.adc_voltage_v[7],
					(double)raw.adc_voltage_v[8],
					(double)raw.adc_voltage_v[9]);

				/* set att and publish this information for other apps */
				/*att.roll = raw.adc_voltage_v[0];
				att.pitch = raw.adc_voltage_v[1];
				att.yaw = raw.adc_voltage_v[2];
				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);*/
				
			}
			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	return 0;
}