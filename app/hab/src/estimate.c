/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdio.h>
#include <time.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <cerebri/core/perf_counter.h>
#include <cerebri/core/log_utils.h>

#include <synapse_topic_list.h>

#include <cerebri/core/casadi.h>

#include "app/hab/casadi/hab.h"

#define MY_STACK_SIZE 4096
#define MY_PRIORITY   4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

CEREBRI_NODE_LOG_INIT(hab_estimate, LOG_LEVEL_WRN);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

// private context
struct context {
	struct zros_node node;
	synapse_pb_Imu imu;
	synapse_pb_MagneticField mag;
	synapse_pb_NavSatFix nav_sat_fix;
	synapse_pb_Altimeter altimeter;
	synapse_pb_Odometry odometry;
	synapse_pb_NavSatFix nav_sat_fix_estimator;
	struct zros_sub sub_imu, sub_mag, sub_nav_sat_fix, sub_altimeter;
	struct zros_pub pub_odometry;
	struct zros_pub pub_nav_sat_fix_estimator;
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
	struct perf_counter perf;
};

// private initialization
static struct context g_ctx = {
	.node = {},
	.imu = synapse_pb_Imu_init_default,
	.mag = synapse_pb_MagneticField_init_default,
	.nav_sat_fix = synapse_pb_NavSatFix_init_default,
	.altimeter = synapse_pb_Altimeter_init_default,
	.odometry =
		{
			.child_frame_id = "base_link",
			.has_stamp = true,
			.stamp = synapse_pb_Timestamp_init_default,
			.frame_id = "odom",
			.has_pose = true,
			.has_twist = true,
			.pose.has_position = true,
			.pose.has_orientation = true,
			.twist.has_angular = true,
			.twist.has_linear = true,
		},
	.nav_sat_fix_estimator =
		{
			.has_stamp = true,
			.stamp = synapse_pb_Timestamp_init_default,
			.frame = synapse_pb_NavSatFix_Frame_WGS_84,
		},
	.sub_imu = {},
	.sub_mag = {},
	.sub_nav_sat_fix = {},
	.sub_altimeter = {},
	.pub_odometry = {},
	.pub_nav_sat_fix_estimator = {},
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
	.perf = {},
};

static void hab_estimate_init(struct context *ctx)
{
	zros_node_init(&ctx->node, "hab_estimate");
	zros_sub_init(&ctx->sub_imu, &ctx->node, &topic_imu, &ctx->imu, 1000);
	zros_sub_init(&ctx->sub_mag, &ctx->node, &topic_magnetic_field, &ctx->mag, 300);
	zros_sub_init(&ctx->sub_nav_sat_fix, &ctx->node, &topic_nav_sat_fix, &ctx->nav_sat_fix, 10);
	zros_sub_init(&ctx->sub_altimeter, &ctx->node, &topic_altimeter, &ctx->altimeter, 30);
	zros_pub_init(&ctx->pub_odometry, &ctx->node, &topic_odometry_estimator, &ctx->odometry);
	zros_pub_init(&ctx->pub_nav_sat_fix_estimator, &ctx->node, &topic_nav_sat_fix_estimator,
		      &ctx->nav_sat_fix_estimator);
	perf_counter_init(&ctx->perf, "estimator imu", 1.0 / 100);
	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
}

static void hab_estimate_fini(struct context *ctx)
{
	zros_sub_fini(&ctx->sub_imu);
	zros_sub_fini(&ctx->sub_mag);
	zros_sub_fini(&ctx->sub_nav_sat_fix);
	zros_sub_fini(&ctx->sub_altimeter);
	zros_pub_fini(&ctx->pub_odometry);
	zros_pub_fini(&ctx->pub_nav_sat_fix_estimator);
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
}

static void hab_estimate_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	int rc = 0;

	hab_estimate_init(ctx);

	// variables
	struct k_poll_event events[1] = {};

	// wait for imu
	LOG_DBG("waiting for imu");
	events[0] = *zros_sub_get_event(&ctx->sub_imu);
	rc = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
	if (rc != 0) {
		LOG_DBG("did not receive imu");
		return;
	}
	zros_sub_update(&ctx->sub_imu);

	double dt = 0;
	int64_t ticks_last = k_uptime_ticks();

	// Constants
	// TODO: magnetic declination is location-specific - update for the
	// actual launch site before flight, this value is a placeholder
	// carried over from ground-vehicle testing.
	// static const double decl_WL = -4.494167 / 180 * M_PI; // West Lafayette (old placeholder)
	static const double decl_WL = 4.38 / 180 * M_PI; // Odense, Denmark (NOAA WMM2025)
	static const double accel_gain = CONFIG_CEREBRI_HAB_ATTITUDE_EST_ACCEL_GAIN * 1e-3;
	static const double mag_gain = CONFIG_CEREBRI_HAB_ATTITUDE_EST_MAG_GAIN * 1e-3;

	// process noise for the constant-velocity position/velocity model -
	// q_vel controls how quickly estimated velocity is allowed to drift
	// between GPS/baro corrections (ascent rate changes, wind gusts);
	// placeholder values, need tuning against real sensor/flight data.
	static const double q_pos = 1e-4;
	static const double q_vel = 1e-2;

	// measurement noise variances - placeholders, need tuning from real
	// GPS/baro sensor characteristics, not copied from any vehicle-
	// specific source.
	static const double R_gps = 25.0;  // (m)^2, ~5 m horizontal accuracy
	static const double R_baro = 1.0;  // (m)^2, ~1 m vertical accuracy

	// local ENU origin, set from the first valid GPS fix
	bool origin_set = false;
	double lat0_rad = 0, lon0_rad = 0, cos_lat0 = 1;
	static const double R_EARTH = 6371000.0;

	// ------ Initialize attitude from magnetometer (yaw only) ------

	double q[4] = {1, 0, 0, 0};

	while (!zros_sub_update_available(&ctx->sub_mag)) {
		LOG_INF("waiting for magnetometer");
		k_sleep(K_MSEC(50));
	}
	zros_sub_update(&ctx->sub_mag);
	double mag_norm = ctx->mag.magnetic_field.x * ctx->mag.magnetic_field.x +
			  ctx->mag.magnetic_field.y * ctx->mag.magnetic_field.y +
			  ctx->mag.magnetic_field.z * ctx->mag.magnetic_field.z;
	while (mag_norm < 1e-4) {
		mag_norm = ctx->mag.magnetic_field.x * ctx->mag.magnetic_field.x +
			   ctx->mag.magnetic_field.y * ctx->mag.magnetic_field.y +
			   ctx->mag.magnetic_field.z * ctx->mag.magnetic_field.z;
		LOG_INF("magnetometer is not valid, waiting for valid data: %f", mag_norm);
		k_sleep(K_MSEC(50));
	}

	{
		CASADI_FUNC_ARGS(yaw_init)

		double mag[3] = {ctx->mag.magnetic_field.x, ctx->mag.magnetic_field.y,
				 ctx->mag.magnetic_field.z};
		args[0] = mag;
		args[1] = &decl_WL;

		res[0] = q;

		CASADI_FUNC_CALL(yaw_init)
	}

	// position/velocity estimator state: [x, y, z, vx, vy, vz]
	double x[6] = {0, 0, 0, 0, 0, 0};

	// position/velocity covariance (6x6)
	double P_pos[36] = {0};
	for (int i = 0; i < 6; i++) {
		P_pos[i * 6 + i] = 1e2; // large initial uncertainty, pulled in by first corrections
	}

	// attitude covariance - not actively used (pass-through in
	// attitude_estimator), kept only for interface compatibility
	double P_att[36] = {0};

	// poll on imu
	events[0] = *zros_sub_get_event(&ctx->sub_imu);

	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {

		// poll for imu
		rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
		if (rc != 0) {
			LOG_DBG_RATELIMIT_RATE(30000, "not receiving imu");
			continue;
		}

		if (zros_sub_update_available(&ctx->sub_imu)) {
			zros_sub_update(&ctx->sub_imu);
			perf_counter_update(&ctx->perf);
		}

		if (zros_sub_update_available(&ctx->sub_mag)) {
			zros_sub_update(&ctx->sub_mag);
		}

		// calculate dt
		int64_t ticks_now = k_uptime_ticks();
		dt = (double)(ticks_now - ticks_last) / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
		ticks_last = ticks_now;
		if (dt <= 0 || dt > 0.5) {
			LOG_WRN_RATELIMIT_RATE(30000, "imu update rate too low");
			continue;
		}

		// ---- propagate position/velocity (constant-velocity model,
		// no accelerometer - see hab.py's derive_position_velocity_
		// propagate docstring for why) ----
		{
			CASADI_FUNC_ARGS(position_velocity_propagate)

			args[0] = x;
			args[1] = P_pos;
			args[2] = &dt;
			args[3] = &q_pos;
			args[4] = &q_vel;

			res[0] = x;
			res[1] = P_pos;

			CASADI_FUNC_CALL(position_velocity_propagate)
		}

		// ---- propagate attitude from gyro ----
		{
			CASADI_FUNC_ARGS(attitude_propagate)

			double omega_b[3] = {ctx->imu.angular_velocity.x,
					     ctx->imu.angular_velocity.y,
					     ctx->imu.angular_velocity.z};
			args[0] = q;
			args[1] = omega_b;
			args[2] = &dt;

			res[0] = q;

			CASADI_FUNC_CALL(attitude_propagate)
		}

		// ---- correct attitude from accel/mag ----
		{
			CASADI_FUNC_ARGS(attitude_estimator)

			double a_b[3] = {ctx->imu.linear_acceleration.x,
					 ctx->imu.linear_acceleration.y,
					 ctx->imu.linear_acceleration.z};
			double omega_b[3] = {ctx->imu.angular_velocity.x,
					     ctx->imu.angular_velocity.y,
					     ctx->imu.angular_velocity.z};
			double mag[3] = {ctx->mag.magnetic_field.x, ctx->mag.magnetic_field.y,
					 ctx->mag.magnetic_field.z};

			args[0] = q;
			args[1] = mag;
			args[2] = &decl_WL;
			args[3] = omega_b;
			args[4] = a_b;
			args[5] = &accel_gain;
			args[6] = &mag_gain;
			args[7] = &dt;
			args[8] = P_att;

			res[0] = q;
			res[1] = P_att;
			CASADI_FUNC_CALL(attitude_estimator)
		}

		// ---- correct vertical position from baro ----
		if (zros_sub_update_available(&ctx->sub_altimeter)) {
			zros_sub_update(&ctx->sub_altimeter);

			CASADI_FUNC_ARGS(scalar_correction)

			double H[6] = {0, 0, 1, 0, 0, 0};
			double z = ctx->altimeter.vertical_position;

			args[0] = x;
			args[1] = P_pos;
			args[2] = H;
			args[3] = &z;
			args[4] = &R_baro;

			res[0] = x;
			res[1] = P_pos;

			CASADI_FUNC_CALL(scalar_correction)
		}

		// ---- correct horizontal position from GPS ----
		if (zros_sub_update_available(&ctx->sub_nav_sat_fix)) {
			zros_sub_update(&ctx->sub_nav_sat_fix);

			double lat_rad = ctx->nav_sat_fix.latitude * M_PI / 180.0;
			double lon_rad = ctx->nav_sat_fix.longitude * M_PI / 180.0;

			if (!origin_set) {
				lat0_rad = lat_rad;
				lon0_rad = lon_rad;
				cos_lat0 = cos(lat0_rad);
				origin_set = true;
				LOG_INF("gps origin set: lat %f lon %f", ctx->nav_sat_fix.latitude,
					ctx->nav_sat_fix.longitude);
			} else {
				// local ENU, flat-earth approximation about the origin -
				// fine for the horizontal extent of a single flight, not
				// meant for long-range dead reckoning.
				double east = (lon_rad - lon0_rad) * cos_lat0 * R_EARTH;
				double north = (lat_rad - lat0_rad) * R_EARTH;

				{
					CASADI_FUNC_ARGS(scalar_correction)

					double H[6] = {1, 0, 0, 0, 0, 0};

					args[0] = x;
					args[1] = P_pos;
					args[2] = H;
					args[3] = &east;
					args[4] = &R_gps;

					res[0] = x;
					res[1] = P_pos;

					CASADI_FUNC_CALL(scalar_correction)
				}
				{
					CASADI_FUNC_ARGS(scalar_correction)

					double H[6] = {0, 1, 0, 0, 0, 0};

					args[0] = x;
					args[1] = P_pos;
					args[2] = H;
					args[3] = &north;
					args[4] = &R_gps;

					res[0] = x;
					res[1] = P_pos;

					CASADI_FUNC_CALL(scalar_correction)
				}
			}
		}

		double v_b[3]; // velocity in body frame
		double v_w[3] = {x[3], x[4], x[5]}; // velocity in world frame

		{
			// rotate_vector_w_to_b:(q[4],v_w[3])->(v_b[3])
			CASADI_FUNC_ARGS(rotate_vector_w_to_b)

			args[0] = q;
			args[1] = v_w;

			res[0] = v_b;

			CASADI_FUNC_CALL(rotate_vector_w_to_b)
		}

		bool data_ok = true;
		for (int i = 0; i < 6; i++) {
			if (!isfinite(x[i])) {
				LOG_ERR("x[%d] is not finite", i);
				// TODO reinitialize
				x[i] = 0;
				data_ok = false;
				break;
			}
		}
		for (int i = 0; i < 4; i++) {
			if (!isfinite(q[i])) {
				LOG_ERR("q[%d] is not finite", i);
				q[0] = 1;
				q[1] = 0;
				q[2] = 0;
				q[3] = 0;
				data_ok = false;
				break;
			}
		}

		// publish odometry
		if (data_ok) {
			stamp_msg(&ctx->odometry.stamp, k_uptime_ticks());
			ctx->odometry.pose.position.x = x[0];
			ctx->odometry.pose.position.y = x[1];
			ctx->odometry.pose.position.z = x[2];
			ctx->odometry.twist.linear.x = v_b[0];
			ctx->odometry.twist.linear.y = v_b[1];
			ctx->odometry.twist.linear.z = v_b[2];
			ctx->odometry.pose.orientation.w = q[0];
			ctx->odometry.pose.orientation.x = q[1];
			ctx->odometry.pose.orientation.y = q[2];
			ctx->odometry.pose.orientation.z = q[3];
			ctx->odometry.twist.angular.x = ctx->imu.angular_velocity.x;
			ctx->odometry.twist.angular.y = ctx->imu.angular_velocity.y;
			ctx->odometry.twist.angular.z = ctx->imu.angular_velocity.z;

			// check quaternion normal
			__ASSERT(fabs((ctx->odometry.pose.orientation.w *
					       ctx->odometry.pose.orientation.w +
				       ctx->odometry.pose.orientation.x *
					       ctx->odometry.pose.orientation.x +
				       ctx->odometry.pose.orientation.y *
					       ctx->odometry.pose.orientation.y +
				       ctx->odometry.pose.orientation.z *
					       ctx->odometry.pose.orientation.z) -
				      1) < 1e-2,
				 "quaternion normal error");
			zros_pub_update(&ctx->pub_odometry);

			// publish global position - the actual output the balloon
			// controller wants, reconstructed from the filter's internal
			// local-ENU state by inverting the same flat-earth conversion
			// used above to turn GPS fixes into local corrections. Only
			// meaningful once a GPS fix has set the local origin.
			if (origin_set) {
				double lat_rad = lat0_rad + x[1] / R_EARTH;
				double lon_rad = lon0_rad + x[0] / (cos_lat0 * R_EARTH);

				stamp_msg(&ctx->nav_sat_fix_estimator.stamp, k_uptime_ticks());
				ctx->nav_sat_fix_estimator.latitude = lat_rad * 180.0 / M_PI;
				ctx->nav_sat_fix_estimator.longitude = lon_rad * 180.0 / M_PI;
				// NOTE: altitude here is the same barometric pressure
				// altitude as odometry.pose.position.z (fixed-101.325kPa
				// reference), not WGS84/MSL altitude - do not mix with
				// raw GPS altitude without accounting for that difference.
				ctx->nav_sat_fix_estimator.altitude = x[2];
				zros_pub_update(&ctx->pub_nav_sat_fix_estimator);
			}
		}
	}

	hab_estimate_fini(ctx);
}

static int start(struct context *ctx)
{
	k_tid_t tid =
		k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				hab_estimate_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "hab_estimate");
	k_thread_start(tid);
	return 0;
}

static int hab_estimate_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
{
	ARG_UNUSED(argc);
	struct context *ctx = data;

	if (strcmp(argv[0], "start") == 0) {
		if (k_sem_count_get(&g_ctx.running) == 0) {
			shell_print(sh, "already running");
		} else {
			start(ctx);
		}
	} else if (strcmp(argv[0], "stop") == 0) {
		if (k_sem_count_get(&g_ctx.running) == 0) {
			k_sem_give(&g_ctx.running);
		} else {
			shell_print(sh, "not running");
		}
	} else if (strcmp(argv[0], "status") == 0) {
		shell_print(sh, "running: %d", (int)k_sem_count_get(&g_ctx.running) == 0);
	}
	return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_hab_estimate, hab_estimate_cmd_handler, (start, &g_ctx, "start"),
			     (stop, &g_ctx, "stop"), (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(hab_estimate, &sub_hab_estimate, "hab estimate commands", NULL);

static int hab_estimate_sys_init(void)
{
	return start(&g_ctx);
};

SYS_INIT(hab_estimate_sys_init, APPLICATION, 1);

// vi: ts=4 sw=4 et
