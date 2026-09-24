/*
 * Copyright CogniPilot Foundation 2026
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>

#include "diag.h"

LOG_MODULE_REGISTER(log_sdcard_diag, LOG_LEVEL_WRN);

#define BREADCRUMB_MAGIC 0x4C617374
#define BREADCRUMB_MSG_LEN 48

static struct {
	uint32_t magic;
	char msg[BREADCRUMB_MSG_LEN];
} g_breadcrumb __attribute__((section(".noinit")));

void diag_breadcrumb(const char *msg)
{
	g_breadcrumb.magic = BREADCRUMB_MAGIC;
	strncpy(g_breadcrumb.msg, msg, BREADCRUMB_MSG_LEN - 1);
	g_breadcrumb.msg[BREADCRUMB_MSG_LEN - 1] = '\0';
}

static uint32_t g_usb_reset_flush_count;
static uint32_t g_usb_reset_flush_max;
static uint32_t g_usb_reset_flush_last;

void diag_usb_reset_flush_record(unsigned int iterations_used)
{
	g_usb_reset_flush_count++;
	g_usb_reset_flush_last = iterations_used;
	if (iterations_used > g_usb_reset_flush_max) {
		g_usb_reset_flush_max = iterations_used;
	}
	printk("*** usb ehci reset flush: used %u iters (call #%u, max so far %u) ***\n",
	       iterations_used, g_usb_reset_flush_count, g_usb_reset_flush_max);
}

static uint32_t g_usb_prime_clear_count;
static uint32_t g_usb_prime_clear_max;

void diag_usb_prime_clear_record(unsigned int iterations_used)
{
	g_usb_prime_clear_count++;
	if (iterations_used > g_usb_prime_clear_max) {
		g_usb_prime_clear_max = iterations_used;
	}
	/* This one can fire on every ordinary transfer completion, so only
	 * print when it actually spends a non-trivial number of iterations
	 * waiting - otherwise the 0-iteration common case would flood the
	 * console and become part of the problem being measured.
	 */
	if (iterations_used > 10) {
		printk("*** usb ehci prime clear: used %u iters (call #%u, max so far %u) ***\n",
		       iterations_used, g_usb_prime_clear_count, g_usb_prime_clear_max);
	}
}

static uint32_t g_usb_cancel_flush_count;
static uint32_t g_usb_cancel_flush_max;

void diag_usb_cancel_flush_record(unsigned int iterations_used)
{
	g_usb_cancel_flush_count++;
	if (iterations_used > g_usb_cancel_flush_max) {
		g_usb_cancel_flush_max = iterations_used;
	}
	printk("*** usb ehci cancel flush: used %u iters (call #%u, max so far %u) ***\n",
	       iterations_used, g_usb_cancel_flush_count, g_usb_cancel_flush_max);
}

static int diag_report_last_breadcrumb(void)
{
	if (g_breadcrumb.magic == BREADCRUMB_MAGIC) {
		printk("*** log_sdcard diag: last breadcrumb before reset: \"%s\" ***\n",
		       g_breadcrumb.msg);
	} else {
		printk("*** log_sdcard diag: no breadcrumb from a previous hang "
		       "(normal boot) ***\n");
	}
	g_breadcrumb.magic = 0;
	return 0;
}
SYS_INIT(diag_report_last_breadcrumb, PRE_KERNEL_1, 0);

/* Direct, sub-second stall detector: a timer tick fires every 20ms; if the
 * actual gap between two ticks is ever larger than expected, something
 * stalled the system for that long, measured directly rather than inferred
 * from gaps between console log messages (which are unreliable once the
 * console itself is flooded with high-rate warnings).
 */
#define HEARTBEAT_PERIOD_MS 20
#define HEARTBEAT_STALL_THRESHOLD_MS 60

static int64_t g_heartbeat_last_uptime;
static uint32_t g_heartbeat_stall_count;
static int64_t g_heartbeat_stall_max_ms;

static void heartbeat_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	int64_t now = k_uptime_get();
	int64_t delta = now - g_heartbeat_last_uptime;

	g_heartbeat_last_uptime = now;

	if (g_heartbeat_stall_count == 0 && delta > 1000) {
		/* first tick after boot, ignore startup skew */
		return;
	}

	if (delta > HEARTBEAT_STALL_THRESHOLD_MS) {
		g_heartbeat_stall_count++;
		if (delta > g_heartbeat_stall_max_ms) {
			g_heartbeat_stall_max_ms = delta;
		}
		printk("*** heartbeat stall: %lld ms gap at uptime %lld ms (expected ~%d) ***\n",
		       delta, now, HEARTBEAT_PERIOD_MS);
	}
}
static K_TIMER_DEFINE(g_heartbeat_timer, heartbeat_timer_handler, NULL);

static int diag_heartbeat_init(void)
{
	g_heartbeat_last_uptime = k_uptime_get();
	k_timer_start(&g_heartbeat_timer, K_MSEC(HEARTBEAT_PERIOD_MS), K_MSEC(HEARTBEAT_PERIOD_MS));
	return 0;
}
SYS_INIT(diag_heartbeat_init, APPLICATION, 98);

/* Thread-level canary: the heartbeat timer above runs in ISR context, so it
 * proves interrupts/the CPU aren't halted but is blind to a lower-priority
 * or preemptible thread being starved by a higher-priority/cooperative
 * thread that hogs the CPU without yielding (e.g. GNSS parsing on the
 * system workqueue at priority -2). This thread wakes via k_sleep, same as
 * actuate_sound (priority 4), so it experiences the same scheduling delays
 * a starved thread would.
 */
#define CANARY_PERIOD_MS 20
#define CANARY_STALL_THRESHOLD_MS 60
#define CANARY_PRIORITY 4

static uint32_t g_canary_stall_count;
static int64_t g_canary_stall_max_ms;

/* Directly finds which thread accumulated the most CPU time between two
 * canary ticks, using the same runtime-stats API "kernel thread list" uses -
 * measured in firmware, not inferred from noisy/interleaved console text.
 */
#define MAX_TRACKED_THREADS 32
struct thread_usage_entry {
	k_tid_t tid;
	uint64_t last_cycles;
};
static struct thread_usage_entry g_thread_usage[MAX_TRACKED_THREADS];
static int g_thread_usage_count;
static k_tid_t g_top_thread;
static uint64_t g_top_delta;

static struct thread_usage_entry *usage_find_or_add(k_tid_t tid)
{
	for (int i = 0; i < g_thread_usage_count; i++) {
		if (g_thread_usage[i].tid == tid) {
			return &g_thread_usage[i];
		}
	}
	if (g_thread_usage_count < MAX_TRACKED_THREADS) {
		g_thread_usage[g_thread_usage_count].tid = tid;
		g_thread_usage[g_thread_usage_count].last_cycles = 0;
		return &g_thread_usage[g_thread_usage_count++];
	}
	return NULL;
}

static void usage_scan_cb(const struct k_thread *thread, void *user_data)
{
	ARG_UNUSED(user_data);
	k_thread_runtime_stats_t stats;

	if (k_thread_runtime_stats_get((k_tid_t)thread, &stats) != 0) {
		return;
	}

	struct thread_usage_entry *e = usage_find_or_add((k_tid_t)thread);

	if (!e) {
		return;
	}

	uint64_t delta = stats.execution_cycles - e->last_cycles;

	e->last_cycles = stats.execution_cycles;

	if (delta > g_top_delta) {
		g_top_delta = delta;
		g_top_thread = (k_tid_t)thread;
	}
}

static void canary_thread_entry(void *p0, void *p1, void *p2)
{
	ARG_UNUSED(p0);
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	int64_t last = k_uptime_get();
	bool first = true;

	while (true) {
		k_sleep(K_MSEC(CANARY_PERIOD_MS));
		int64_t now = k_uptime_get();
		int64_t delta = now - last;

		last = now;

		g_top_delta = 0;
		g_top_thread = NULL;
		k_thread_foreach_unlocked(usage_scan_cb, NULL);

		if (first) {
			first = false;
			continue;
		}

		if (delta > CANARY_STALL_THRESHOLD_MS) {
			g_canary_stall_count++;
			if (delta > g_canary_stall_max_ms) {
				g_canary_stall_max_ms = delta;
			}
			printk("*** canary thread stall: %lld ms gap at uptime %lld ms "
			       "(expected ~%d), culprit=\"%s\" used %llu cycles ***\n",
			       delta, now, CANARY_PERIOD_MS,
			       g_top_thread ? k_thread_name_get(g_top_thread) : "?", g_top_delta);
		}
	}
}
K_THREAD_DEFINE(diag_canary, 1024, canary_thread_entry, NULL, NULL, NULL, CANARY_PRIORITY, 0, 100);

#define WDT_NODE DT_NODELABEL(wdog0)
#define WDT_TIMEOUT_MS 5000

static const struct device *const g_wdt_dev = DEVICE_DT_GET(WDT_NODE);
static int g_wdt_channel = -1;

static void wdt_feed_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	if (g_wdt_channel >= 0) {
		wdt_feed(g_wdt_dev, g_wdt_channel);
	}
}
static K_TIMER_DEFINE(g_wdt_feed_timer, wdt_feed_timer_handler, NULL);

static int diag_watchdog_init(void)
{
	if (!device_is_ready(g_wdt_dev)) {
		LOG_ERR("watchdog device not ready");
		return -ENODEV;
	}

	struct wdt_timeout_cfg cfg = {
		.window.min = 0,
		.window.max = WDT_TIMEOUT_MS,
		.flags = WDT_FLAG_RESET_SOC,
	};

	g_wdt_channel = wdt_install_timeout(g_wdt_dev, &cfg);
	if (g_wdt_channel < 0) {
		LOG_ERR("failed to install watchdog timeout: %d", g_wdt_channel);
		return g_wdt_channel;
	}

	int ret = wdt_setup(g_wdt_dev, 0);
	if (ret != 0) {
		LOG_ERR("failed to start watchdog: %d", ret);
		return ret;
	}

	k_timer_start(&g_wdt_feed_timer, K_MSEC(WDT_TIMEOUT_MS / 5), K_MSEC(WDT_TIMEOUT_MS / 5));

	LOG_WRN("watchdog armed, %d ms timeout", WDT_TIMEOUT_MS);
	return 0;
}
SYS_INIT(diag_watchdog_init, APPLICATION, 98);

static int diag_usb_cmd_handler(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "reset flush:  calls=%u max=%u last=%u", g_usb_reset_flush_count,
		    g_usb_reset_flush_max, g_usb_reset_flush_last);
	shell_print(sh, "prime clear:  calls=%u max=%u", g_usb_prime_clear_count,
		    g_usb_prime_clear_max);
	shell_print(sh, "cancel flush: calls=%u max=%u", g_usb_cancel_flush_count,
		    g_usb_cancel_flush_max);
	shell_print(sh, "heartbeat:    stalls=%u max=%lld ms", g_heartbeat_stall_count,
		    g_heartbeat_stall_max_ms);
	shell_print(sh, "canary:       stalls=%u max=%lld ms", g_canary_stall_count,
		    g_canary_stall_max_ms);
	return 0;
}
SHELL_CMD_REGISTER(diag_usb, NULL, "show EHCI retry-loop iteration counters", diag_usb_cmd_handler);
