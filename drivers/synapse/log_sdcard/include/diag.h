/*
 * Copyright CogniPilot Foundation 2026
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CEREBRI_LOG_SDCARD_DIAG_H
#define CEREBRI_LOG_SDCARD_DIAG_H

/* Isolated re-test: ONLY the watchdog + periodic feed timer (no HAL fix, no
 * CONFIG_SYMTAB) - the one remaining untested variable from the earlier
 * build that ran clean, to check whether it alone is what stops the hang.
 */
void diag_breadcrumb(const char *msg);

/* Records how many retry iterations the EHCI bus-reset EPFLUSH/EPPRIME wait
 * (usb_device_ehci.c, USB_DeviceEhciInterruptReset) actually consumed before
 * the bit cleared, to measure how close each bus reset comes to the bounded
 * retry limit - called directly from HAL/ISR context, no zephyr headers
 * pulled into the vendor file.
 */
void diag_usb_reset_flush_record(unsigned int iterations_used);

/* Same idea, for the other three bounded EHCI retry sites: prime-clear
 * (fires on ordinary transfer completion, InterruptTokenDone), and the
 * cancel-path flush loops (CancelControlPipe / Cancel, fire on endpoint
 * cancellation).
 */
void diag_usb_prime_clear_record(unsigned int iterations_used);
void diag_usb_cancel_flush_record(unsigned int iterations_used);

#endif
