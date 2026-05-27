/*
 * Public surface of app_main.c — currently a single helper.
 *
 * The DJI Osmo Action camera occasionally ignores a one-shot stop_record
 * over BLE (field-observed 2026-05-26). To make stop reliable, every code
 * path that sends `command_logic_stop_record()` must ALSO call
 * `app_request_pending_stop()` immediately afterwards. The main loop in
 * app_main.c then re-issues stop_record on a 1.5 s cadence until the
 * camera's status push reports `!is_camera_recording()`, or until a small
 * retry budget runs out (5 retries → ~7.5 s worst case).
 *
 * Asymmetric to start_record on purpose: a missed start is naturally
 * re-fired by the next motion event, but a missed stop leaves the camera
 * recording silently for a long time, which is the expensive failure mode
 * the user reported.
 */

#ifndef APP_MAIN_H
#define APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

void app_request_pending_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_H */
