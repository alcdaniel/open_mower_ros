#!/usr/bin/env python3
"""
ntrip_watchdog.py

Manages the ntrip_client lifecycle gracefully:
  - If credentials are missing/placeholder → stays alive, logs warning, never crashes.
  - If credentials are set → spawns ntrip_ros.py as a subprocess and monitors it.
  - If the subprocess dies quickly (< FAST_FAIL_THRESHOLD_S) more than MAX_FAST_FAILS
    times in a row → pauses for BACKOFF_S before retrying.
  - OM_USE_NTRIP and the config file are never modified.
"""

import os
import subprocess
import time

import rospy


MAX_FAST_FAILS       = 3
FAST_FAIL_THRESHOLD_S = 15.0   # seconds — faster death = credential/network error
BACKOFF_S            = 600.0   # 10 min pause after too many fast fails


def _ntrip_script_path():
    try:
        pkg = subprocess.check_output(
            ['rospack', 'find', 'ntrip_client'],
            stderr=subprocess.DEVNULL,
        ).decode().strip()
        return os.path.join(pkg, 'scripts', 'ntrip_ros.py')
    except Exception:
        return None


def main():
    rospy.init_node('ntrip_watchdog', anonymous=False)

    # Read credentials — prefer explicit ROS params, fall back to env vars
    username = rospy.get_param('~username', os.environ.get('OM_NTRIP_USER',     '')).strip()
    password = rospy.get_param('~password', os.environ.get('OM_NTRIP_PASSWORD', '')).strip()
    host     = rospy.get_param('~host',     os.environ.get('OM_NTRIP_HOSTNAME', '')).strip()
    port     = str(rospy.get_param('~port', os.environ.get('OM_NTRIP_PORT', '2101'))).strip()
    endpoint = rospy.get_param('~mountpoint', os.environ.get('OM_NTRIP_ENDPOINT', '')).strip()

    # ── No credentials yet ────────────────────────────────────────────────────
    if not username or username == 'CHANGE_ME':
        rospy.logwarn(
            '[ntrip_watchdog] OM_NTRIP_USER not configured — NTRIP corrections disabled.\n'
            '  Register free at https://ergnss.ign.es/gnuserportal/ then set\n'
            '  OM_NTRIP_USER and OM_NTRIP_PASSWORD in mower_config.sh and restart.'
        )
        while not rospy.is_shutdown():
            rospy.loginfo_throttle(
                120,
                '[ntrip_watchdog] Waiting for NTRIP credentials (OM_NTRIP_USER / OM_NTRIP_PASSWORD).'
            )
            rospy.sleep(10.0)
        return

    # ── Find ntrip_ros.py ─────────────────────────────────────────────────────
    script = _ntrip_script_path()
    if not script or not os.path.isfile(script):
        rospy.logerr('[ntrip_watchdog] Cannot find ntrip_client package — NTRIP disabled.')
        rospy.spin()
        return

    # ── Supervisor loop ───────────────────────────────────────────────────────
    fast_fails = 0

    while not rospy.is_shutdown():

        if fast_fails >= MAX_FAST_FAILS:
            rospy.logwarn(
                f'[ntrip_watchdog] {fast_fails} rapid failures in a row — '
                f'pausing NTRIP for {BACKOFF_S / 60:.0f} min. '
                f'Check credentials and server status.'
            )
            deadline = time.time() + BACKOFF_S
            while not rospy.is_shutdown() and time.time() < deadline:
                remaining = int(deadline - time.time())
                rospy.loginfo_throttle(
                    60,
                    f'[ntrip_watchdog] NTRIP paused — retrying in {remaining}s.'
                )
                rospy.sleep(5.0)
            fast_fails = 0
            continue

        rospy.loginfo(
            f'[ntrip_watchdog] Starting NTRIP  '
            f'{host}:{port}/{endpoint}  (attempt {fast_fails + 1})'
        )

        cmd = [
            'python3', script,
            '__name:=ntrip_client',
            f'_host:={host}',
            f'_port:={port}',
            f'_mountpoint:={endpoint}',
            '_authenticate:=True',
            f'_username:={username}',
            f'_password:={password}',
            '_rtcm_message_package:=rtcm_msgs',
            '_rtcm_timeout_seconds:=30',
            '/nmea:=/ll/position/gps/nmea',
            '/rtcm:=/ll/position/gps/rtcm',
        ]

        t0 = time.time()
        proc = None
        try:
            proc = subprocess.Popen(cmd)
            while not rospy.is_shutdown():
                if proc.poll() is not None:
                    break
                rospy.sleep(0.5)
        except Exception as exc:
            rospy.logwarn(f'[ntrip_watchdog] Subprocess error: {exc}')
        finally:
            if proc and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()

        elapsed = time.time() - t0

        if elapsed < FAST_FAIL_THRESHOLD_S:
            fast_fails += 1
            rospy.logwarn(
                f'[ntrip_watchdog] NTRIP exited after {elapsed:.1f}s '
                f'(fast fail {fast_fails}/{MAX_FAST_FAILS}).'
            )
            rospy.sleep(3.0)
        else:
            fast_fails = 0
            if not rospy.is_shutdown():
                rospy.loginfo('[ntrip_watchdog] NTRIP connection ended — reconnecting in 2s.')
                rospy.sleep(2.0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
