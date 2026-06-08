# This file can now be found in the /config folder of this repository

export OM_MOWER="CUSTOM"
export OM_MOWER_ESC_TYPE="xesc_mini"

# Platform config


export OM_NTRIP_USER=alcdaniel
export OM_NTRIP_PASSWORD=0c7331c5-d
export OM_NTRIP_RECONNECT_WAIT_SECONDS=5
export OM_NTRIP_RECONNECT_MAX=99999


# Platform config (auto) — added by setup_raspberry_ubuntu.sh
export HARDWARE_PLATFORM=2
export OM_V2=True
export MOWER=$OM_MOWER
export ESC_TYPE=$OM_MOWER_ESC_TYPE
export PARAMS_PATH=$HOME
export RECORDINGS_PATH=$HOME
export OM_GPS_PROTOCOL=UBX
export OM_MEGA_BAUD=115200
export OM_GPS_BAUDRATE="115200"
export OM_GPS_PORT="/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"
export OM_USE_RELATIVE_POSITION=False
export OM_USE_NTRIP=True
export OM_NTRIP_HOSTNAME=192.148.213.42
export OM_NTRIP_PORT=2102
export OM_NTRIP_ENDPOINT=XIXO3M
