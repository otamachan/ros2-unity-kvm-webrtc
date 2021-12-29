#!/bin/bash
export XORG_MODE="${SCREEN_SIZE:-1280x1024}"
export XORG_VIRTUAL="${XORG_MODE/x/ }"
export XORG_BUSID=$(printf %d:%d:%d $(nvidia-smi --query-gpu=gpu_bus_id --format=csv,noheader| sed -E 's/.*[0-9]+:([^:]+):([^\.]+)\.(.*)/0x\1 0x\2 0x\3/g'))
envsubst < /etc/X11/xorg.conf.temp > /etc/X11/xorg.conf

# START X
# use seat to avoid tty
Xorg -seat seat1 -logfile /tmp/Xorg.log &
sleep 1

# pulseaudio --start
# sleep 2

x11vnc -display :0 -nopw -forever -rfbport 5900 &
sleep 2

/noVNC-1.1.0/utils/launch.sh --vnc localhost:5900 --listen 8081 &
sleep 2

export DISPLAY=:0
openbox &
tint2
