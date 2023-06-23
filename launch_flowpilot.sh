set -e
source ./.env

export WIDE_ROAD_CAMERA_SOURCE="selfdrive/assets/fcam.avi" # no affect on android
export ROAD_CAMERA_SOURCE="selfdrive/assets/tmp" # no affect on android
export USE_GPU="1" # no affect on android, gpu always used on android
export PASSIVE="0"
#export MSGQ="1"
#export USE_PARAMS_NATIVE="1"
export ZMQ_MESSAGING_PROTOCOL="TCP" # TCP, INTER_PROCESS, SHARED_MEMORY

#export DISCOVERABLE_PUBLISHERS="1" # if enabled, other devices on same network can access sup/pub data.
#export DEVICE_ADDR="127.0.0.1" # connect to external device running flowpilot over same network. useful for livestreaming.

#export SIMULATION="1"
export FINGERPRINT="KIA SEDONA 2004"
# export FINGERPRINT="HONDA CIVIC 2016"

## android specific ##
export USE_SNPE="0" # only works for snapdragon devices.

scons && flowinit

while true; do sleep 1; done
