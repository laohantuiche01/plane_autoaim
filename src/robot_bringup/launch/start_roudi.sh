#!/bin/bash
if ! pgrep -x "iox-roudi" > /dev/null
then
    echo "Starting iox-roudi..."
    # Do not run in background, otherwise ros2 launch might kill it or it dies with the shell
    exec nice -n -20 iox-roudi -c /home/zxk/0ckyf_vision/ckyf_vision/iceoryx_config.toml
else
    echo "iox-roudi is already running."
    exit 0
fi
