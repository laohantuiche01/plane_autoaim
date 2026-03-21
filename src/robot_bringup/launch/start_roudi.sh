#!/bin/bash
if ! pgrep -x "iox-roudi" > /dev/null
then
    echo "Starting iox-roudi with high priority..."
    nice -n -20 iox-roudi -c /home/mijiao/ckyf_vision/iceoryx_config.toml &
else
    echo "iox-roudi is already running."
fi
