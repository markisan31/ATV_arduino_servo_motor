#!/bin/bash
gst-launch-1.0 udpsrc port=5000 ! application/x-rtp,media=video,payload=96,encoding-name=H265 ! rtph265depay ! h265parse ! avdec_h265 ! autovideoconvert ! ximagesink sync=false