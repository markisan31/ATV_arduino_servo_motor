#!/bin/bash
set timeout -1
ssh teamv2lk@10.10.0.232
expect "password: "
send "nvidia"
cd Desktop/ATV/Praktika_ATV/scripts/nvidia_camera.sh