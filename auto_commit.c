#include <stdio.h>
#include <stdlib.h>

#define LOCATE "cd ../test_2/ATV_arduino_servo_motor"
#define gitSTATUS "git status"
#define gitADDALL "git add --all"
#define gitCOMMIT "git commit"

/*  "gst-launch-1.0 udpsrc port=5000 ! \
application/x-rtp,media=video,payload=96,encoding-name=H265 ! \
rtph265depay ! h265parse ! avdec_h265 ! \
autovideoconvert ! ximagesink sync=false" */


/* "gst-launch-1.0 -e videomixer name=mix sink_0::xpos=0 sink_0::ypos=0 sink_1::xpos=640 sink_1::ypos=0 ! \
queue ! videorate max-rate=30 ! autovideoconvert ! nvv4l2h265enc ! h265parse ! rtph265pay ! udpsink host=10.10.0.40 port=5000 v4l2src device=/dev/video0 ! \
video/x-raw,format=YUY2,width=640,height=480 ! mix_sink_0 v4l2src device=/dev/video1 ! video/x-raw,format=YUY2,width=640,height=480 ! mix_sink_1" */


int main()
{
  puts(LOCATE);
  //puts("Starting now");
  system(LOCATE);
  system(gitSTATUS);
  system(gitADDALL);
  system(gitCOMMIT);

  return 0;
}