## Having trouble?

## Build

```
colcon build --packages-select ros2_deepstream && sudo cp build/ros2_deepstream/libds_rclcpp_publisher.so /usr/lib/x86_64-linux-gnu/gstreamer-1.0/
```
First debug deepstream by trying (video /dev/video* appropriately)
```
cd /opt/nvidea/deepstream/deepstream-4.0/samples
gst-launch-1.0 -v v4l2src device=/dev/video6 ! video/x-raw,framerate=30/1 ! videoconvert ! nvvideoconvert ! 'video/x-raw(memory:NVMM),format=NV12' ! m.sink_0 nvstreammux name=m batch-size=4 width=800 height=488 ! nvinfer config-file-path= configs/deepstream-app/config_infer_primary.txt batch-size=1 ! nvvideoconvert ! nvdsosd ! nveglglessink
```

Or using Yolo
```
gst-launch-1.0 -v v4l2src device=/dev/video6 ! video/x-raw,framerate=30/1 ! videoconvert ! nvvideoconvert ! 'video/x-raw(memory:NVMM),format=NV12' ! m.sink_0 nvstreammux name=m batch-size=4 width=800 height=488 ! nvinfer config-file-path= config_infer_primary_yoloV2.txt batch-size=4 ! nvvideoconvert ! nvdsosd ! nveglglessink sync=false

```



Combining everything

In one terminal to publish an image to image_transport
```
gst-launch-1.0 -v v4l2src device=/dev/video6 ! video/x-raw,framerate=30/1 ! videoconvert ! rclcpp_publisher
```

In another terminal to read and do inference:
```
gst-launch-1.0 -v rclcpp_subscriber ! videoconvert ! video/x-raw,width=800,height=448,format=RGBA ! nvvideoconvert ! 'video/x-raw(memory:NVMM),format=NV12' ! m.sink_0 nvstreammux name=m batch-size=4 width=800 height=448 ! nvinfer config-file-path= config_infer_primary_yoloV2.txt batch-size=4 ! nvvideoconvert ! nvdsosd ! nveglglessink
```
