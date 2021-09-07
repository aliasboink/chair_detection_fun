# Chair Detection and Depth Measurement (plus Rviz marking)
All of these have to be built within a ROS workspace with the ros_deep_learning Nvidia made ROS package.
I take information from the detectnet _node_ and otherwise leave it intact. All the messages are received and then filtered in the "object_det" based on the detection ID. The RViz visualization currently uses the frame_id of "/map", this should be changed to the "/base_link" of the robot for the sake of adequate mapping (if everything works proper).
## Steps
1. Run roscore on your main machine.
> roscore
2. This runs the detect net node with the image input topic as "/cam1/image_raw". This was set during the CCM_SLAM testing period.
> rosrun ros_deep_learning detectnet /detectnet/image_in:=/cam1/image_raw
3. The camera publisher is simply another node you ought to run. If you want to see the image overlay (with the bounding boxes), simply run the "camera_subscriber.py". This works with OpenCV, so it will be heavily delayed, but you can use it for debugging.
> rosrun camera_stream_csi camera_publisher.py 
