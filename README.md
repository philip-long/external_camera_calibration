# external_camera_calibration


A interface for tuning a depth camera's external parameters. For a given desired frame and camera frame launch as follows 

## Manually Calibration

```
roslaunch camera_calibration_man camera_calibration.launch world_frame:=base camera_frame:=kinect2_link
```
Dynamically edit the location of the camera frame until point cloud matches robot model. Then save the transform 
and add it to your camera launch file. 

## Automatic Calibration