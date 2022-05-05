# arthur_perception
ROS package for all perception-related functionalities of the Arthur robot

# Testing and Full Usage Instructions

### Start the camera node:

```
roslaunch arthur_perception start_camera.launch
```

Note: This step will prompt you to type the password. If you're getting errors here, try a different USB port or reboot the camera.

2. Once the camera is up and running, you can run the registration script in testing mode. This will load a saved pointcloud that was previously collected. This assumes that the camera extrinsics haven't changed. You can do this as follows:

```
roslaunch arthur_perception registration.launch testing:=true
```


This will open the first Open3D window. It is just a visualization of the points. Press Q to close it. This will open the next window. In the second window, you will see the scanned pelvis model.  Choose 4 points (Shift + Left mouse click) on the acetabulum and press Q. Choose the same 4 points in the third window and press Q. You will see the registered output in a new window. Press Q to close it. The final window will ask you to choose the point to which you want to ream. Choose only ONE point, and close the window. This will then publish the transform to the reaming end point. 

3. To run the perception pipeline and also collect points, do the following: 

```
roslaunch arthur_perception registration.launch
```

# Camera Calibration

Note: It is possible that the camera has moved, and therefore the extrinsics needs to be re-calibrated. To recalibrate, do the following: 

4. Make sure that the robot is connected to the computer, and that you are able to communicate with it. Once this is done, run the camera node by following the instructions in step 1. Once both camera and the arm are running, run the calibration launch file as follows: 

```
roslaunch arthur_calibration calibrate.launch 
```

5.  Make sure that the calibration target on the arm is visible to the camera as well. This will open a GUI on which you can collect points for calibration. Make sure that you rotate the end-effector about roll, pitch and yaw. Do not command extravagant translational motions - keep them minimal and redundant. Collect up to 10-15 points, and click ‘Compute'. This will give you the translation and rotation of the camera with respect to the base-link.

6. Update these values in the static transform publisher in `kortex_driver.launch` in the `kortex_driver` package (found within `ros_kortex`).