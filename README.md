# using line laser scan objects

easyscan NOT easy

## Part 1 using ros
### Calibrate the camera
Use ros package camera_calibration to get the camera intrinsic parameters
1. calibrate the first camera
`roslaunch easyscan calibrate_camera0.launch`

2. calibrate the second camera
`roslaunch easyscan calibrate_camera1.launch`

Move the calibration results to folder `config`, and named the two yaml files as `ost0.yaml` and `ost1.yaml`.

3. calibrate the laser plane and camera

Reference: Fast method to calibrate structure parameters of line structured light vision sensor
Use this command to calibrate the laser and camera
`roslaunch easy calibrate_laser_plane.launch`
The calibration results will be saved in file `config/laser_plane.yaml`
It will save three parameters, theta0, theta1, theta2, which represented the laser plane equation in camera1 coordintate system.

4. calibrate the transform matrix between camera1 and the QR_code
We use the camera0 to assist the calibration.
In this step, we should use the keyboard. 
* First, run the command
`roslaunch easyscan calibrate_camera1_QRcode.launch`
You can use this to see the results during the calibration.
`rqt_image_view results/calibrate_camera_QRcode`

* Second, put the chessboard board in the visual field of the camera0. You should thoose a position which is easy to localize the chessboard for camera0. Such as put the field center of the camera0. After the chessboard was solidly in place. You should press Enter to start the localization. 


## Part 2 no ros



