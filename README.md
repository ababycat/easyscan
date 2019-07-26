# using line laser scan objects

easyscan NOT easy

## Part 1 using ros
### Calibrate the camera
Use ros package camera_calibration to get the camera intrinsic parameters
#### calibrate the first camera
`roslaunch easyscan calibrate_camera0.launch`

#### calibrate the second camera
`roslaunch easyscan calibrate_camera1.launch`

Move the calibration results to folder easyscan/config, and named the two yaml files as "ost0.yaml" and "ost1.yaml".

### Calibrate the laser plane and camera

## Part 2 no ros



