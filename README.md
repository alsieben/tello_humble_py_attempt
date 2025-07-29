# tello_humble_py_attempt
Attempting to have a DJI Tello drone follow an aruco marker. Does actually work (surprisingly enough)

Runs on ros humble (in a docker container)

# Setup
Clone the repo and navigate to the main folder. To start the docker container, run:
```
docker compose up
```

From there, docker exec into the container with five terminals:
```
docker exec -it <container_name> /bin/bash"
```
In one terminal, run:
```
colcon build
``` 
and in all five terminals, run:
```
source install/setup.bash
```

# Calibration
## Tello Setup
Turn on the tello and connect to its wifi on the host computer. Then in one of the docker terminals, run the following to start the core tello node:
```
ros2 run tello_humble_py tello_core
```

In another docker terminal, open rviz by running:
```
ros2 run rviz2 rviz2
```
You may need to run ```xhost +``` in a new terminal outside of the docker container if this doesn't work. When rviz opens, click 'add' in the bottom right corner, go to the 'by topic' tab in the popup window, and click the 'image_raw' to bring up a video feed from the tello's front camera.

## Calibration Setup
Print or bring up a chessboard calibration plate/image and modify the expected parameters of the calibration plate in the calibrator script (change the values in lines 34-36 of 'tello_calib_img_cap_srv.py' or run them as arguments when you run the script from 'ros2 run'. If you change the defaults in the code, make sure to rebuild the package with colcon to update the built code with the new changes).
```
colcon build --packages-select tello_aruco_follower
```


## Calibrating the Camera
In a third docker terminal, run the calibration server with:
```
ros2 run tello_aruco_follower tello_calibrator
```

Point the tello drone's camera at the calibration plate. In a fourth docker terminal, run the calibration service server with:
```
ros2 service call /tello_calib_srv std_srvs/srv/Trigger
```
Use the video feed from rviz to make sure that the calibration plate is well visible to the camera. Repeat this 9 more times to get 10 calibration images (try to get images of the calibration plate from a few different angles and distances to improve calibration). You should see messages in the terminal running the calibration service that tell you how many images you have/need. Once you have enough images, the terminal running the calibration service will tell you to call the same service again to run the calibration. Do this, watch the calibration process run, and check the calibration service terminal again to confirm that it dumped a YAML file of the camera parameters (these will be used by the ArUco marker pose solver). You can now cancel the calibration service with ctrl+c (may make this automatic in the future)

# Running the code
## Tello Setup
Repeat the steps from the 'Tello Setup' subsection of the 'Calibration' section above (turn on the tello, connect to wifi, run ```ros2 run tello_humble_py tello_core```). You can run rviz again to get a video feed from the camera if you are interested.

## ArUco marker setup
Print or bring up an image of an aruco marker from the 4x4 50 dictionary (I used the '0' id marker). Ensure the marker is flat (bring up an image on a computer screen or print it and stick it to something flat and rigid like a clipboard), as the code assumes a flat marker and any bending can mess with the pose estimation. Ensure the 'aruco_edge_size_mm' parameter is appropriate to the real life size of the aruco marker (again, set the parameter when you run the tello_aruco_follower or change it in line 48 of the tello_aruco_follower.py code and run ```colcon build --packages-select tello_aruco_follower```).

## Follower node setup
In another docker terminal, run the ArUco follower node with:
```
ros2 run tello_aruco_follower tello_aruco_follower
```
It won't do anything yet, but you can point it at the aruco marker and watch the control inputs show on the tello_core terminal. In rviz, you can click 'add' in the bottom left corner again, go to the 'by topic' tab, and click on the 'image_aruco_det' to bring up a video feed that draws an outline over the aruco marker if it is detected.

## Flying the drone
Place the tello drone in an open space. You should have two free docker terminals. In one, type in (but don't run) the command 
```
ros2 topic pub --once /takeoff std_msgs/msg/Empty
``` 
and in the other, type (but also don't run yet):
```
ros2 topic pub --once /land std_msgs/msg/Empty
```
When you are ready to fly, grab your Aruco marker, press enter in the 'takeoff' terminal to run the command you typed in and make the drone take off. Hold the ArUco marker in front of the drone and it should begin following it. Don't move it too fast or the drone may lose tracking and stop. If this happens, just show the drone the marker again and it should resume following. When you are ready to land, press enter in the 'land' terminal to tell the drone to land.

## Tips on flying
- The drone video quality is not great, and its built-in wifi is not great for streaming HD video. It will probably stutter and freeze. Move slowly and have someone on standby to land the drone quickly if it loses tracking and does something strange.
- Again, the wifi connection is not great for HD video. If possible, use an external wifi adapter and try to maintain a clear path between the drone and the wifi adapter at all times. It still won't be great, but it will be much better
- A laptop is great for displaying the ArUco marker since the screen is very flat. Make sure the size is displayed properly on the laptop (check with a ruler). Lights in the room may cause glare on the screen which can interfere with tracking. However, turning the lights in the room off disrupts the drone's ability to hover in one place (I would guess that the optical flow sensor it uses to maintain staitionary hover does not work well in the dark), so don't do that.
- Don't bend the ArUco marker if it's on paper. This can severely impact the visual pose estimation algorithm, which assumes a perfectly flat marker.
- Look through the parameters in the 'tello_aruco_follower.py' file and change whatever you want. You can change the following (lines ~40-130):
  - PD controller parameters
  - control input scaling parameters for control inputs in each direction (as % of max possible input range)
  - target distance (in mm) the drone will follow the marker from
