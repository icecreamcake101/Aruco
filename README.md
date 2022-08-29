
# Aruco Mapping and Localization Nodes

### Dependencies
- [alfons_msgs]
- [aruco_detector_ocv]
- [simple_kf]
- [rviz_pics]
- [zed_cpu_ros]
- opencv

### Installation
- run `sudo apt-get install ros-noetic-aruco-ros`
- run `mkdir -p ~/aruco_ws/src`
- run `cd ~/aruco_ws`
- place all your packages inside src
- run `catkin_make`
- run `source devel/setup.bash`

### Converting Python 2 to Python 3
- run `sudo apt-get install 2to3` or use synaptic to install 2to3
- run `cd ~/aruco_ws/src/tf_mapping/scripts`
- run `2to3 -w tf_create_map.py` 
- run `2to3 -w tf_navigate.py` 

### Mapping
- place the camera such that no marker is detected
- run `rosrun tf_mapping tf_create_map.py`
- run `roslaunch tf_mapping start_tf_mapping.launch`
- Point the camera towards the first marker
- Move the camera slowly such that only the second marker and the first marker are detected at the same
- Move the camera slowly again such that now second marker and third marker are detected and so on
- After the mapping is done you will find the pictures saved in rviz_pics/map and the location of markers in tf_mapping/launch/usemap.launch
##### **Note: Make sure that exactly two markers are visible at any given point** 

### Localization
- run `roslaunch tf_mapping start_tf_navigate.launch`
- Point the camera towards any marker, this will become your first marker

### Editing Launch files to use zed camera
- inside aruco_detector_ocv/launch open detector.launch
- uncomment the zed_cpu_ros node and make sure image_raw is remapped to /camera/left/image_raw and camera_info to /camera/left/camera_info
- comment out the image_republish node (This is used in case you are running compressed image_raw from a bag file)

### Possible Errors and Fixes
- #### CORNER_REFINEMENT_SUBPIX No such method ...
 >&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Update OpenCV to 3.3+
- #### QT Version Mismatch Error
>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; In rviz_pics edit the CMakeLists so that you remove the lines &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;which use the version of QT different from the one on system  

