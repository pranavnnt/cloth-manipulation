# cloth-manipulation

This is a repository for cloth manipulation using the Franka Emika Panda. 

## for color mask

in terminal 1 : 
```sh
    cd ~/workspace/dressing_ws
    source devel/setup.bash 
    roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
```
in terminal 2 : 
```sh
    cd ~/workspace/dressing_ws
    source devel/setup.bash 
    roslaunch cloth_manipulation color_mask.launch
```
