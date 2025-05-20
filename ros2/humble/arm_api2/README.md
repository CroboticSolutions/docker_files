# Dockerfile for moveit 

Specify target you want to build. 

If you want to build moveit for the Franka robot call: 

```
docker build --target real_robot -t <img_name>:<tag_name> .
```

# Workspaces

`moveit2_ws`: 

Contains `moveit2` built from source. 

And after that: 

`arms_ws`: is the workspace where we develop. 


#### Possible issues: 

- [ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper/issues/21)

## TODO: 

- [ ] ROS 2 + humble working again
- [ ] SSH keys 
- [ ] Dev setup [autocomplete + standard] 
