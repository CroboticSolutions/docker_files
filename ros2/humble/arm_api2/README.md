# Dockerfile for moveit

Specify target you want to build.

If you want to build moveit for the Franka robot call:

```
DOCKER_BUILDKIT=1 docker build -t <img_name>:<ros2_distro> --target ros2_base .
```
Recommended command is:
```
DOCKER_BUILDKIT=1 docker build -t arm_api2_img:humble --target ros2_base .
```

Or you can pull it straight from the dockerhub as:
```
docker pull crobotic_solutions/arm_api2:latest
```

If you want to add GUI to your docker image, you can build it with (the GUI is currently in a private repository):
```
DOCKER_BUILDKIT=1 docker build -t arm_api2_img:humble --target add_gui --ssh default .
```

You can run the new docker image with:
```
./run_docker.sh
```

After that, you can start docker with:
```
docker start -i arm_api2_cont
```

After starting container you can execute it with:
```
docker exec -it arm_api2_cont bash
```

#### Possible issues:

- [ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper/issues/21)

## TODO:

- [x] ROS 2 + humble working again
- [ ] One click run
- [ ] SSH keys
- [ ] Dev setup [autocomplete + standard]
