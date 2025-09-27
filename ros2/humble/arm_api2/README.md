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

## Docker compose
You can use docker compose to run an example of arm_api2, ros2_dash_gui and ollama LLM server.

Start the services in the background:
```
docker-compose -f compose.dash_ollama.yml up -d
```

Open the GUI in your browser at `http://localhost:8050`.
In the first run, ollama must download the model, so it can take some time. Refresh the page once in a while until it appears.

To stop the services:
```
docker-compose -f compose.dash_ollama.yml stop
```
Don't use the `down` command, because it will delete all the data in ollama and you will have to download the model again.


## TODO:

- [x] ROS 2 + humble working again
- [ ] One click run
- [ ] SSH keys
- [ ] Dev setup [autocomplete + standard]


#### Possible issues:

- [ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper/issues/21)