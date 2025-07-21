# GenoM - Gazebo

Initial docker to set up GenoM components for the easier development.

## Setup
Build with `docker build -t genom_img:gz_new --ssh default .`

Instructions how to use Docker can be found at the [LARICS wiki](https://github.com/larics/docker_files/wiki). 
Instructions how to build docker image and container can be found at [How to use Docker?](https://github.com/larics/docker_files/wiki/3.-Usage). 

# Env variables for gz: 

```
export GZ_SIM_PLUGIN_PATH=$HOME/openrobots/lib/gazebo:${GAZEBO_PLUGIN_PATH}
export GZ_SIM_MODEL_PATH=$HOME/openrobots/share/gazebo/models
```

Rest of the env variables is sourced with the corresponding script in the `scripts` folder. 

## How to run init simulation:

In one terminal you can run:
```
./tk3-quadrotor-simulation.sh
```

In the other terminal you can run:
```
python3 -i quadrotor_ctl.py
```

After pressing play in the gazebo simulation you can run
quadrotor in the python interpreter as:
```
setup()
start()
...
nhfc.set_position()
...
```

## How to build custom GenoM pkg

Position in the `/root/devel/<genom_folder>`. And run set of the following commands:
```
./bootstrap.sh
mkdir build; cd build
POCOLIBS: ../configure --prefix=$DEVEL_BASE --with-templates=pocolibs/client/c,pocolibs/server"
ROS: ../configure --prefix=$DEVEL_BASE --with-templates=ros/client/c,ros/server,ros/client/ros"
make install
```

## TODO:
- [x] build example telekyb3 with all reqs
- [x] Set up init quadrotor simulation with gz-sim
- [x] Test ctl
- [x] Add aerial physical interaction package with it
- [x] Wait for the ROS 2 support check
- [x] Buid custom package
- [x] Run custom package
- [x] Write instructions for the README.md
- [x] Someone else builds Dockerfile

## Useful documentation:
- [Genom-ROS-Gazebo](https://homepages.laas.fr/felix/files/tp.pdf)
- [Quick setup guid for robotpkg](http://robotpkg.openrobots.org/install.html)
- [mrsim-gazebo](https://git.openrobots.org/projects/mrsim-gazebo)
- [mrsim-gazebo-docs](https://git.openrobots.org/projects/mrsim-gazebo/pages/README)
- [robotpkg](https://git.openrobots.org/projects/telekyb3/pages/software/install/robotpkg)
- [Running a quadrotor in simulation](https://git.openrobots.org/projects/telekyb3/pages/software/run/quadrotor-simulation)
