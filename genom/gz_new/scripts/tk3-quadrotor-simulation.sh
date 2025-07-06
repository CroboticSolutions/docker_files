#!/bin/sh

# settings
middleware=pocolibs # or ros
components="
  rotorcraft
  nhfc
  pom
  optitrack
"
gzworld=$HOME/openrobots/share/gazebo/worlds/example.world

export GZ_SIM_RESOURCE_PATH=$HOME/openrobots/share/gazebo/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/openrobots/lib/gazebo


# list of process ids to clean, populated after each spawn
pids=

# cleanup, called after ctrl-C
atexit() {
    trap - 0 INT CHLD
    set +e

    kill $pids
    wait
    case $middleware in
        pocolibs) h2 end;;
    esac
    exit 0
}
trap atexit 0 INT
set -e

# init middleware
case $middleware in
    pocolibs) h2 init;;
    ros) roscore & pids="$pids $!";;
esac

# optionally run a genomix server for remote control
genomixd & pids="$pids $!"

# spawn required components
for c in $components; do
    $c-$middleware & pids="$pids $!"
done

# start gazebo
gz sim $gzworld --render-engine ogre & pids="$pids $!"

# wait for ctrl-C or any background process failure
trap atexit CHLD
wait

