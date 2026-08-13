#!/bin/bash
# Run this on the HOST machine, NOT inside the docker container.
# The container already has --privileged + /dev:/dev, so it inherits whatever
# permissions the host udev rule grants -- the rule itself only needs to exist
# on the host, since the container has no udev daemon of its own.

echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
