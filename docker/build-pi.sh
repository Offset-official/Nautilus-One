#!/bin/bash

docker build -t pi-ros-full -f PiDockerFile --platform linux/arm64/v8 ..
