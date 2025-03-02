#!/bin/bash

docker build -t nano-ros-full -f NanoDockerFile --platform linux/arm64/v8 ..
