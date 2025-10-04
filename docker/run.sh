#!/usr/bin/env bash

# Runs a locally built RoboPlan ROS container.

docker run \
  --rm \
  -it \
  --name roboplan-ros \
  --network host \
  roboplan_ros:latest \
  bash
