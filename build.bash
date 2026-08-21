#!/bin/bash
docker build --build-arg user_id=$(id -u) --build-arg USER=$(whoami) --build-arg NAME=glider --rm -t dtc-jackal-`hostname`:glider .