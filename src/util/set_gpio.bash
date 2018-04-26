#!/usr/bin/env bash
echo 17 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio17/direction