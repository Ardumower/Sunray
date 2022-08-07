#!/bin/bash

## https://github.com/Ardumower/Sunray/tree/master/alfred

## install new sunray executable ##
sudo systemctl stop sunray
cp sunray ~/sunray_install/
sudo systemctl start sunray
