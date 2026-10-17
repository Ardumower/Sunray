#!/bin/bash


while getopts m:p: flag
do
    case "${flag}" in
        m) MSG=${OPTARG};;
        p) PARAM=${OPTARG};;
    esac
done


#echo "MSG=$MSG"
#echo "PARAM=$PARAM"

#dbus-send --system --dest=de.sunray.Bus --print-reply /de/sunray/Bus de.sunray.Bus.$MSG string:"$PARAM" 2>&1 >/dev/null  

dbus-send --system --dest=de.sunray.Bus /de/sunray/Bus de.sunray.Bus.$MSG string:"$PARAM"
