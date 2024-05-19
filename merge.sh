#! /bin/bash

cd $1;

for scene in `ls samples/`; do
    for channel in `ls samples/$scene/`; do
        if [ -d sweeps/$scene/$channel ]; then
            for frame in `ls samples/$scene/$channel/`; do
                cp samples/$scene/$channel/$frame sweeps/$scene/$channel/$frame &
            done
            wait
        fi
    done
done

cd ..;