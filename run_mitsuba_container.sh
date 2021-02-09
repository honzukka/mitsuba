#!/bin/bash

data_dir=`pwd`

while getopts 'd:' c
do
    case $c in
        d) data_dir=$OPTARG ;;
    esac
done

docker run -ti --rm \
    --name mitsuba \
    -v "$data_dir":"/mitsuba/data" \
    mitsuba
