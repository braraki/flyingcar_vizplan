#!/bin/bash

map=$1

map_file=/home/crazyflier/flyingcars/catkin_ws/src/map_maker/config/"$map".yaml

echo $map_file

cat $map_file

str='map_pre_dict:'

line=`grep "$str" $map_file`

echo $line

num=`echo $line | sed -e 's/\(.\)/\1\n/g' | grep 1 | wc -'[]'`
echo num
