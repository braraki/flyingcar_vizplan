#!/bin/bash

map=$1

map_file=/home/crazyflier/flyingcars/catkin_ws/src/map_maker/config/"$map".yaml

echo $map_file

cat $map_file

str='map_pre_dict:'

line=`grep "$str" $map_file`
#IFS=':'; line2=($line); unset IFS;

line2=`echo $line |  tr : ' '`
line3=`echo $line2 | tr , ' '`
#echo $line2

#num=`echo $line | sed -e 's/\(.\)/\1\n/g' | grep 1 | wc -'[]'`
#num=grep word1 $line | wc -'[]'
#big_num=`grep -o '[]' $map_file | wx -l`

#echo $big_num


setopt shwordsplit

count=0
for thing in $line3; do
	echo $thing
	if [ $thing = '[]' ]; then
		count=$(($count+1))
	fi
done


echo $count
