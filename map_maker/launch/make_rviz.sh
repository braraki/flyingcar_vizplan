#!/bin/bash
directory=$1
cf_num=$2
map=$3
buildings=$4
echo $buildings

echo $directory

map_file="$directory"/config/"$map".yaml
cat $map_file
str='map_pre_dict:'
line=`grep "$str" $map_file`
line2=`echo $line |  tr : ' '`
line3=`echo $line2 | tr , ' '`

#setopt shwordsplit

count=0
for thing in $line3; do
	#echo $thing
	if [ $thing = '[]' ]; then
		count=$(($count+1))
	fi
done


echo $count




current_num=0

#define rviz file as file
file="$directory"/launch/duckietown.rviz

#delete everything in file
> $file
#replace with base_world txt
cat "$directory"/launch/base_world.rviz >> $file

#filename1=/home/crazyflier/flyingcars/catkin_ws/src/map_maker/launch/difference.txt

if [ $buildings == 'True' ] ; then
	current_num2=0

	filename3="$directory"/launch/house_insert.txt

	while [ $current_num2 -lt $count ]; do

		#sed -i "/#difference1/r $filename1" $file
		#sed -i -e $'s/#insertID/'$current_num$'/g' $file
		house_num=$((1 + $current_num2 % 5))

		sed -i "/#difference2/r $filename3" $file
		sed -i -e $'s/#insertID/'$current_num2$'/g' $file
		sed -i -e $'s/#inserthousenum/'$house_num$'/g' $file

		let current_num2=current_num2+1
	done
fi


filename2="$directory"/launch/difference2.txt

while [ $current_num -lt $cf_num ]; do

	#sed -i "/#difference1/r $filename1" $file
	#sed -i -e $'s/#insertID/'$current_num$'/g' $file
	
	sed -i "/#difference2/r $filename2" $file
	sed -i -e $'s/#insertID/'$current_num$'/g' $file

	let current_num=current_num+1
done
