#! /bin/sh
# mkTimeLapse creates a time-lapse move from a source of .jpg files.
# Copyright 2009-2013 Kjeld Jensen <kj@kjen.dk>
#
# Copying and distribution of this file, with or without modification,
# are permitted in any medium without royalty provided the copyright
# notice and this notice are preserved.  This file is offered as-is,
# without any warranty.
#

DIR_IMAGES="images" # Location of your images
FILE_VIDEO="robot_navigation.mp4" # Video output file

META_TITLE="Robot navigation"
META_YEAR="2013"
META_COPYRIGHT="Copyright (c) 2013 Kjeld Jensen"
META_COMMENT=""

FRAME_RATE_IN=10 # Change this value to control the frame rate of your time lapse movie
FRAME_RATE_OUT=25
BIT_RATE="750k"
#SCALE_CROP="scale=1920:1920*ih/iw, crop=1920:1080" # This creates 1920x1080 from GoPro Hero3 black 12Mp images
SCALE_CROP="scale=800:800*ih/iw" # This creates 1920x1080 from GoPro Hero3 black 12Mp images

# Copy images from the ROS data dir
mkdir $DIR_IMAGES
cp ~/.ros/img*.jpg $DIR_IMAGES

DIR_TEMP="images_sorted"

echo "Creating temporary image directory"
mkdir $DIR_TEMP

# Create symbolic links to the images in order (derived from the FFmpeg FAQ)
echo "Creating symbolic links to the images"
n=1
for i in $DIR_IMAGES/*jpg; do
	c=$(printf %05d $n)
	ln "$i" $DIR_TEMP/img"$c".jpg
	n=$(($n+1))
done

echo "Encoding video"
if [ -e $FILE_VIDEO ]; then
	rm $FILE_VIDEO
fi
ffmpeg -r $FRAME_RATE_IN -i $DIR_TEMP/img%05d.jpg -vcodec libx264 -vprofile high -b $BIT_RATE -vf "$SCALE_CROP" -metadata title="$META_TITLE" -metadata year="$META_YEAR" -metadata copyright="$META_COPYRIGHT" -metadata comment="$META_COMMENT" -r $FRAME_RATE_OUT $FILE_VIDEO
 
echo "Cleaning up"
rm -r $DIR_TEMP/*.jpg
rmdir $DIR_TEMP
rm -r $DIR_IMAGES/*.jpg
rmdir $DIR_IMAGES
