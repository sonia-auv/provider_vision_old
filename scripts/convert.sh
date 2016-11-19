#!/bin/sh                                                                                                                                                     

for file in *avi
do
    	echo "=> Transcoding '$file'... "
    	ffmpeg -i $file -c:v mpeg4 -q:v 1 -vtag xvid converted_$file
    	rm $file
	echo
done

