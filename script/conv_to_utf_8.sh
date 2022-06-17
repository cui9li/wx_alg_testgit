#!/bin/bash

src_path="../"
#find $src_path | grep '\.[ch]$' > temp1
#file `cat temp1`
file_list=`find $src_path | grep '\.[ch]$'`
file_type=`file $file_list`
file_conv=`echo "$file_type" | grep "ISO-8859"`
file_conv_list=`echo "$file_conv" | awk -F ':' '{print $1}'`

for file_name in $file_conv_list 
do
	echo $file_name
	#echo "iconv -f GB18030 -t UTF-8 $file_name > ${file_name}.A"
	iconv -f GB18030 -t UTF-8 $file_name > temp1
	mv temp1 $file_name
done
