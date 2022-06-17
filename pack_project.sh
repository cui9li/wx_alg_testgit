#!/bin/bash

COMMIT_ID=`git rev-parse HEAD`
suffix=`date +%Y%m%d`

list="${list} active_monitor/"
list="${list} spo2/"
list="${list} taishou/"
list="${list} test/"
list="${list} alg_ComFunc.c"
list="${list} alg_ComFunc.h"

echo ${COMMIT_ID}

zip_file_name=WX_alg_${COMMIT_ID:0:7}_${suffix}.zip

rm -f $zip_file_name

./script/zip.exe -r $zip_file_name $list
