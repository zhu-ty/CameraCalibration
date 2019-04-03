#!/bin/bash 

for i in `ls $1`
do
    #echo $i
    file=${i%%.*}
    
    if ! (($((10#$file)) % 3 == 0)); then
        #echo $((10#$file))
        #echo ${file}
        mv $1/$i "$1/$i.del"
    fi
done
