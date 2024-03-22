#!/bin/bash
# get old and new hostname
hn=`hostname`
# read new name in the file robotname
nn=`cat /home/local/robotname`
if [ $hn != $nn ]; 
then 
  echo new name, so rename host
  sed -i "s/$hn/$nn/g" /etc/hostname
  sed -i "s/$hn/$nn/g" /etc/hosts
else
  echo Same hostname, all is fine.
fi
