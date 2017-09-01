#!/bin/bash

# Set build mode if specified, default to Release otherwise.
mode=Release
if [ $# -ge 1 ]
then
  mode=$1
fi

echo "Build mode: $mode"

# Ensure storage folders exist
mkdir -p "build/Debug"
mkdir -p "build/Release"

# If the desired build is not the current one, swap files.

if [ -e "mode.txt" ] && [ $(echo -n $(cat "mode.txt")) != $mode ]
then
  if [ "$mode" == "Release" ]
  then
    checked="build/Debug"
    fetched="build/Release"
  else
    checked="build/Release"
    fetched="build/Debug"
  fi

  mv "build_isolated" "devel_isolated" $checked

  mv $fetched/* .
fi

echo $mode > "mode.txt"

# Build workspace.
if [ "$(set | grep DOKKA_IMAGE)" ]
then
    dokka exec catkin_make_isolated -DCMAKE_BUILD_TYPE=$mode
else
    catkin_make_isolated -DCMAKE_BUILD_TYPE=$mode
fi
