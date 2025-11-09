#!/usr/bin/env bash

# Usage:
# ./build.sh <target>

if [ -z "$1" ]; then
  echo "Error: <target> argument required."
  echo "Usage: ./build.sh <target>"
  echo "where <target> can be:"
  echo "  sim:  includes Gazebo and ArduSub"
  echo "  hw:   minimal install for topside computer"
  exit 1
fi

TARGET=$1
TAG="orca5:${TARGET}"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $DIR || exit

# --progress-plain will show more output, handy for debugging
docker build --progress=plain --target ${TARGET} -f $DIR/Dockerfile -t ${TAG} ..
