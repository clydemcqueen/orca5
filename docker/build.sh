#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $DIR

# --progress-plain will show more output, handy for debugging
docker build --progress=plain -f $DIR/Dockerfile -t orca5:latest ..
