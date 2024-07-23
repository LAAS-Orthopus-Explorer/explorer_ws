#! /bin/bash

docker build --no-cache --progress=plain . 2>&1 | tee build.log
