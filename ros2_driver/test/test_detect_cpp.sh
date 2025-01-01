#!/bin/bash
#DATASET_NAME="no_background"
DATASET_NAME="current"
RESULT_DIR="result"
source ../install/setup.bash
rm -fr data/${RESULT_DIR}
mkdir -p data/${RESULT_DIR}

for f in data/${DATASET_NAME}/*; do 
    filename=$(basename "${f}")
    ros2 run octa_ros test_detect "${f}" "data/${RESULT_DIR}/${filename}"
done
