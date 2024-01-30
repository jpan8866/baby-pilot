#!/bin/bash

# This script is used to download the TF Lite models
# Can experiment with different models by adding more files
# supply filename when running detect.py

if [ $# -eq 0 ]; then
  DATA_DIR="./"
else
  DATA_DIR="$1"
fi

# Download TF Lite models
FILE=${DATA_DIR}/efficientdet_lite0.tflite
if [ ! -f "$FILE" ]; then
  curl \
    -L 'https://storage.googleapis.com/download.tensorflow.org/models/tflite/task_library/object_detection/rpi/lite-model_efficientdet_lite0_detection_metadata_1.tflite' \
    -o ${FILE}
fi

echo -e "Downloaded files are in ${DATA_DIR}"
