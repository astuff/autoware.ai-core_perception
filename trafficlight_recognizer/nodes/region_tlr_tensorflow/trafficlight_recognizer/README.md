# TensorFlow Traffic Light Recognizer (TLR)
This package provides a region-based traffic light recognizer using a custom TensorFlow framework. The weights/framework file (.hdf or .hdf5) is not included with the node.

## Architecture
The TensorFlow TLR consists of two nodes: `region_tlr_tensorflow` and `tensorflow_tlr`. The first is mostly a topic subscriber/publisher and image cropper. It subscribes to a list of signals provided in an `autoware_msgs/Signals` message and a raw camera image. It uses the ROIs in the Signals message to crop the raw camera image and passes that cropped image to the `tensorflow_tlr` node via a service. The `tensorflow_tlr` node has only one service and no publishers or subscribers. It is a wrapper around a TensorFlow-based traffic light recognizer which uses `keras` to process the image and provide a light state and confidence back to the `region_tlr_tensorflow`, which then publishes it.

# TensorFlow Installation Instructions
Installation of TensorFlow can be done with or without GPU support. For this package to find TensorFlow, it must be installed with Python2.7 (not Python3) and must be a v1 version (not 2.0).

## Installing GPU Support (Optional)
If you wish to use GPU support in TensorFlow, you must install several prerequisites prior to installing the TensorFlow GPU package (`tensorflow-gpu`). Instructions for installing the necessary packages can be found at https://www.tensorflow.org/install/gpu.

## Installing the TensorFlow Python Package
To install TensorFlow with `pip`, you will need several prerequisite packages. To install these in Ubuntu, run:

`sudo apt update && sudo apt install --no-install-recommends python-pip python-dev python-wheel`

After these are installed, you can install TensorFlow with:

`pip install --user --upgrade tensorflow`

Or with GPU support:

`pip install --user --upgrade tensorflow-gpu`
