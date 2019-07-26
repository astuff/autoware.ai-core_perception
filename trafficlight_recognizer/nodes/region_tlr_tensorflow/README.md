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
