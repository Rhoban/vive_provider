# Vive provider

## Get the dependencies

You need to install the following dependencies:

    pip install numpy pybullet openvr protobuf

## Usage

- vive_provider.py           : getting data from the mutliple vive trackers
- vive_server.py             : broadcasting UDP data to network
- vive_visualizer.py         : opengl visualization 
- vive_field_calibration.py  : calibrate vive referential using multiple points from soccer field

# Re-generate protobuf

You can use `generate_protobuf.sh` to regenerate protobuf iles
