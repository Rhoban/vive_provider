# Vive provider

## Get the dependencies

### Steam and SteamVR

You need to install steam and SteamVR.

*Note: There is possibilities that it doesn't create `udev` rules properly, if the file
`/lib/udev/rules.d/60-HTC-Vive-perms.rules` is not created by the process, you can
use the one from `misc/`.*

### Python

You need to install the following dependencies:

    pip install numpy pybullet openvr protobuf

## Usage

- vive_provider.py           : getting data from the mutliple vive trackers
- vive_server.py             : broadcasting UDP data to network
- vive_visualizer.py         : opengl visualization 
- vive_field_calibration.py  : calibrate vive referential using multiple points from soccer field

# Re-generate protobuf

You can use `generate_protobuf.sh` to regenerate protobuf iles
