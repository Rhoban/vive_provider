# Vive provider

This OpenVR based code can be used to grab positions from the HTC Vive, allow you to calibrate
it, and to run a server that broadcasts the positions over the network.

## Get the dependencies

### Steam and SteamVR

You need to install steam and SteamVR.

*Note: There is possibilities that it doesn't create `udev` rules properly, if the file
`/lib/udev/rules.d/60-HTC-Vive-perms.rules` is not created by the process, you can
use the one from `misc/`.*

### Python

You need to install the following dependencies:

    pip install numpy pybullet openvr protobuf

## How to use SteamVR ##

### Install the beta version of SteamVR ###

Is this really needed ?

### Remove the need of headset

Add the following options to `~/.steam/steam/config/steamvr.settings`:

```json
"steamvr"{
    "requireHmd" : false,
    "forcedDriver": null,
    "activateMultipleDrivers": true
},
"driver_null" : {
    "enable" : true
}
```

### If you have intel graphics ###

Intel graphics are not supported by SteamVRV.
Error (306) : install mesa-vulkan-drivers (needed ?)
Error (307) : you can track with this error

After lunching steam, you might need to kill the process vrcompositor.

## Usage

### Calibration

First, you can edit `fieldPositions.json` to setup your ground truth positions. Then, run
`vive_field_calibration.py`. You need to have a paired controller then, and to go to each
positions one by one (indicated in the 3D viewer by an arrow) to tag them.

The `fieldPositions.json` should contain at least 3 points, and formatted as following:

```json
[
    [x1, y1, z1],
    [x2, y2, z2],
    [x3, y3, z3],
    ...
    [xn, yn, zn]
]
```

Alternatively, you can pass an argument to `vive_field_calibration.py` which is the json
file path to the field positions.

If a consistency error occurs, the joystick will vibrate and you should start the calibration
over.

### Running the viewer

You can run the viewer using `vive_bullet.py` script. This will display the trackers and the
controllers positions.

### Running the server

The script `vive_server.py` is a server that broadcasts the positions through the network
using UDP and protobuf definition from `proto/vive.proto`.

To check, you can also run the `vive_bullet_client.py` that listens to the network instead of
using directly the OpenVR API.

## Re-generate protobuf

You can use `generate_protobuf.sh` to regenerate protobuf files

## Vive trackers frame

OpenVR output seems to be different with what is explained in the official guidelines. Thus, we referred
to the `vr_tracker_vive_1_0.stl` file. The frame we provide is modified so that:

- the origin point is at the bottom of the sensor, centered on the screw hole
- `z` is upward
- `x` is facing the board LED