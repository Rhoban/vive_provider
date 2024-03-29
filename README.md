# Vive provider (Version 2.0 - Last update : Sept, 27 2022)

This OpenVR based code can be used to grab positions from the HTC Vive, allow you to calibrate
it, and to run a server that broadcasts the positions over the network. You can also visualize data save in .bin file 
for analysis purpose. 

## Get the dependencies

### Steam and SteamVR

You need to install steam and SteamVR.

*Note: There is possibilities that it doesn't create `udev` rules properly, if the file
`/lib/udev/rules.d/60-HTC-Vive-perms.rules` is not created by the process, you can
use the one from `misc/`.*

### Python

In this repository python3.* must be used.

You need to install the following dependencies:

    pip install -r requirements.txt

## How to use SteamVR ##

### Install the beta version of SteamVR ###

Go to Steam -> SteamVR -> Properties -> Betas. And choose beta - SteamVR Beta Update

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
#### Old calibration version with `vive_field_calibration.py` and `vive_server.py`
First, you can edit `field_points.json` to set up your ground truth positions. Then, run
`vive_field_calibration.py`. You need to have a paired controller then, and to go to each
position one by one (indicated in the 3D viewer by an arrow) to tag them.

The `field_points.json` should contain at least 3 points, and formatted as following:

```json
[
    [x1, y1, z1],
    [x2, y2, z2],
    [x3, y3, z3],
    ...
    [xn, yn, zn]
]
```

Alternatively, you can pass a file path to `vive_field_calibration.py` with `-p my_field_points.json`.

If a consistency error occurs, the joystick will vibrate, and you should start the calibration
over.

#### New calibration version
In the new calibration version, a new file `field_points_trackers.json` is created with each tracker used to calibrate the field.
```json
{
  "LHR-42A22618": [-2.5, -2, 0],
  "LHR-815E4573": [1, 0, 0],
  "LHR-52015056": [-2.5, 2, 0],
  ...,
  "<trackers_id>": [xn, yn, zn]
}
```
It's recommended not to place trackers on point of interest.

### Running the viewer

You can run the viewer using `vive_bullet.py` script. This will display the trackers and the
controllers positions.

### Running the server

The script `vive_server_auto_calib.py` (old version : `vive_server.py`) is a server that broadcasts the positions through the network
using UDP and protobuf definition from `proto/vive.proto`.

**Note: to avoid spamming, we broadcast UDP messages to `192.168.0.255` by default, this can be changed
using `-b` flag, you can pass `<broadcast>` to send the packets to all possible addresses**

To check, you can also run the `vive_bullet_client.py` that listens to the network instead of
using directly the OpenVR API.

After you killed a server. A binary file with all data from all tracker, base station, and controller are saved in two different file.
The first one contain data uncalibrated and the second with the calibrated ones.

## Re-generate protobuf

You can use `generate_protobuf.sh` to regenerate protobuf files

## Vive trackers frame

OpenVR output seems to be different with what is explained in the official guidelines. Thus, we referred
to the `vr_tracker_vive_1_0.stl` file. The frame we provide is modified so that:

- the origin point is at the bottom of the sensor, centered on the screw hole
- `z` is upward
- `x` is facing the board LED

## Data visualization 

When you closed `vive_server_auto_calib.py`, two files are generated. The first one is the uncalibrated version and the 
other one (with _auto_calib at the end) is the file with all data calibrated. 
To visualize data from a .bin file, you can launch 
```bash 
    python vive_data_visualization.py -l logs/<your log>.bin
```
Example : 
```bash 
    python vive_data_visualization.py -l logs/2022_09_27-11h32m48s_vive_auto_calib.bin
```
or
```bash 
    python vive_data_visualization.py -l logs/2022_09_27-11h32m48s_vive.bin
```