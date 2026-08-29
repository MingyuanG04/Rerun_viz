# Vive Tracker 3.0 6-DoF visualizer

`vive_tracker_rerun.py` displays every valid Vive Tracker in a Rerun window. Its pose is expressed in the local coordinate system of one SteamVR Base Station 2.0, so that selected base station is always at `[0, 0, 0]` with identity rotation. The RGB axes are X/red, Y/green, and Z/blue.

## Reality check: what can and cannot be headless

An HMD is not required. Vive Tracker 3.0 devices paired through their USB dongles can be tracked by two Base Station 2.0 units in a tracker-only setup.

However, OpenVR is only a client library; it reads poses from an OpenVR runtime. For Vive hardware, that runtime is SteamVR (including its lighthouse and Vive tracker drivers). Therefore an OpenVR program cannot be independent of SteamVR. This application uses the background application mode and never creates an HMD render context or uses the SteamVR dashboard; SteamVR only needs to remain running in the background.

Before running the script, use SteamVR once to pair all three trackers and ensure each tracker and both base stations appears as tracked. A tracker-only rig may need a one-time SteamVR room calibration so that the runtime reports tracking-reference poses.

## Install and run

Use a Python environment with the packages below:

```powershell
py -m pip install -r requirements.txt
py .\vive_tracker_rerun.py
```

If `py` is unavailable, install Python 3.10+ and use the interpreter path that your installation provides. Start SteamVR first, power on the two base stations, connect the three paired tracker dongles, and turn on the trackers. The script opens a Rerun viewer automatically.

At start-up, the script prints the serial number selected as the world reference. To pin a particular base station for repeatable measurements, restart with that serial:

```powershell
py .\vive_tracker_rerun.py --reference-serial LHB-XXXXXXXX
```

The default reference is the valid tracking-reference device with the lowest current OpenVR index. OpenVR indices can change after reconnecting hardware, while serials are stable.

## Operating notes

- The reference changes the coordinate origin only; it does not improve tracker accuracy. Base-station-relative coordinates remain subject to normal lighthouse tracking error and occlusion.
- If no reference pose appears, confirm that SteamVR has valid poses for the base stations. The program intentionally does not make up a base-station transform.
- All tracker serials are shown as Rerun entity names under `world/trackers`. This avoids device-index swapping when devices are turned on in a different order.
- `--hz 90` controls pose polling. Use `--no-spawn` if a Rerun viewer has already been started separately.

## If SteamVR truly must not be used

The architecture needs to change: use a lighthouse-tracking implementation such as libsurvive (with compatible hardware/driver support) to obtain raw tracker poses, then feed its output into Rerun. That is not an OpenVR-based solution and requires a separate hardware compatibility and calibration validation effort.
