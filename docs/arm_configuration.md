# UR5 Setup For Realtime Control

## First time setup

Follow the UR5 Manual to enable SSH access to the control box, this will make installation more convinient. Additionally, make sure to enable all RTDE related ports in advanced settings. For this configuration, we will use two UR5 caps; this are installed by dowloading the cap files, and using either **scp** (recommended) or an USB stick to copy over to the control box

* [External Control URCap](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases): used to have a controlled realtime loop, while retaining access to the pendant. 
* [RS485 Forwarder](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap/releases): launches a daemon which forwards comunication to the tool-io over TCP, this allows us to control the gripper at all times.

Once the caps are installed, make a blank script and add the External Control loop that was made available.

## Connecting and Using

1. Plug in the external control computer via ethernet to the control box.
    * Make sure the computer is in the same *subnet* as the control box. For example if control box is `192.168.0.15`, then set wired connection to `192.168.0.10`.

2. Launch control script on computer, this will wait until arm is enabled

3. On the pendant, load the external control listener script.

4. Hit play in the external control script on the pendant. 