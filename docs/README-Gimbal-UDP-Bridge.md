# DJI Gimbal UDP Bridge Runbook

| Field | Value |
| --- | --- |
| Status | Working runbook for the modified DJI Mobile SDK Android sample app |
| Last updated | 2026-06-30 |
| Audience | Operator or developer setting up gimbal attitude UDP streaming |
| Output | Gimbal pitch/roll/yaw JSON over UDP |
| Receiver | Ubuntu/ROS PC on UDP port `5005` |

## Summary

This document explains how to stream DJI gimbal attitude data from the modified DJI Mobile SDK Android sample app to an Ubuntu/ROS PC over UDP.

The current implementation sends JSON packets from the Android controller/app to:

```text
10.187.118.204:5005/udp
```

The sender is implemented in:

```text
Sample Code/app/src/main/java/com/dji/sdk/sample/internal/view/MainContent.java
```

The app sends approximately 10 packets per second. Each packet has this shape:

```json
{"t":1782491123456,"pitch":-12.400,"roll":0.100,"yaw":3.700}
```

Do not move on to ROS integration until `tcpdump` shows repeated UDP packets on the Ubuntu PC.

## Network Topology

Use one shared network for both devices:

```text
DJI controller / Android device running the sample app
        |
        | UDP 5005
        v
Ubuntu / ROS PC at 10.187.118.204
```

Recommended options:

1. Phone hotspot: connect both the Ubuntu PC and the DJI controller/Android device to the same phone hotspot.
2. Wi-Fi router: connect both devices to the same Wi-Fi network.

Avoid networks with client isolation enabled. Some hotspots or guest Wi-Fi networks allow internet access but block device-to-device traffic.

## Prerequisites

Ubuntu PC:

- This repository checked out.
- Java 11 installed at `/usr/lib/jvm/java-11-openjdk-amd64`.
- Ability to run `tcpdump` with `sudo`.
- Optional: `nc` or `socat` for UDP listener tests.

Android/DJI side:

- DJI controller or Android device that can run the DJI SDK sample app.
- Aircraft connected to the app.
- DJI Pilot, Litchi, or other DJI apps force-stopped before opening the sample app.

## Step 1: Put Both Devices on the Same Network

Start the phone hotspot or connect both devices to the same Wi-Fi.

On Ubuntu, check the IP address used on that network:

```bash
hostname -I
ip route get 8.8.8.8
```

Use the `src` address from `ip route get 8.8.8.8`. Example:

```text
8.8.8.8 via 10.187.118.1 dev wlp0s20f3 src 10.187.118.204
```

In this example, the Ubuntu PC IP is:

```text
10.187.118.204
```

That IP must match `ROS_PC_IP` in `MainContent.java`.

## Step 2: Confirm the Android Target IP

From the repository root:

```bash
cd ~/work_ws/buscan/Mobile-SDK-Android/"Sample Code"

grep -n "ROS_PC_IP\|GIMBAL_UDP_PORT" \
  app/src/main/java/com/dji/sdk/sample/internal/view/MainContent.java
```

Expected values for the current hotspot setup:

```java
private static final String ROS_PC_IP = "10.187.118.204";
private static final int GIMBAL_UDP_PORT = 5005;
```

If the Ubuntu PC gets a different IP on a different hotspot or router, update `ROS_PC_IP`, rebuild, and reinstall the APK.

## Step 3: Build the APK

From `Sample Code/`:

```bash
JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64 ./gradlew clean assembleDebug
```

The APK will be generated at:

```text
Sample Code/app/build/outputs/apk/debug/app-debug.apk
```

Optional convenience copy:

```bash
cp app/build/outputs/apk/debug/app-debug.apk ~/Downloads/dji-msdk-v4-sample.apk
```

## Step 4: Install the APK on the Controller

Use MTP, USB file transfer, `adb install`, or a USB stick depending on the controller.

Recommended clean install:

1. Uninstall the old DJI SDK Sample app from the controller.
2. Install the newly built `app-debug.apk`.
3. Grant all requested permissions when the app starts.

Before opening the sample app, force-stop other DJI apps:

- DJI Pilot
- DJI Pilot 2
- Litchi
- Any other app that may own the DJI product connection

## Step 5: Open and Connect the App

On the controller:

1. Connect to the same hotspot/Wi-Fi as the Ubuntu PC.
2. Open `DJI SDK Sample`.
3. Tap `Register APP`.
4. Wait for SDK registration success.
5. Connect the aircraft/product.
6. Wait for the app to show product connection status.
7. Confirm the toast appears:

```text
Gimbal UDP bridge started
```

That toast means the UDP bridge initialized. It does not by itself prove packets are arriving on Ubuntu; use `tcpdump` for that.

## Step 6: Verify Packets on Ubuntu

Primary verification:

```bash
sudo tcpdump -ni any udp port 5005
```

Expected output:

```text
IP 10.187.118.141.45257 > 10.187.118.204.5005: UDP, length 58
```

To view payloads:

```bash
sudo tcpdump -ni any udp port 5005 -A
```

Expected payload:

```json
{"t":1782491123456,"pitch":-12.400,"roll":0.100,"yaw":3.700}
```

`tcpdump` is the most reliable first test. `nc -klu 5005` can be used later, but if `tcpdump` shows packets and `nc` does not, the problem is the listener command, not the Android sender.

## Optional: Local UDP Listener Test

This only proves Ubuntu can receive UDP locally. It does not prove Android is sending.

Terminal 1:

```bash
nc -klu 5005
```

Terminal 2:

```bash
echo "hello" | nc -u -w1 127.0.0.1 5005
```

Terminal 1 should print:

```text
hello
```

If `nc` behaves inconsistently, use:

```bash
socat -v UDP-RECV:5005,fork -
```

Install if needed:

```bash
sudo apt install socat
```

## Troubleshooting

### No packets in tcpdump

Check these in order:

1. Ubuntu and Android/controller are on the same hotspot/Wi-Fi.
2. `ROS_PC_IP` matches the Ubuntu `src` IP from `ip route get 8.8.8.8`.
3. The modified APK was rebuilt and reinstalled, not just rebuilt.
4. The old app was uninstalled before installing the new APK.
5. The app shows SDK registration success.
6. The app shows product connected.
7. The toast `Gimbal UDP bridge started` appears.
8. DJI Pilot, DJI Pilot 2, Litchi, and similar apps are force-stopped.
9. The hotspot/router does not block client-to-client traffic.

### Toast appears but no packets arrive

This usually means one of:

- Wrong `ROS_PC_IP`.
- The Android/controller is on a different network than the Ubuntu PC.
- Hotspot client isolation is blocking device-to-device UDP.
- The installed APK is not the one that was just built.

Run on Ubuntu:

```bash
ip route get 8.8.8.8
sudo tcpdump -ni any udp port 5005
```

Then confirm `ROS_PC_IP` matches the `src` IP.

### Only one packet arrives

Install the current APK from this repository. The current bridge caches the latest gimbal attitude and actively sends it every 100 ms. Older versions sent only when DJI emitted an attitude change event, which could produce only one packet.

### tcpdump shows packets but nc shows nothing

The network and Android sender are working. Use `tcpdump -A` or `socat` for payload inspection:

```bash
sudo tcpdump -ni any udp port 5005 -A
socat -v UDP-RECV:5005,fork -
```

### Product not connected

No product connection means no valid gimbal data. On the controller:

1. Force-stop DJI Pilot/Pilot 2/Litchi.
2. Reopen `DJI SDK Sample`.
3. Tap `Register APP`.
4. Connect the aircraft.
5. Wait for product connected status.

## Implementation Notes

The bridge uses `GimbalKey.ATTITUDE_IN_DEGREES` through DJI `KeyManager`, not `Gimbal.setStateCallback`.

Reason: the sample app's `PushGimbalDataView` also uses `setStateCallback`. DJI exposes that as a single callback slot, so using it for the UDP bridge can be overwritten when viewing gimbal data in the UI.

The current bridge:

- Listens for keyed gimbal attitude updates.
- Caches the latest `Attitude`.
- Polls the key and sends the latest attitude every 100 ms.
- Sends UDP JSON to `ROS_PC_IP:GIMBAL_UDP_PORT`.
- Stops the listener and closes the socket on product disconnect or view detach.

## Changing the Receiver IP or Port

Edit:

```text
Sample Code/app/src/main/java/com/dji/sdk/sample/internal/view/MainContent.java
```

Change:

```java
private static final String ROS_PC_IP = "10.187.118.204";
private static final int GIMBAL_UDP_PORT = 5005;
```

Then rebuild and reinstall:

```bash
cd ~/work_ws/buscan/Mobile-SDK-Android/"Sample Code"
JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64 ./gradlew clean assembleDebug
```

## Definition of Done

The setup is working when all of these are true:

- Ubuntu and the controller are on the same network.
- The sample app registers successfully.
- The aircraft/product is connected in the sample app.
- The controller shows `Gimbal UDP bridge started`.
- Ubuntu `tcpdump` shows repeated UDP packets to port `5005`.
- `tcpdump -A` shows JSON with `t`, `pitch`, `roll`, and `yaw`.
