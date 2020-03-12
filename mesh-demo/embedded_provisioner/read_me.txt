-------------------------------------------------------------------------------
Mesh Embedded Provisioner application
-------------------------------------------------------------------------------

Overview
--------
This demo application shows an implementation of a dimmable light which
also serves as an embedded provisioner. A network may only have a single
device like that. To build application, specify EMBEDDED_PROVISIONER=1 in
the command line, or in a make file. For example
demo.mesh.embedded_provisioner-CYW920719Q40EVB_01 EMBEDDED_PROVISION=1 download

When embedded provisioner device starts for the very first time, it
creates a network, provisions the local device into the network and reboots.
The next time the app starts up, it is already provisioned. The app
uses Remote Provisioning Service feature to add devices to the network.
On startup the app goes through all network devices and tells them to scan
for unprovisioned devices. When a new device is found by, the app starts
a guard timeout to allow other provisioners to report the same device and
when timeout expires, the app selects the best provisioner and performs
provisioning using that remote provisioning server.

To decide if an unprovisioned device needs to be added to the network,
the app can either use RSSI or can use a magic number in the UUID. See
mesh_provsioner_validate_unprovision_device function in the mesh_scanner.
It is expected for a production app to update this function to use
preprogrammed list of UUIDs and some OOB data.

After the new device is provisioned, the app performs configuration. To
do that the app adds an application key, binds vendor specific model to
this application key and then sends the command to self configure. The
configuration values are hardcoded in the embedded_provisioner.h file.

The app is based on the snip/mesh/mesh_light_lightness sample which
implements BLE Mesh Light Lightness Server model. Because Light Lightness
Server model extends Generic OnOff and Generic Level, the dimmable
light can be controlled by a Switch (Generic OnOff Client), a Dimmer
(Generic Level Client), or by an application which implements Light
Lightness Client.  The WICED Mesh Models library takes care of the
translation of the OnOff and Level messages and the only messages
that the application layer needs to process is those of the Light
Lightness Model.

This application supports factory reset via five fast power off cycles:
  turn off device in less than 5 seconds after power on and do it five times.
It increments the power on counter (persistent in the NVRAM) on each power on
  and does factory reset when counter reaches 5. It resets that counter when
  device stays in the power on state for longer than 5 seconds.

Features demonstrated
 - Autoprovisioning and configuration of unprovisioned devices
 - LED usage on the CYBT-213043-MESH EZ-BT Mesh Evaluation Kit / CYW920819EVB-02 Evaluation Kit
 - Processing of the Light Lightness messages

See chip specific readme for more information about the BT SDK.

Instructions
------------
To demonstrate the app, work through the following steps.
1. Build and download the application to the Mesh Evaluation Kit / CYW920819EVB-02 Evaluation Kit
2. Build and download a controlling application such as BLE MeshDimmerSelfConfig to another Mesh Evaluation Kit
   / CYW920819EVB-02 Evaluation Kit
3. This application will detect an unprovisioned device, provision it into the network and configure.
4. Push/release the  user button (SW3) on the dimmer board.  The LED (LED1 on Mesh Evaluation Kit
   / LED 2 on CYW920819EVB-02 Evaluation Kit) on the light bulb side should turn on.
5. Push/release the user button (SW3) on the dimmer board.  The LED on the light bulb
   side should turn off.
6. Push and hold the user button (SW3) on the dimmer board.  The LED on the light bulb
   side should gradually go from Off to On within 4 seconds.
7. Push and hold the user button (SW3) on the dimmer board.  The LED on the light bulb
   side should gradually go from On to Off within 4 seconds.
8. Try pushing and holding button for less than 4 seconds, and all other
   combinations.

Notes
-----
1. The board will factory reset if you press and hold the user button (SW3) on
   the board for more than 15 seconds.
2. The application GATT database is located in -
   bt_sdk-1.x\components\BT-SDK\common\libraries\mesh_app_lib\mesh_app_gatt.c
   If you create a GATT database using Bluetooth Configurator, update the
   GATT database in the location mentioned above.

Project Settings
----------------
Application specific project settings are as below -

MESH_MODELS_DEBUG_TRACES
   Turn on debug trace from Mesh Models library
MESH_CORE_DEBUG_TRACES
   Turn on debug trace from Mesh Core library
MESH_PROVISIONER_DEBUG_TRACES
   Turn on debug trace from Mesh Provisioner library
REMOTE_PROVISION_SRV
   Enable device as Remote Provisioning Server
LOW_POWER_NODE
   Enable device as Low Power Node

-------------------------------------------------------------------------------
