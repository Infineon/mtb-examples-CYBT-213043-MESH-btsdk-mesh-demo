-------------------------------------------------------------------------------
Mesh Embedded Provisioner application
-------------------------------------------------------------------------------

Overview
--------
This demo application shows an implementation of a self organized BLE mesh
network. One of the devices shall become an embedded provisioner. The device
which becomes and embedded provisioner searches for the unprovisioned devices
and adds them to the network. A network may only have a single embedded
provisioner. To build application, followingflags shall be set to 1 in the
makefile.

SELF_CONFIG?=1
EMBEDDED_PROVISION?=1

The sample application waits for the button push (interrupt) as an indication
that it needs to assume the embedded provisioner role. When swtich to the
embedded provisioner the it creates a network, provisions the local device
into the network and reboots. The next time the app starts up, it is already
provisioned and serves as the provisioner to the network. When a new
unprovisioned device is found, the embedded provisioner check it device
matches required criteria, for example, some pattern in the UUID, and if
matches, provisions and configures the device to the network.

A device which is not set to be an embedded provisioner acts as a normal
BLE Mesh network device. In addition to that it supports a vendor specific
model which allows quicker configuration. If a provisioner sends the vendor
specific command to perform a self config, the device binds all existing
models to an existing app key. And subscribes all the models to a hardcoded
group address (0xC000).

The app uses Remote Provisioning Service feature to add devices to the network.
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

This application also support DFU feature. An MCU can download an
image to the embedded provisioner over WICED HCI UART and tell the app
to perform download to all other devices in the network. The app uses
the BT SIG Mesh DFU procedure to perform the download. At the end of the
DFU the receivers verify the image using ECDSA procedure. To support
this functionality each application shall include ecdsa256_pub.c. The
project directory also contains the private key ecdsa256_key.pri.bin
that was used to generate the public key. The pair needs to be replaced
for production. See tools/ecdsa directory for more information.

Features demonstrated
 - Autoprovisioning and configuration of unprovisioned devices
 - Vendor Specific self configuration
 - BLE Mesh DFU

See chip specific readme for more information about the BT SDK.

Instructions
------------
To demonstrate the app, work through the following steps.
1. Build and download the application to several 213043-MESH Kits.
2. Push and release a button on one of the kits to set it up as an embedded provisioner.

Notes
-----
1. The board will factory reset if you press and hold the user button (SW3) on
   the board for more than 3 seconds.
2. The application GATT database is located in -
   bt_sdk-1.x\components\BT-SDK\common\libraries\mesh_app_lib\mesh_app_gatt.c
   If you create a GATT database using Bluetooth Configurator, update the
   GATT database in the location mentioned above.


-------------------------------------------------------------------------------
