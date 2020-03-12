-------------------------------------------------------------------------------
BLE Mesh Dimmer demo application
-------------------------------------------------------------------------------


Overview
--------
This demo application shows a simple dimmer implementation with self
configuration feature. The feature allows fast configuration of
the device. The SELF_CONFIG feature works in concert with the
embedded provisioner. For embedded provisioner, the configuration needs
to be hardcoded and then each device needs to be configured after
provisioning according to hardcoded configuration. For the self
configuration, the rules are hardcoded in a shared embedded_provisioner.h.
After provisioning is the provisioner sends new device the application key,
binds vendor specific model to the keys, and then sends vendor specific
command to self configure. The code snippets under #if defined(SELF_CONFIG)
can be copied to any application to allow the feature. To build the app
use SELF_CONFIG=1 in the build command line, or in the make file.  For example,

demo.mesh.dimmer_self_config-CYW920719Q40EVB_01 SELF_CONFIG=1 download

The app is based on the snip/mesh/mesh_level_client which implements
BLE Mesh Generic Level Client model.
Normally a dimmer has at least 2 buttons to turn the light on and off.
This application performs dimming using a single button available on the
EVK.  On a short push, the level is toggled between 0% and 100%.
When a button is pushed and not released, the level is changed
every 0.5 seconds from 0 to 100 in 8 steps 12.5% each.  The button
can be released before the level reaches 100% and in this case
consecutive push or long push will continue increasing the level.
If level reaches 100% the next button control will decrease the level.

By default application does not support Relay, Proxy or Friend features
It can also be compiled to support a Low Power Node by adding
#define LOW_POWER_NODE  1

Features demonstrated
 - Self configuration of the device
 - Button usage on the CYBT-213043-MESH EZ-BT Mesh Evaluation Kit / CYW920819EVB-02 Evaluation Kit
 - Controlling of a BLE Mesh light bulb using BLE Mesh Set Level messages

See 20819_readme.txt for more information about the BT SDK.

Instructions
------------
To demonstrate the app, work through the following steps.
1. Build and download the application to the Mesh Evaluation Board / CYW920819EVB-02 Evaluation Kit
2. Build and download Mesh_EmbeddedProvisioner application (Mesh Evaluation Board / CYW920819EVB-02 Evaluation Kit)
3. The Embedded Provisioner will automatically provision and will tell this app to self configure
4. Push/release the user button (SW3) on the dimmer board.  The LED (LED1 on Mesh Evaluation Kit
   / LED 2 on CYW920819EVB-02 Evaluation Kit) on the light bulb side should turn on.
5. Push/release the user button (SW3) on the dimmer board.  The LED on the light bulb
   side should turn off.
6. Push and hold the user button (SW3) on the dimmer board.  The LED on the light bulb
   side should gradually go from Off to On within 4 seconds.
7. Push and hold the application button on the dimmer board.  The LED on the light bulb
   side should gradually go from On to Off within 4 seconds.
8. Try pushing and holding button for less than 4 seconds, and all other
   combinations.

Details
-------
By default application does not support Relay, Proxy or Friend features
It can also be compiled to support a Low Power Node by adding
#define LOW_POWER_NODE  1

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
