# SOFA HaplyRobotics Inverse3 Plugin

[![Documentation](https://img.shields.io/badge/doc-on_GitHub-green.svg)](https://github.com/InfinyTech3D/SofaHaplyRobotics/tree/main/doc)
[![Support](https://img.shields.io/badge/support-on_GitHub_Discussions-blue.svg)](https://github.com/sofa-framework/sofa/discussions/categories/haptics)
[![Discord](https://img.shields.io/badge/chat-on_Discord-darkred.svg)](https://discord.gg/G63t3a8Ra6)
[![Contact](https://img.shields.io/badge/contact-on_website-orange.svg)](https://infinytech3d.com/)
[![Support us](https://img.shields.io/badge/support_us-on_Github_Sponsor-purple.svg)](https://github.com/sponsors/InfinyTech3D)

## Description

**Inverse3** by **Haply Robotics** is a compact, high-performance haptic device that provides full 6-DoF motion input with fast, stable force feedback. 
Designed for simulation, teleoperation, and advanced human–machine interaction, it allows users to feel and control virtual objects with exceptional precision. 
Its lightweight design, interchangeable tool grips, and broad software integration support make it ideal for research, training, and interactive biomechanical simulation. 
For further details, please visit the [Haply Robotics website](https://www.haply.co/inverse3).

<img align="center" width="60%" height="auto" src="./doc/Inverse3_picture.jpg">

## Features
The real device is represented virtually in SOFA using rigid dof position of the device. Position, orientation, scale of the device can be changed in the simulation scene.
It can be interfaced with LCPForceFeedBack SOFA component to compute and send force feedback to the device.
During the simulation, 2 haptics thread are used to communicate with the Haptic device at high frequency (~3kHz could be easily increased at 5kHz). It is used to retrieve the tool information and send the force feedback using SOFA lCPForceFeedback mechanism. 
Then the second thread running as well at high frequency is used to copy the tool information into a data which will be used by the simulation thread.

- Compatible with SOFA v25.06 (tested on Windows only, Linux support to be done)
- Support main Inverse3 device, simple Verse Grip tool
- Support of wireless verse grip has been successfully tested but not yet integrated in the plugin (due to hardware availability limitation)
- Multiple demo scenes for 1 device in XML format

*TODO*:
- [ ] Full documentation
- [ ] Support of wireless verse grip integrated in the plugin
- [ ] More demo scenes in python and XML format
- [ ] Support of 2 devices
- [ ] Linux support
- [ ] Support of tool grips changed during simulation
- [ ] Add regression_test using a haptic emulator
- [ ] Add full visualisation of the haptic device in the scene


<img align="center" width="60%" height="auto" src="./doc/device_integration_01.jpg">
Example of Inverse3 device integrated in InfinyTech3D Ultrasound demo (not provided asis in the plugin)

**Integration into SOFA-Unity is available on request**. See More information [here](https://infinytech3d.com/assets-haptic-inverse3/)


## Installation and Setup

### Setup the device on your computer (Windows):

First install the Haply Inverse3 device using the [Haply Inverse3 Installer](https://develop.haply.co/releases/installer) which will install de device deriver on your computer.
Then connect the device to your computer using the provided USB cable and use the [Haply Device Manager](https://develop.haply.co/releases/manager) to check that the device is well recognized by your computer.

Tested here with: 
- Inverse Service Installer 3.4.8: https://develop.haply.co/releases/installer
- Device Manager 0.14: https://develop.haply.co/releases/manager

Full documentation about the device installation and setup is available on the [Haply Develop website](https://develop.haply.co/) or directly in the [online documentation](https://docs.haply.co/docs/quick-start/).

### Step to compile the plugin:
- Download this plugin into your SOFA plugin external_directories folder
- Use CMake in SOFA register this path into your SOFA_EXTERNAL_DIRECTORIES variable
- Configure and generate the SOFA solution using CMake
- Compile SOFA solution (the plugin will be compiled as well)


## Architecture
- **doc:** 
	- Documentation and screenshots of the exampples
- **scenes:**
    - Various simple demo scenes
    - Mesh needed by simulations
- **src/SofaHaplyRobotics:**
	- source code of the device SOFA component. More class to be added to support others devices from Haply Robotics in the future
- **external:**
	- Haply Robotics C++ headers needed to communicate with the device

### Scene Examples
Several examples ready to be used are provided in the plugin under the scenes folder. Below are short descriptions and illustrations of them.
Note that the illustrations have been recorded from the Unity integration. Therefore, the rendering might vary from your setup but not the behavior nor the haptic force feedback feeling.  
- **Inverse3-01-DeviceMotion:** This first example is just showing how the virtual 3D device is mapped to the real device. This is the minimal scene to test if the hardware and all software are well configured. The virtual device should follow the move the Inverse3 + the Verse Grip.


<table>
<tr>
<th style="width:10%"><img align="center" height="auto" src="https://github.com/InfinyTech3D/SofaHaplyRobotics/blob/add_doc_examples/doc/Inverse3-02-RigidSphere.gif"></th>
<th style="width:50%"><img align="center" height="auto"  src="https://github.com/InfinyTech3D/SofaHaplyRobotics/blob/add_doc_examples/doc/Inverse3-03-DeformableSphere.gif"></th>
</tr>
</table>

|||
|--|--|
| **Inverse3-02-RigidSphere:** Interaction between the Inverse3 and a rigid sphere | **Inverse3-03-DeformableSphere:** Interaction between the Inverse3 and a deformable sphere simulated using FEM|

|<img align="center" width="45%" height="auto" src="https://github.com/InfinyTech3D/SofaHaplyRobotics/blob/add_doc_examples/doc/Inverse3-04-RigidCube.gif">|<img align="center" height="auto" width="45%" src="https://github.com/InfinyTech3D/SofaHaplyRobotics/blob/add_doc_examples/doc/Inverse3-05-DeformableCubes.gif">|
|--|--|
| **Inverse3-04-RigidCube:** Interaction between the Inverse3 and a rigid Cube | **Inverse3-05-DeformableCubes:** Interaction between the Inverse3 and 2 deformable Cubes simulated using FEM with different stiffness |

|<img align="center" width="45%" height="auto" src="https://github.com/InfinyTech3D/SofaHaplyRobotics/blob/add_doc_examples/doc/Inverse3-06-RigidSkull.gif">|<img align="center" width="45%" height="auto" src="https://github.com/InfinyTech3D/SofaHaplyRobotics/blob/add_doc_examples/doc/Inverse3-07-DeformableLiver.gif">|
|--|--|
| **Inverse3-06-RigidSkull:** A more advanced simulation interaction between a rigid skull with sharp edges and the Inverse3 | **Inverse3-07-DeformableLiver:** This example shows an advanced simulation between a 3D mesh of a liver simulated using FEM and the Inverse3 |


## License
This work is dual-licensed under either [GPL](https://github.com/InfinyTech3D/SofaHaplyRobotics/blob/main/LICENSE.md) or Commercial License. 
For commercial license request, please contact us by email at contact@infinytech3d.com

