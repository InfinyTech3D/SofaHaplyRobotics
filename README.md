# SofaHaplyRobotics
SOFA plugin for Haply Robotics Inverse3 device

# Notes

When connecting the Device, it is important to put it in the rest position.
did not find a way to calibrate it.

check c++ example: https://gitlab.com/Haply/public/haply-examples/-/blob/1.2.0/04-combined/04-combined.cpp?ref_type=tags

# Step to compile the plugin:

Use: https://develop.haply.co/releases/installer
https://develop.haply.co/releases

Downloaded:
- Inverse Service Installer 3.4.8: https://develop.haply.co/releases/installer
- HardwareAPI c++ 0.2.8: https://develop.haply.co/releases/cpp
- Device Manager 0.14: https://develop.haply.co/releases/manager

Doc: https://docs.haply.co/docs/quick-start/

*TODO*:
- [ ] For now the path to the Haply SDK is hardcoded using CMake variable: HAPLY_PREFIX, improve that
- [ ] Change default value depending on the install: HAPLY_PREFIX = C:/Program Files/Haply/Inverse
- [ ] Right now, the c++ API is searched at: C:\Program Files\Haply\Inverse\include should be placed somewhere else

https://docs.haply.co/hardwareAPI/cpp/ForceFeedback_sample