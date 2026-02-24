/*****************************************************************************
 *                - Copyright (C) 2022-Present InfinyTech3D -                *
 *                                                                           *
 * This file is part of the SofaHaplyRobotics plugin for the SOFA framework. *
 *                                                                           *
 * This file is dual-licensed:                                               *
 *                                                                           *
 * 1) Commercial License:                                                    *
 *      This file may be used under the terms of a valid commercial license  *
 *      agreement provided wih the software by InfinyTech3D.                 *
 *                                                                           *
 * 2) GNU General Public License (GPLv3) Usage                               *
 *      Alternatively, this file may be used under the terms of the          *
 *      GNU General Public License version 3 as published by the             *
 *      Free Software Foundation: https://www.gnu.org/licenses/gpl-3.0.html  *
 *                                                                           *
 * Contact: contact@infinytech3d.com                                         *
 * Further information: https://infinytech3d.com                             *
 ****************************************************************************/
#include <SofaHaplyRobotics/config.h>

namespace sofa::component
{

//Here are just several convenient functions to help user to know what contains the plugin

extern "C" {
    SOFA_HAPLYROBOTICS_API void initExternalModule();
    SOFA_HAPLYROBOTICS_API const char* getModuleName();
    SOFA_HAPLYROBOTICS_API const char* getModuleVersion();
    SOFA_HAPLYROBOTICS_API const char* getModuleLicense();
    SOFA_HAPLYROBOTICS_API const char* getModuleDescription();
    SOFA_HAPLYROBOTICS_API const char* getModuleComponentList();
}

void initExternalModule()
{
    static bool first = true;
    if (first)
    {
        first = false;
    }
}

const char* getModuleName()
{
    return "SofaHaplyRobotics";
}

const char* getModuleVersion()
{
    return "0.1";
}

const char* getModuleLicense()
{
    return "LGPL";
}


const char* getModuleDescription()
{
    return "Plugin to manage Haply Robotics Inverse3 device in SOFA.";
}

const char* getModuleComponentList()
{
    return "Haply_Inverse3Controller";
}

} // namespace sofa::component
