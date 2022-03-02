/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
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
