/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHaplyRobotics/Haply_Inverse3Controller.h>
#include <SofaHaplyRobotics/Haply_HapticThreadManager.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <sofa/core/visual/VisualParams.h>
#include <iomanip> 
#include <sofa/core/ObjectFactory.h>



namespace sofa::HaplyRobotics
{

int Haply_Inverse3ControllerClass = core::RegisterObject("Driver allowing interfacing with Haptic Haply Robotics Inverse3 device.")
    .add< Haply_Inverse3Controller >()
    ;


//constructeur
Haply_Inverse3Controller::Haply_Inverse3Controller()
    : d_portName(initData(&d_portName, std::string("//./COM3"), "portName", "Name of the port used by this device"))
    , d_hapticIdentity(initData(&d_hapticIdentity, "hapticIdentity", "Data to store Information received by HW device"))
    , d_positionBase(initData(&d_positionBase, Vec3(0, 0, 0), "positionBase", "Position of the device base in the SOFA scene world coordinates"))
    , d_orientationBase(initData(&d_orientationBase, Quat(0, 0, 0, 1), "orientationBase", "Orientation of the device base in the SOFA scene world coordinates"))
    , d_drawDebug(initData(&d_drawDebug, false, "drawDebugForce", "Parameter to draw debug information"))
    , d_scale(initData(&d_scale, 1.0, "scale", "Default scale applied to the Device coordinates"))
    , d_forceScale(initData(&d_forceScale, 1.0, "forceScale", "Default forceScale applied to the force feedback. "))
    , d_posDevice(initData(&d_posDevice, "positionDevice", "position of the base of the part of the device"))
    , l_forceFeedback(initLink("forceFeedBack", "link to the forceFeedBack component, if not set will search through graph and take first one encountered."))
{
    this->f_listening.setValue(true);
    this->f_printLog.setValue(true);
    
    d_hapticIdentity.setReadOnly(true);
}


//executed once at the start of Sofa, initialization of all variables excepts haptics-related ones
void Haply_Inverse3Controller::init()
{
    msg_info() << "Haply_Inverse3Controller::init()";
    initDevice();
}


void Haply_Inverse3Controller::initDevice()
{
    
}


void Haply_Inverse3Controller::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!m_deviceReady)
        return;

    // draw internal specific info
    drawImpl(vparams);

    // If true draw debug information
    if (d_drawDebug.getValue())
    {
        drawDebug(vparams);
    }
}

void Haply_Inverse3Controller::handleEvent(core::objectmodel::Event *event)
{
    if (!m_deviceReady)
        return;

    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        Haply_HapticThreadManager::getInstance()->setSimulationStarted();
        simulation_updateData();
    }
}

} // namespace sofa::HaplyRobotics