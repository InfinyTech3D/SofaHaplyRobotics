/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

#include <SofaHaplyRobotics/config.h>
#include <sofa/type/Vec.h>
#include <SofaUserInteraction/Controller.h>

//force feedback
#include <SofaHaptics/ForceFeedback.h>

namespace sofa::HaplyRobotics
{
using namespace sofa::defaulttype;
using namespace sofa::simulation;
using namespace sofa::component::controller;

/**
* Haptic driver
*/
class SOFA_HAPLYROBOTICS_API Haply_Inverse3Controller : public Controller
{
public:
    SOFA_CLASS(Haply_Inverse3Controller, Controller);

    using Coord = Vec1Types::Coord;
    using VecCoord = Vec1Types::VecCoord;
    using VecDeriv = Vec1Types::VecDeriv;

    using Vec3 = sofa::type::Vec3d;
    using Quat = sofa::type::Quat<SReal>;

    /// default constructor
    Haply_Inverse3Controller();

    /// Component API 
    ///{
    void init() override;
    void handleEvent(core::objectmodel::Event *) override;
    void draw(const sofa::core::visual::VisualParams* vparams) override;
    ///}
   
protected:
    /// Internal method to init specific info. Called by init
    virtual void initDevice();
    
    /// Main method to clear the device
    virtual void clearDevice() {};

    /// Main method from the SOFA simulation call at each simulation step begin.
    virtual void simulation_updateData() {};

    
    /// Internal method to bo overriden by child class to draw specific information. Called by @sa draw
    virtual void drawImpl(const sofa::core::visual::VisualParams* vparams) { SOFA_UNUSED(vparams); }

    /// Internal method to bo overriden by child class to draw debug information. Called by @sa draw if d_drawDebug is true
    virtual void drawDebug(const sofa::core::visual::VisualParams* vparams) { SOFA_UNUSED(vparams); }

protected:
    void updatePosition() {}

    void updateButtonStates() {}

public:
    /// Name of the port for this device
    Data<std::string> d_portName; 
    /// Data to store Information received by HW device
    Data<std::string> d_hapticIdentity;

    Data<Vec3> d_positionBase; ///< Input Position of the device base in the scene world coordinates
    Data<Quat> d_orientationBase; ///< Input Orientation of the device base in the scene world coordinates

    Data<SReal> d_scale; ///< Default scale applied to the device Coordinates
    Data<SReal> d_forceScale; ///< Default forceScale applied to the force feedback. 

    /// Data parameter to draw debug information
    Data<bool> d_drawDebug;

    //Output Data
    Data<Coord> d_posDevice; ///< position of the base of the part of the device

    // Pointer to the forceFeedBack component
    ForceFeedback::SPtr m_forceFeedback;
    // link to the forceFeedBack component, if not set will search through graph and take first one encountered
    SingleLink<Haply_Inverse3Controller, ForceFeedback, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_forceFeedback;

protected:
    /// Internal parameter to know if device is ready or not.
    bool m_deviceReady = false;


};

} // namespace sofa::HaplyRobotics
