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
#include <Inverse3.h>
#include <SerialStream.h>
#include <mutex>

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

    //using Coord = Vec1Types::Coord;
    //using VecCoord = Vec1Types::VecCoord;
    //using VecDeriv = Vec1Types::VecDeriv;
    using Coord = sofa::defaulttype::RigidTypes::Coord;
    using VecCoord = sofa::defaulttype::RigidTypes::VecCoord;

    using Vec3 = sofa::type::Vec3d;
    using Quat = sofa::type::Quat<SReal>;

    /// default constructor
    Haply_Inverse3Controller();

    virtual ~Haply_Inverse3Controller();

    /// Component API 
    ///{
    void init() override;
    /// SOFA api method called after all components have been init
    void bwdInit() override;
    void handleEvent(core::objectmodel::Event *) override;
    void draw(const sofa::core::visual::VisualParams* vparams) override;
    ///}

    /// Main Haptic thread methods
    void Haptics(std::atomic<bool>& terminate, void* p_this);

    /// Thread methods to cpy data from m_hapticData to m_simuData
    void CopyData(std::atomic<bool>& terminate, void* p_this);
   
    /// Method to notify that simulation is running
    void setSimulationStarted() { m_simulationStarted = true; }

protected:
    /// Internal method to init specific info. Called by init
    virtual void initDevice();
    
    /// Main method to clear the device
    virtual void clearDevice();

    bool createHapticThreads();

    /// Main method from the SOFA simulation call at each simulation step begin.
    void simulation_updatePosition();
   
    void updateButtonStates() {}

public:
    /// Name of the port for this device
    Data<std::string> d_portName; 
    /// Data to store Information received by HW device
    Data<std::string> d_hapticIdentity;

    Data<Vec3> d_positionBase; ///< Input Position of the device base in the scene world coordinates
    Data<Quat> d_orientationBase; ///< Input Orientation of the device base in the scene world coordinates
    Data<SReal> d_scale; ///< Default scale applied to the device Coordinates

    /// Data parameter to draw debug information
    Data<bool> d_drawDebug;

    //Output Data
    Data<Coord> d_posDevice; ///< position of the base of the part of the device
    Vec3 forceDevice;

    Vec3 FullBBmins = { 10000, 10000, 10000 };
    Vec3 FullBBmaxs = { -10000, -10000, -10000 };

    Vec3 ToolBBmins = { 10000, 10000, 10000 };
    Vec3 ToolBBmaxs = { -10000, -10000, -10000 };

    // Pointer to the forceFeedBack component
    ForceFeedback::SPtr m_forceFeedback;
    // link to the forceFeedBack component, if not set will search through graph and take first one encountered
    SingleLink<Haply_Inverse3Controller, ForceFeedback, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_forceFeedback;


    /// Data public for haptic thread
    /// Structure used to transfer data fromt he haptic thread to the simulation thread.
    struct DeviceData
    {
        float position[3];
        Vec3 force;
        sofa::type::fixed_array<float, 3> motorValues;
    };

    /// Data belonging to the haptic thread only
    DeviceData m_hapticData;
    /// Data used in the copy thread to copy @sa m_hapticData into this data that can be used by simulation thread.
    DeviceData m_simuData;

    Haply::HardwareAPI::Devices::Inverse3* m_deviceAPI = nullptr;

protected:
    /// Internal parameter to know if device is ready or not.
    bool m_deviceReady = false;

    bool hapticLoopStarted = false; ///< Bool to store the information is haptic thread is running or not.
    bool m_simulationStarted = false; ///< Bool to store the information that the simulation is running or not.

    bool logThread = true;

    /// Bool to notify thread to stop work
    std::atomic<bool> m_terminateHaptic = true;
    std::atomic<bool> m_terminateCopy = true;

    /// haptic thread c++ object
    std::thread haptic_thread;

    std::thread copy_thread;

    SerialStream* m_stream;

};

} // namespace sofa::HaplyRobotics
