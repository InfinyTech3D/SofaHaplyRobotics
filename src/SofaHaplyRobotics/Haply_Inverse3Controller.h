/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

#include <SofaHaplyRobotics/config.h>
#include <sofa/type/Vec.h>
#include <sofa/component/controller/Controller.h>
#include <mutex>
#include <libhv.h>


//force feedback
#include <sofa/component/haptics/ForceFeedback.h>

namespace sofa::HaplyRobotics
{
using namespace sofa::defaulttype;
using namespace sofa::simulation;
using namespace sofa::component::controller;
using namespace sofa::component::haptics;

/**
* Haptic driver
*/
class SOFA_HAPLYROBOTICS_API Haply_Inverse3Controller : public Controller
{
public:
    SOFA_CLASS(Haply_Inverse3Controller, Controller);

    using Coord = sofa::defaulttype::RigidTypes::Coord;
    using VecCoord = sofa::defaulttype::RigidTypes::VecCoord;

    using Vec3 = sofa::type::Vec3d;
    using Quat = sofa::type::Quat<SReal>;


    /// default constructor
    Haply_Inverse3Controller();

    ~Haply_Inverse3Controller() override;

    /// Component API 
    ///{
    void init() override;
    /// SOFA api method called after all components have been init
    void bwdInit() override;
    void handleEvent(core::objectmodel::Event *) override;
    void draw(const sofa::core::visual::VisualParams* vparams) override;
    ///}

    /// Main Haptic thread methods
    void HapticsHandling(const std::string& msg);

    /// Thread methods to cpy data from m_hapticData to m_simuData
    void CopyData(std::atomic<bool>& terminate, void* p_this);
   
    /// Method to notify that simulation is running
    void setSimulationStarted() { m_simulationStarted = true; }

protected:
    /// Internal method to init specific info. Called by init
    virtual void initDevice();
    
    /// Main method to connect to the device
    void connect();

	/// Main method to disconnect from the device
    void disconnect();
    
	/// Method to create and start haptic thread. Will be called by bwdInit
    bool createHapticThreads();

    /// Main method from the SOFA simulation call at each simulation step begin.
    void simulation_updatePosition();

   
public:
    /// Data to store Information received by HW device
    Data<std::string> d_hapticIdentity;

    Data<Vec3> d_positionBase; ///< Input Position of the device base in the scene world coordinates
    Data<Quat> d_orientationBase; ///< Input Orientation of the device base in the scene world coordinates
    Data<SReal> d_scale; ///< Default scale applied to the device Coordinates

    /// Output Data
    Data<bool> d_handleButton; ///< Bool value showing if handle button is pressed
    Data<Coord> d_posDevice; ///< position of the device end-effector in SOFA Frame. Take into account @sa d_positionBase, @sa d_orientationBase and @sa d_scale
    Data<Vec3> d_rawForceDevice; ///< For debug: raw values sent to the device in the device frame
	Data<SReal> d_dampingForce; ///< Damping value, it is a factor applied to the velocity and substracted to force feedback to avoid oscillations. 

    /// Data parameter to draw debug information
    Data<bool> d_drawDebug;


    // Pointer to the forceFeedBack component
    ForceFeedback::SPtr m_forceFeedback;
    // link to the forceFeedBack component, if not set will search through graph and take first one encountered
    SingleLink<Haply_Inverse3Controller, ForceFeedback, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_forceFeedback;


    /// Data public for haptic thread
    /// Structure used to transfer data fromt he haptic thread to the simulation thread.
    struct DeviceData
    {
        float position[3]; // raw position of the end-effector
        float orientation[4]; // raw quaternion of the handle
        float force[3]; // debug raw force vector sent to the device
        bool buttonStatus; // button press status
    };

    /// Data belonging to the haptic thread only
    DeviceData m_hapticData;
    /// Data used in the copy thread to copy @sa m_hapticData into this data that can be used by simulation thread.
    DeviceData m_simuData;

   
private:
    /// Internal parameter to know if device is ready or not.
    bool m_deviceReady = false;
    bool m_initDevice = false;

    bool hapticLoopStarted = false; ///< Bool to store the information is haptic thread is running or not.
    bool m_simulationStarted = false; ///< Bool to store the information that the simulation is running or not.

    bool m_logThread = false;

    /// Bool to notify thread to stop work
    std::atomic<bool> m_terminateHaptic = true;
    std::atomic<bool> m_terminateCopy = true;

    /// haptic thread c++ object
    //std::thread haptic_thread;
    std::thread copy_thread;

	/// WebSocket client
    hv::WebSocketClient m_ws;

	// Todo : protect data with mutex
    //std::mutex dataMutex_;

    static const std::string inverseKey_;
    static const std::string deviceIdKey_;
    static const std::string gripIdKey_;
};

} // namespace sofa::HaplyRobotics
