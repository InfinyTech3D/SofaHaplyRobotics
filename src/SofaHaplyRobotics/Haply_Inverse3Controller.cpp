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

#include <sofa/simulation/Node.h>
#include <sofa/core/visual/VisualParams.h>
#include <iomanip> 
#include <sofa/core/ObjectFactory.h>

#include <sofa/helper/system/thread/CTime.h>
#include <chrono>

namespace sofa::HaplyRobotics
{

using namespace sofa::helper::system::thread;

const int Haply_Inverse3ControllerClass = core::RegisterObject("Driver allowing interfacing with Haptic Haply Robotics Inverse3 device.")
    .add< Haply_Inverse3Controller >()
    ;


//constructeur
Haply_Inverse3Controller::Haply_Inverse3Controller()
    : d_portName(initData(&d_portName, std::string("COM8"), "portName", "Name of the port used by this device"))
    , d_hapticIdentity(initData(&d_hapticIdentity, "hapticIdentity", "Data to store Information received by HW device"))
    , d_positionBase(initData(&d_positionBase, Vec3(0, 0, 0), "positionBase", "Position of the device base in the SOFA scene world coordinates"))
    , d_orientationBase(initData(&d_orientationBase, Quat(0, 0, 0, 1), "orientationBase", "Orientation of the device base in the SOFA scene world coordinates"))    
    , d_scale(initData(&d_scale, 1.0, "scale", "Default scale applied to the Device coordinates"))
    , d_drawDebug(initData(&d_drawDebug, false, "drawDebug", "Parameter to draw debug information"))
    , d_posDevice(initData(&d_posDevice, "positionDevice", "position of the base of the part of the device"))
    , d_rawForceDevice(initData(&d_rawForceDevice, "rawForceDevice", "For debug: raw values sent to the device in the device frame"))
    , d_fullBBmins(initData(&d_fullBBmins, "fullBBmins", "min values of the BBox the tool cover in SOFA frame"))
    , d_fullBBmaxs(initData(&d_fullBBmaxs, "fullBBmaxs", "max values of the BBox the tool cover in SOFA frame"))
    , l_forceFeedback(initLink("forceFeedBack", "link to the forceFeedBack component, if not set will search through graph and take first one encountered."))
{
    this->f_listening.setValue(true);
    
    d_hapticIdentity.setReadOnly(true);
}


Haply_Inverse3Controller::~Haply_Inverse3Controller()
{
    msg_info() << "~Haply_Inverse3Controller()";

    if (logThread)
    {
        msg_warning("HapticAvatar_HapticThreadManager") << "kill s_hapticThread";
    }

    if (m_terminateHaptic == false && m_deviceReady)
    {
        m_terminateHaptic = true;
        haptic_thread.join();
    }

    if (m_terminateCopy == false && m_deviceReady)
    {
        m_terminateCopy = true;
        copy_thread.join();
    }

    clearDevice();
}

//executed once at the start of Sofa, initialization of all variables excepts haptics-related ones
void Haply_Inverse3Controller::init()
{
    msg_info() << "Haply_Inverse3Controller::init()";
    
    // Retrieve ForceFeedback component pointer
    if (l_forceFeedback.empty())
    {
        const simulation::Node* context = dynamic_cast<simulation::Node*>(this->getContext()); // access to current node
        m_forceFeedback = context->get<ForceFeedback>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);
    }
    else
    {
        m_forceFeedback = l_forceFeedback.get();
    }

    if (!m_forceFeedback.get())
    {
        msg_warning() << "No forceFeedBack component found in the scene. Only the motion of the haptic tool will be simulated.";
    }

    // Bounding box computed during execution
    d_fullBBmins.setValue(Vec3(-0.301056, -0.29919, -0.118068) * d_scale.getValue());
    d_fullBBmaxs.setValue(Vec3(0.285928, 0.16325, 0.377896) * d_scale.getValue());

    initDevice();
}


void Haply_Inverse3Controller::bwdInit()
{
    msg_info() << "Haply_Inverse3Controller::bwdInit()";

    if (m_initDevice) {
        m_deviceReady = createHapticThreads();
    }
}




void Haply_Inverse3Controller::initDevice()
{
    msg_info() << "Haply_Inverse3Controller::initDevice()";
    const std::string& portName = d_portName.getValue();

    msg_info() << "PortName: " << portName;
    m_stream = new SerialStream(portName.c_str());
    
    // check device connected
    if (!m_stream->isConnected())
    {
        m_initDevice = false;
        msg_error() <<"Opening device " << d_hapticIdentity.getValue() << " on port: " <<  portName << " failed and return code: (" << m_stream->errorConnection() << ")";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }
    
    m_deviceAPI = new Haply::HardwareAPI::Devices::Inverse3(m_stream);
    m_deviceAPI->SendDeviceWakeup();
    m_deviceAPI->SendEndEffectorForce();
    m_deviceAPI->ReceiveDeviceInfo(true);

    m_initDevice = true;
}


void Haply_Inverse3Controller::clearDevice()
{   
    if (m_deviceAPI != nullptr)
    {
        delete m_deviceAPI;
        m_deviceAPI = nullptr;
    }

    if (m_stream != nullptr)
    {
        delete m_stream;
        m_stream = nullptr;
    }
}


bool Haply_Inverse3Controller::createHapticThreads()
{
    m_terminateHaptic = false;
    haptic_thread = std::thread(&Haply_Inverse3Controller::Haptics, this, std::ref(this->m_terminateHaptic), this);
    hapticLoopStarted = true;

    m_terminateCopy = false;
    copy_thread = std::thread(&Haply_Inverse3Controller::CopyData, this, std::ref(this->m_terminateCopy), this);

    return true;
}


void Haply_Inverse3Controller::Haptics(std::atomic<bool>& terminateHaptic, void* p_this)
{
    if (logThread)
        msg_warning("HapticAvatar_HapticThreadManager") << "Main Haptics thread created";

    Haply_Inverse3Controller* _deviceCtrl = static_cast<Haply_Inverse3Controller*>(p_this);
    auto _deviceAPI = _deviceCtrl->m_deviceAPI;

    if (logThread && _deviceAPI)
        msg_info("HapticAvatar_HapticThreadManager") << "_deviceCtrl->m_deviceAPI: OK";


    // Loop Timer
    long targetSpeedLoop = 1; // Target loop speed: 1ms

    // Use computer tick for timer
    ctime_t refTicksPerMs = CTime::getRefTicksPerSec() / 1000;
    ctime_t targetTicksPerLoop = targetSpeedLoop * refTicksPerMs;

    int cptLoop = 0;
    ctime_t startTimePrev = CTime::getRefTime();
    ctime_t summedLoopDuration = 0;

    while (!terminateHaptic)
    {
        ctime_t startTime = CTime::getRefTime();
        summedLoopDuration += (startTime - startTimePrev);
        startTimePrev = startTime;

        // compute ForceFeedback
        if (m_simulationStarted)
        {
            // 1. Get the current position of the tool in device frame
            // as request is needed before retriving info. Start loop with saved position
            Vec3 pos = { m_hapticData.position[0], m_hapticData.position[1], m_hapticData.position[2] };
            
            // 2. Compute the actual position of the tool in SOFA world
            const Vec3& basePosition = _deviceCtrl->d_positionBase.getValue();
            const Quat& baseOrientation = _deviceCtrl->d_orientationBase.getValue();
            const SReal& scale = _deviceCtrl->d_scale.getValue();
            
            Vec3 posInSWorld = basePosition + baseOrientation.rotate(pos * scale);
            Vec3 forceInSWorld = { 0.0f, 0.0f, 0.0f };
            if (m_forceFeedback)
            {
                m_forceFeedback->computeForce(posInSWorld[0], posInSWorld[1], posInSWorld[2], 0, 0, 0, 0, forceInSWorld[0], forceInSWorld[1], forceInSWorld[2]);
            }

            Vec3 forceInDevice = baseOrientation.inverseRotate(forceInSWorld);

            float forceRaw[3] = { float(forceInDevice[0]), float(forceInDevice[1]), float(forceInDevice[2]) };
            float position[3];
            float velocity[3];
            
            // send computed forces
            _deviceAPI->SendEndEffectorForce(forceRaw);

            // retrieve device position
            _deviceAPI->ReceiveEndEffectorState(position, velocity);

            m_hapticData.position[0] = position[0];
            m_hapticData.position[1] = position[1];
            m_hapticData.position[2] = position[2];
            m_hapticData.force[0] = forceRaw[0];
            m_hapticData.force[1] = forceRaw[1];
            m_hapticData.force[2] = forceRaw[2];
        }


        if (logThread)
        {
            cptLoop++;
            if (cptLoop % 1000 == 0) {
                float updateFreq = 1000 * 1000 / ((float)summedLoopDuration / (float)refTicksPerMs); // in Hz
                std::cout << "DeviceName: " << " | Iteration: " << cptLoop << " | Average haptic loop frequency " << std::to_string(int(updateFreq)) << std::endl;
                summedLoopDuration = 0;
            }
        }

        ctime_t endTime = CTime::getRefTime();
        ctime_t duration = endTime - startTime;

        // If loop is quicker than the target loop speed. Wait here.
        //duration = 0;
        while (duration < targetTicksPerLoop)
        {
            endTime = CTime::getRefTime();
            duration = endTime - startTime;
        }
    }

    if (logThread)
        msg_warning("Haply_HapticThreadManager") << "Haptics thread END!!";
}


void Haply_Inverse3Controller::CopyData(std::atomic<bool>& terminateCopy, void* p_this)
{
    Haply_Inverse3Controller* _deviceCtrl = static_cast<Haply_Inverse3Controller*>(p_this);

    // Use computer tick for timer
    ctime_t targetSpeedLoop = 1 / 2; // Target loop speed: 0.5ms
    ctime_t refTicksPerMs = CTime::getRefTicksPerSec() / 1000;
    ctime_t targetTicksPerLoop = targetSpeedLoop * refTicksPerMs;
    double speedTimerMs = 1000 / double(CTime::getRefTicksPerSec());

    ctime_t lastTime = CTime::getRefTime();
    int cptLoop = 0;

    // Haptics Loop
    while (!terminateCopy)
    {
        ctime_t startTime = CTime::getRefTime();
        _deviceCtrl->m_simuData = _deviceCtrl->m_hapticData;

        ctime_t endTime = CTime::getRefTime();
        ctime_t duration = endTime - startTime;

        // If loop is quicker than the target loop speed. Wait here.
        while (duration < targetTicksPerLoop)
        {
            endTime = CTime::getRefTime();
            duration = endTime - startTime;
        }
    }
}


void Haply_Inverse3Controller::simulation_updatePosition()
{
    // get base position and orientation as well as device scale
    const Vec3& positionBase = d_positionBase.getValue();
    const Quat& orientationBase = d_orientationBase.getValue();
    const SReal& scale = d_scale.getValue();
    Vec3 position = { m_simuData.position[0], m_simuData.position[1], m_simuData.position[2] };

    Coord& posDevice = sofa::helper::getWriteOnlyAccessor(d_posDevice);
    posDevice.getCenter() = positionBase + orientationBase.rotate(position * scale);
    posDevice.getOrientation() = orientationBase;

    // for debug dump rawforce
    d_rawForceDevice.setValue(Vec3(m_simuData.force[0], m_simuData.force[1], m_simuData.force[2]));
}


void Haply_Inverse3Controller::handleEvent(core::objectmodel::Event* event)
{
    if (!m_deviceReady)
        return;

    if (dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event))
    {
        m_simulationStarted = true;
        simulation_updatePosition();
    }
}


void Haply_Inverse3Controller::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!m_deviceReady)
        return;

    // If true draw debug information
    if (d_drawDebug.getValue())
    {
        //const Coord& posDevice = d_posDevice.getValue();
        //sofa::type::RGBAColor color4(1.0f, 0.0, 0.0f, 1.0);
        //vparams->drawTool()->drawSphere(posDevice.getCenter(), 1.0f, color4);

        vparams->drawTool()->drawBoundingBox(d_fullBBmins.getValue(), d_fullBBmaxs.getValue());
    }
}

} // namespace sofa::HaplyRobotics