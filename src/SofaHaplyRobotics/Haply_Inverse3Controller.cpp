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

#include "haply.hpp"

namespace sofa::HaplyRobotics
{

using namespace sofa::helper::system::thread;

const int Haply_Inverse3ControllerClass = core::RegisterObject("Driver allowing interfacing with Haptic Haply Robotics Inverse3 device.")
    .add< Haply_Inverse3Controller >()
    ;


//constructeur
Haply_Inverse3Controller::Haply_Inverse3Controller()
    : d_hapticIdentity(initData(&d_hapticIdentity, "hapticIdentity", "Data to store Information received by HW device"))
    , d_positionBase(initData(&d_positionBase, Vec3(0, 0, 0), "positionBase", "Position of the device base in the SOFA scene world coordinates"))
    , d_orientationBase(initData(&d_orientationBase, Quat(0, 0, 0, 1), "orientationBase", "Orientation of the device base in the SOFA scene world coordinates"))    
    , d_scale(initData(&d_scale, 1.0, "scale", "Default scale applied to the Device coordinates"))    
    
    , d_handleButton(initData(&d_handleButton, false, "handleButton", "Bool value showing if handle button is pressed"))
    , d_posDevice(initData(&d_posDevice, "positionDevice", "position of the device end-effector in SOFA frame"))
    , d_rawForceDevice(initData(&d_rawForceDevice, "rawForceDevice", "For debug: raw values sent to the device in the device frame"))
    
    , d_drawDebug(initData(&d_drawDebug, false, "drawDebug", "Parameter to draw debug information"))

    , d_fullBBmins(initData(&d_fullBBmins, "fullBBmins", "min values of the BBox the tool cover in SOFA frame"))
    , d_fullBBmaxs(initData(&d_fullBBmaxs, "fullBBmaxs", "max values of the BBox the tool cover in SOFA frame"))
    , l_forceFeedback(initLink("forceFeedBack", "link to the forceFeedBack component, if not set will search through graph and take first one encountered."))
{
    this->f_listening.setValue(true);
   
    d_hapticIdentity.setReadOnly(true);
    d_hapticIdentity.setGroup("Infos");

    d_posDevice.setReadOnly(true);
    d_handleButton.setReadOnly(true);
    d_rawForceDevice.setReadOnly(true);
    d_posDevice.setGroup("Device Status");
    d_handleButton.setGroup("Device Status");
    d_rawForceDevice.setGroup("Device Status");
}


Haply_Inverse3Controller::~Haply_Inverse3Controller()
{
    msg_info_when(m_logThread, "HapticAvatar_HapticThreadManager") << "kill s_hapticThread";

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


void Haply_Inverse3Controller::init()
{
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

    m_logThread = f_printLog.getValue();

    initDevice();
}


void Haply_Inverse3Controller::bwdInit()
{
    if (m_initDevice) {
        m_deviceReady = createHapticThreads();
    }
}


void Haply_Inverse3Controller::initDevice()
{
    m_initDevice = false;

    // Create the Haply client
    haply::init();

    m_client = std::make_unique <haply::client>();
    
    // Connect to the Haply service
    auto ret = m_client->connect();
    if (ret != haply_ok) {
        msg_error() << "Unable to connect the client: " << haply::retstr_c(ret);
        return;
    }

    // parse the list of devices and print info
    if (f_printLog.getValue())
    {
        std::vector<haply::device_id> list;
        {
            auto result = m_client->device_list();
            if (!result) {
                msg_error() << "Unable to list devices: " << haply::retstr_c(result.error());
                return;
            }
            list = result.move();
        }

        ret = haply_ok;
        msg_info() << "Device number: " << list.size();
        for (auto id : list)
        {
            auto result = m_client->latest(id);
            if (!result) {
                msg_error() << "Unable to retrieve device id: " << id << ", state: " << haply::retstr_c(result.error());
                ret = result.error();
                continue;
            }

            haply::latest latest = result.move();
            auto deviceType = haply::is_inverse3(latest.device) ? "Inverse3" : "Handle";
            auto isMock = latest.device.mock ? "yes" : "no";
            msg_info() << "Device: " << deviceType << " with id: " << id
                << " | tag: " << latest.device.tag_id
                << " | mocks: " << isMock;
        }
    }

    // Get the device ids
    m_idDevice = m_client->device_open_first(haply_device_type_inverse3)
        .unwrap("Unable to connect Inverse3");

    m_idHandle = m_client->device_open_first(haply_device_type_handle)
        .unwrap("Unable to connect Handle");

    std::ostringstream oss;
    oss << "haply::version: " << haply::version() << " | Inverse3 ID : " << m_idDevice << " | Handle ID : " << m_idHandle;
    msg_info() << oss.str();
    d_hapticIdentity.setValue(oss.str());

    // set initDevice to true if Haply client and devices have well be initialized
    m_initDevice = true;
}


void Haply_Inverse3Controller::clearDevice()
{   

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
    msg_info_when(m_logThread, "HapticAvatar_HapticThreadManager") << "Main Haptics thread created for id: " << m_idDevice;

    auto _deviceCtrl = static_cast<Haply_Inverse3Controller*>(p_this);
    
    haply::thread thread{ *_deviceCtrl->m_client };

    // Loop Timer
    long targetSpeedLoop = 1; // Target loop speed: 1ms

    // Use computer tick for timer
    ctime_t refTicksPerMs = CTime::getRefTicksPerSec() / 1000;
    ctime_t targetTicksPerLoop = targetSpeedLoop * refTicksPerMs /4;
    if (m_logThread)
    {
        std::cout << "refTicksPerMs: " << refTicksPerMs << std::endl;
        std::cout << "targetTicksPerLoop: " << targetTicksPerLoop << std::endl;
    }

    int cptLoop = 0;
    ctime_t startTimePrev = CTime::getRefTime();
    ctime_t summedLoopDuration = 0;

    const Vec3& basePosition = _deviceCtrl->d_positionBase.getValue();
    const Quat& baseOrientation = _deviceCtrl->d_orientationBase.getValue();
    const SReal& scale = _deviceCtrl->d_scale.getValue();

    haply::result<bool> result;

    while (!terminateHaptic)
    {
        ctime_t startTime = CTime::getRefTime();
        summedLoopDuration += (startTime - startTimePrev);
        startTimePrev = startTime;

        // compute ForceFeedback
        if (m_simulationStarted)
        {
            // 1. Get the current position of the tool in device frame
            haply::latest latest_inv3 = m_client->latest(m_idDevice).unwrap(
                "Unable to retrieve cursor state");
            auto position = latest_inv3.state.cursor.position;

            // Read the latest_inv3 cached orientation quaternion and button
            // state from the handle thread.
            haply::latest latest_handle = m_client->latest(m_idHandle).unwrap(
                "Unable to retrieve handle state");

            bool button = latest_handle.state.handle.data.haply.button;
            
            // Transform our quaternion into a direction vector representing
// where the handle is pointing.
            // device quaternion with components w, x, y, and z.
            auto q = latest_handle.state.handle.quaternion;

            float s = 0.0;
            for (int i=0; i<4; ++i)
                s += q[i] * q[i];
            s = 1 / s;

            // The handle quaternion has r,x,y,z order.
            Vec3 dir;
            static constexpr size_t r = 0, x = 1, y = 2, z = 3;
            dir[0] = 2 * s * (q[x] * q[y] - q[z] * q[r]);
            dir[1] = 1 - 2 * s * (q[x] * q[x] + q[z] * q[z]);
            dir[2] = 2 * s * (q[y] * q[z] + q[x] * q[r]);

            // 2. Compute the actual position of the tool in SOFA world
            Vec3 pos = { position[0], position[1], position[2] };
            Vec3 posInSWorld = basePosition + baseOrientation.rotate(pos * scale);
            Vec3 forceInSWorld = { 0.0f, 0.0f, 0.0f };

            // compute the forcefeedback given the current device position and 3D SOFA world constraints
            if (m_forceFeedback)
            {
                m_forceFeedback->computeForce(posInSWorld[0], posInSWorld[1], posInSWorld[2], 0, 0, 0, 0, forceInSWorld[0], forceInSWorld[1], forceInSWorld[2]);
            }

            // project the force in the device frame
            Vec3 forceInDevice = baseOrientation.inverseRotate(forceInSWorld);

            bool changed = false;
            for (int i = 0; i < 3; ++i)
            {
                if (fabs(forceInDevice[i]) > 3.f) {
                    forceInDevice[i] = 0.0f; 
                    changed = true;
                }
            }

            if (changed)
                std::cout << baseOrientation.inverseRotate(forceInSWorld) << std::endl;

            // Send the current force value to the device
            auto cursor_force = haply::make_cursor_force(forceInDevice[0], forceInDevice[1], forceInDevice[2]);
            m_client->cursor_set_force(m_idDevice, cursor_force);

            // Copy all data to the hapticData for further copy to SOFA Data 
            m_hapticData.position[0] = position[0];
            m_hapticData.position[1] = position[1];
            m_hapticData.position[2] = position[2];

            m_hapticData.orientation[0] = q[1];
            m_hapticData.orientation[1] = q[2];
            m_hapticData.orientation[2] = q[3];
            m_hapticData.orientation[3] = q[0];

            m_hapticData.force[0] = forceInDevice[0];
            m_hapticData.force[1] = forceInDevice[1];
            m_hapticData.force[2] = forceInDevice[2];
            
            m_hapticData.buttonStatus = button;

            if (m_logThread)
            {
                cptLoop++;
                if (cptLoop % 1000 == 0) 
                {
                    float updateFreq = 1000 * 1000 / ((float)summedLoopDuration / (float)refTicksPerMs); // in Hz
                    std::cout << "Iter: " << cptLoop << " | Average haptic loop frequency " << std::to_string(int(updateFreq)) 
                        << " | pos: [" << m_hapticData.position[0] << ", " << m_hapticData.position[1] << ", " << m_hapticData.position[2] << "]"
                        << " | q: [" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "]"
                        << " | dir: [" << dir[0] << ", " << dir[1] << ", " << dir[2] << "]"
                        << " | F: [" << m_hapticData.force[0] << ", " << m_hapticData.force[1] << ", " << m_hapticData.force[2] << "]"
                        << std::endl;

                    summedLoopDuration = 0;
                }
            }
        }

        ctime_t endTime = CTime::getRefTime();
        ctime_t duration = endTime - startTime;

        // If loop is quicker than the target loop speed. Wait here.
        while (duration < targetTicksPerLoop)
        {
            endTime = CTime::getRefTime();
            duration = endTime - startTime;
        }
    }

    msg_info_when(m_logThread, "Haply_HapticThreadManager") << "Haptics thread END!!";
}


void Haply_Inverse3Controller::CopyData(std::atomic<bool>& terminateCopy, void* p_this)
{
    auto _deviceCtrl = static_cast<Haply_Inverse3Controller*>(p_this);

    // Use computer tick for timer
    ctime_t targetSpeedLoop = 1 ; // Target loop speed: 0.5ms
    ctime_t refTicksPerMs = CTime::getRefTicksPerSec() / 1000;
    ctime_t targetTicksPerLoop = targetSpeedLoop * refTicksPerMs / 4;

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
    
    // Update Data on button status
    d_handleButton.setValue(m_simuData.buttonStatus);

    Quat ori = { m_simuData.orientation[0], m_simuData.orientation[1], m_simuData.orientation[2], m_simuData.orientation[3] };

    Coord& posDevice = sofa::helper::getWriteOnlyAccessor(d_posDevice);
    posDevice.getCenter() = positionBase + orientationBase.rotate(position * scale);
    posDevice.getOrientation() = orientationBase * ori;

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
    if (!d_drawDebug.getValue())
        return;

    // Debug: Draw end effector position as 3D axis
    const Coord& posDevice = d_posDevice.getValue();
    vparams->drawTool()->drawFrame(posDevice.getCenter(), posDevice.getOrientation(), sofa::type::Vec3f(1.0f, 1.0f, 1.0f));
    vparams->drawTool()->drawBoundingBox(d_fullBBmins.getValue(), d_fullBBmaxs.getValue());

    // Debug: Draw force feedback vector
    sofa::type::RGBAColor color4(1.0f, 0.0, 0.0f, 1.0);
    if (d_handleButton.getValue())
    {
        color4 = sofa::type::RGBAColor(0.0f, 1.0, 0.0f, 1.0);
    }
    vparams->drawTool()->drawLine(posDevice.getCenter(), posDevice.getCenter() + d_rawForceDevice.getValue(), color4);
}

} // namespace sofa::HaplyRobotics