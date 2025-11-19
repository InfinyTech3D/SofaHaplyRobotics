/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHaplyRobotics/Haply_Inverse3Controller.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/Node.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>

#include <sofa/helper/system/thread/CTime.h>

#include <libhv.h>
#include <nlohmann/json.hpp>


using namespace std::chrono_literals;
using namespace hv;
using json = nlohmann::json;


namespace sofa::HaplyRobotics
{

// Static constants
const std::string Haply_Inverse3Controller::inverseKey_ = "inverse3";
const std::string Haply_Inverse3Controller::deviceIdKey_ = "device_id";
const std::string Haply_Inverse3Controller::gripIdKey_ = "verse_grip";


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
    , d_dampingForce(initData(&d_dampingForce, 0.0001, "damping", "Default damping applied to the force feedback"))
    
    , d_drawDebug(initData(&d_drawDebug, false, "drawDebug", "Parameter to draw debug information"))
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
    msg_info_when(m_logThread, "Haply_Inverse3Controller") << "kill s_hapticThread";

    //if (m_terminateHaptic == false && m_deviceReady)
    //{
    //    m_terminateHaptic = true;
    //    haptic_thread.join();
    //}

    if (m_terminateCopy == false && m_deviceReady)
    {
        m_terminateCopy = true;
        copy_thread.join();
    }

    disconnect();
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
    auto current = std::chrono::high_resolution_clock::now();

    reconn_setting_t reconn;
    reconn_setting_init(&reconn);
    reconn.min_delay = 1000;
    reconn.max_delay = 5000;
    reconn.delay_policy = 2;

    m_ws = std::make_unique<WebSocketClient>();
    m_ws->setReconnect(&reconn);

    m_initDevice = true;
}


void Haply_Inverse3Controller::disconnect()
{   
    if (m_ws->isConnected()) {
        m_ws->close();
    }

    if (m_ws != nullptr)
    {
        m_ws.release();
    }
}


bool Haply_Inverse3Controller::createHapticThreads()
{
    msg_info() << "Haply_Inverse3Controller::createHapticThreads()";

    m_terminateCopy = false;
    copy_thread = std::thread(&Haply_Inverse3Controller::CopyData, this, std::ref(this->m_terminateCopy), this);

    m_ws->onopen = [&]() {
        std::cout << "[MyDeviceDriver] WebSocket opened." << std::endl;
        };

    m_ws->onmessage = [&](const std::string& msg) {
        HapticsHandling(msg);
        };

    m_ws->onclose = [&]() {
        std::cout << "[MyDeviceDriver] WebSocket closed." << std::endl;
        };

    return true;
}


void Haply_Inverse3Controller::connect() {
    m_ws->open("ws://localhost:10001");
}


void Haply_Inverse3Controller::HapticsHandling(const std::string& msg) 
{
    // Loop Timer
    long targetSpeedLoop = 0.33; // Target loop speed: (~0.33 s at 3 kHz)

    // Use computer tick for timer
    static ctime_t refTicksPerMs = CTime::getRefTicksPerSec() / 1000;
    static ctime_t targetTicksPerLoop = targetSpeedLoop * refTicksPerMs;

    static int cptLoop = 0;
    ctime_t startTime = CTime::getRefTime();
    ctime_t summedLoopDuration = 0;
    
    json data = json::parse(msg);

    // Todo check if this is called at the haptic speed
    const Vec3& basePosition = d_positionBase.getValue();
    const Quat& baseOrientation = d_orientationBase.getValue();
    const SReal& scale = d_scale.getValue();
    const float damping = float(d_dampingForce.getValue());

    if (data[inverseKey_].empty()) {
        json update_request = {
            {"session", {{"force_render_full_state", json::object()}}}
        };

        std::cout << "[MyDeviceDriver] No devices found. Waiting for connection..." << std::endl;
        std::this_thread::sleep_for(1000ms);
        m_ws->send(update_request.dump());

        return;
    }

    json request = {};

    // Probe cursor position for each inverse
    // Handle inverse3 device(s)
    // TODO check what are the differnt request possible
    for (auto& el : data[inverseKey_].items()) 
    {
        std::string deviceId = el.value()[deviceIdKey_];
        request[inverseKey_].push_back({
            {deviceIdKey_, deviceId},
            {"commands", {{"probe_position", json::object()}}}
            });
    }


    // example of inverse3 data
    // "device_id":"04EA",
    // "state":{"angular_position":{"a0":-30.927332,"a1":53.878685,"a2":-6.642623},"angular_velocity":{"a0":0,"a1":0,"a2":0},"body_orientation":{"w":0.69885254,"x":0.0014648438,"y":0.715271,"z":0.0024414062},"control_domain":"undefined","control_mode":"idle","current_angular_position":{"a0":0,"a1":0,"a2":0},"current_angular_torques":{"a0":0,"a1":0,"a2":0},"current_cursor_force":{"x":0,"y":0,"z":0},"current_cursor_position":{"x":0,"y":0,"z":0},"cursor_position":{"x":0.10428079,"y":-0.03648221,"z":-0.06100729},"cursor_velocity":{"x":0,"y":0,"z":0},"mode":"idle","transform":{"position":{"x":0,"y":0,"z":0},"rotation":{"w":1,"x":0,"y":0,"z":0},"scale":{"x":1,"y":1,"z":1}}},"status":{"calibrated":true,"in_use":false,"power_supply":true,"ready":true,"started":true}}
    for (auto& el : data[inverseKey_].items()) 
    {
        const std::string device_id = el.value()[deviceIdKey_];
        const json state = el.value()["state"];

        // 1. Get the current position of the tool in device frame
        const float x = state["cursor_position"]["x"].get<float>();
        const float y = state["cursor_position"]["y"].get<float>();
        const float z = state["cursor_position"]["z"].get<float>();
        const float Vx = state["cursor_velocity"]["x"].get<float>();
        const float Vy = state["cursor_velocity"]["y"].get<float>();
        const float Vz = state["cursor_velocity"]["z"].get<float>();

		m_hapticData.position[0] = x;
		m_hapticData.position[1] = y;
		m_hapticData.position[2] = z;

        // compute ForceFeedback
        if (m_simulationStarted)
        {
            // 2. Compute the actual position of the tool in SOFA world
            Vec3 pos = { m_hapticData.position[0], m_hapticData.position[1], m_hapticData.position[2] };
            Vec3 posInSWorld = basePosition + baseOrientation.rotate(pos * scale);
            Vec3 forceInSWorld = { 0.0f, 0.0f, 0.0f };
            Vec3 forceInDevice = { 0.0f, 0.0f, 0.0f };

            // compute the forcefeedback given the current device position and 3D SOFA world constraints
            if (m_forceFeedback)
            {
                m_forceFeedback->computeForce(posInSWorld[0], posInSWorld[1], posInSWorld[2], 0, 0, 0, 0, forceInSWorld[0], forceInSWorld[1], forceInSWorld[2]);

                // project the force in the device frame
                forceInDevice = baseOrientation.inverseRotate(forceInSWorld);
                bool changed = false;
                bool isInContact = false;
                for (int i = 0; i < 3; ++i)
                {
                    auto forceAbs = fabs(forceInDevice[i]);
                    if (forceAbs > 0.0f) {
                        isInContact = true;

                        if (forceAbs > 5.f) {
                            forceInDevice[i] = 0.0f;
                            changed = true;
                        }
                    }
                }

                if (changed)
                    msg_warning() << "Max force reached: " << baseOrientation.inverseRotate(forceInSWorld);

                json forces_obj = { {"x", 0}, {"y", 0}, {"z", 0} };
                // Send the current force value to the device
                if (isInContact)
                {
                    // Damping is an effective tool for smoothing velocityand thus can mitigate buzzing.A typical damping formula adds a retarding
                    // force proportional to the velocity of the device
                    float retardingForce[3] = { -Vx * damping, -Vy * damping, -Vz * damping };

                    forces_obj = { {"x", forceInDevice[0] + retardingForce[0]}, {"y", forceInDevice[1] + retardingForce[1]}, {"z", forceInDevice[2] + retardingForce[2]} };
                }

                request[inverseKey_].push_back({
                    {deviceIdKey_, device_id},
                    {"commands", {{"set_cursor_force", {{"values", forces_obj}}}}}
                });
            }

            // copy force for debug draw
            m_hapticData.force[0] = forceInDevice[0];
            m_hapticData.force[1] = forceInDevice[1];
            m_hapticData.force[2] = forceInDevice[2];
        }
    }

    if (data.contains(gripIdKey_)) 
    {
        // example of grip data
        // grip_id: 61576 -> {"button":false, "hall" : 1, "orientation" : {"w":0.71655273, "x" : 0.19647217, "y" : 0.6290283, "z" : -0.22833252}, 
        // "transform" : {"position":{"x":0, "y" : 0, "z" :    // 0}, "rotation" : {"w":1, "x" : 0, "y" : 0, "z" : 0}, "scale" : {"x":1, "y" : 1, "z" : 1}}}
        for (auto& el : data[gripIdKey_]) 
        {
            std::string grip_id = el[deviceIdKey_];
            const json& state = el["state"];

            float qx = state["orientation"]["x"].get<float>();
            float qy = state["orientation"]["y"].get<float>();
            float qz = state["orientation"]["z"].get<float>();
            float qw = state["orientation"]["w"].get<float>();

            m_hapticData.orientation[0] = qx;
            m_hapticData.orientation[1] = qy;
            m_hapticData.orientation[2] = qz;
            m_hapticData.orientation[3] = qw;
            m_hapticData.buttonStatus = state["button"].get<bool>();
        }
    }

    m_ws->send(request.dump());

    cptLoop++;
    ctime_t endTime = CTime::getRefTime();
    ctime_t duration = endTime - startTime;
	summedLoopDuration += duration;

    // If loop is quicker than the target loop speed. Wait here.
    while (duration < targetTicksPerLoop)
    {
        endTime = CTime::getRefTime();
        duration = endTime - startTime;
    }

    if (m_logThread)
    {
        if (cptLoop % 1000 == 0) {
            float updateFreq = 1000 * 1000 / ((float)summedLoopDuration); // in Hz
            msg_info() << "DeviceName: " << " | Iteration: " << cptLoop << " | Average haptic loop frequency " << std::to_string(int(updateFreq)) << " Hz";
            summedLoopDuration = 0;
        }
    }
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
        if (hapticLoopStarted == false)
        {
            msg_info() << "Haply_Inverse3Controller Start device communication at: ws://localhost:10001";
			connect();
			hapticLoopStarted = true;
        }

        m_simulationStarted = true;
        simulation_updatePosition();
    }
}


void Haply_Inverse3Controller::draw(const sofa::core::visual::VisualParams* vparams)
{
    // If true draw debug information
    if (!m_deviceReady || !d_drawDebug.getValue())
        return;

    // Debug: Draw device frames
    const SReal scale = d_scale.getValue();
    vparams->drawTool()->drawFrame(d_positionBase.getValue(), d_orientationBase.getValue(), sofa::type::Vec3f(0.1f * scale, 0.1f * scale, 0.1f * scale));

    // Debug: Draw end effector position as 3D axis
    const Coord& posDevice = d_posDevice.getValue();	
    vparams->drawTool()->drawFrame(posDevice.getCenter(), posDevice.getOrientation(), sofa::type::Vec3f(0.05f * scale, 0.05f * scale, 0.05f * scale));

    // Debug: Draw force feedback vector
    sofa::type::RGBAColor color4(1.0f, 0.0, 0.0f, 1.0);
    if (d_handleButton.getValue())
    {
        color4 = sofa::type::RGBAColor(0.0f, 1.0, 0.0f, 1.0);
    }
    vparams->drawTool()->drawLine(posDevice.getCenter(), posDevice.getCenter() + d_rawForceDevice.getValue(), color4);
}

} // namespace sofa::HaplyRobotics