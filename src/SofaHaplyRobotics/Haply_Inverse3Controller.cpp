/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHaplyRobotics/Haply_Inverse3Controller.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <sofa/simulation/Node.h>
#include <sofa/core/visual/VisualParams.h>
#include <iomanip> 
#include <sofa/core/ObjectFactory.h>

#include <sofa/helper/system/thread/CTime.h>
#include <chrono>

#include <libhv.h>
//#include <external/libhv.h>
#include <nlohmann/json.hpp>
#include <cstdio>
#include <string>
#include <vector>


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
    msg_info_when(m_logThread, "Haply_Inverse3Controller") << "kill s_hapticThread";

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


    // Bounding box computed during execution
    const Quat& orientationBase = d_orientationBase.getValue();
    auto posDevice = d_positionBase.getValue();
    const SReal& scale = d_scale.getValue();

    Vec3 min = { -0.301056, -0.29919, -0.118068 };
    Vec3 max = { 0.285928, 0.16325, 0.377896 };
    min = orientationBase.rotate(min * scale);
    max = orientationBase.rotate(max * scale);

    for (int i = 0; i < 3; ++i)
    {
        min[i] += posDevice[i];
        max[i] += posDevice[i];
    }

    d_fullBBmins.setValue(min);
    d_fullBBmaxs.setValue(max);

    //auto posDevice = d_positionBase.getValue();
    //d_fullBBmins.setValue((Vec3(-0.301056 + posDevice[0], -0.29919 + posDevice[1], -0.118068 + posDevice[2]))* d_scale.getValue());
    //d_fullBBmaxs.setValue(Vec3(0.285928 + posDevice[0], 0.16325 + posDevice[1], 0.377896 + posDevice[2]) * d_scale.getValue());

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

    msg_info() << "haply::inverse::init()";

	lastPrintTime_ = std::chrono::high_resolution_clock::now();
    const auto print_delay = std::chrono::milliseconds(100);
    auto current = std::chrono::high_resolution_clock::now();

    reconn_setting_t reconn;
    reconn_setting_init(&reconn);
    reconn.min_delay = 1000;
    reconn.max_delay = 5000;
    reconn.delay_policy = 2;
    ws.setReconnect(&reconn);

    m_initDevice = true;
}


void Haply_Inverse3Controller::disconnect()
{   
    if (ws.isConnected()) {
        ws.close();
    }
}


bool Haply_Inverse3Controller::createHapticThreads()
{
    //m_terminateHaptic = false;
    //haptic_thread = std::thread(&Haply_Inverse3Controller::Haptics, this, std::ref(this->m_terminateHaptic), this);
    //hapticLoopStarted = true;

    m_terminateCopy = false;
    copy_thread = std::thread(&Haply_Inverse3Controller::CopyData, this, std::ref(this->m_terminateCopy), this);

    ws.onopen = [&]() {
        std::cout << "[MyDeviceDriver] WebSocket opened." << std::endl;
        };

    ws.onmessage = [&](const std::string& msg) {
        HapticsHandling(msg);
        };

    ws.onclose = [&]() {
        std::cout << "[MyDeviceDriver] WebSocket closed." << std::endl;
        };

    msg_info() << "Haply_Inverse3Controller::createHapticThreads()";

    return true;
}


void Haply_Inverse3Controller::connect() {
    ws.open("ws://localhost:10001");
}


void Haply_Inverse3Controller::HapticsHandling(const std::string& msg) {
    json data = json::parse(msg);

    const auto print_delay = std::chrono::milliseconds(1);

    if (data[inverseKey_].empty()) {
        json update_request = {
            {"session", {{"force_render_full_state", json::object()}}}
        };

        std::cout << "[MyDeviceDriver] No devices found. Waiting for connection..." << std::endl;
        std::this_thread::sleep_for(1000ms);
        ws.send(update_request.dump());

        return;
    }

    json request = {};

    // Probe cursor position for each inverse
    // Handle inverse3 device(s)

    // el.value example:
    // "device_id":"04EA",
    // "state":{"angular_position":{"a0":-30.927332,"a1":53.878685,"a2":-6.642623},"angular_velocity":{"a0":0,"a1":0,"a2":0},"body_orientation":{"w":0.69885254,"x":0.0014648438,"y":0.715271,"z":0.0024414062},"control_domain":"undefined","control_mode":"idle","current_angular_position":{"a0":0,"a1":0,"a2":0},"current_angular_torques":{"a0":0,"a1":0,"a2":0},"current_cursor_force":{"x":0,"y":0,"z":0},"current_cursor_position":{"x":0,"y":0,"z":0},"cursor_position":{"x":0.10428079,"y":-0.03648221,"z":-0.06100729},"cursor_velocity":{"x":0,"y":0,"z":0},"mode":"idle","transform":{"position":{"x":0,"y":0,"z":0},"rotation":{"w":1,"x":0,"y":0,"z":0},"scale":{"x":1,"y":1,"z":1}}},"status":{"calibrated":true,"in_use":false,"power_supply":true,"ready":true,"started":true}}


    for (auto& el : data[inverseKey_].items()) 
    {
        //std::cout << "el.value(): "<< el.value() << std::endl;
        std::string deviceId = el.value()[deviceIdKey_];
        request[inverseKey_].push_back({
            {deviceIdKey_, deviceId},
            {"commands", {{"probe_position", json::object()}}}
            });
    }


	// example of grip data
    // grip_id: 61576 -> {"button":false, "hall" : 1, "orientation" : {"w":0.71655273, "x" : 0.19647217, "y" : 0.6290283, "z" : -0.22833252}, 
    // "transform" : {"position":{"x":0, "y" : 0, "z" :    // 0}, "rotation" : {"w":1, "x" : 0, "y" : 0, "z" : 0}, "scale" : {"x":1, "y" : 1, "z" : 1}}}

    if (data.contains(gripIdKey_)) {
		for (auto& el : data[gripIdKey_]) {
			std::string grip_id = el[deviceIdKey_]; // grip_id: 61576
            const json& state = el["state"];
            //std::cout << "grip_id: " << grip_id  << " -> " << state << std::endl;

            float qx = state["orientation"]["x"].get<float>();
            float qy = state["orientation"]["y"].get<float>();
            float qz = state["orientation"]["z"].get<float>();
            float qw = state["orientation"]["w"].get<float>();


			//json orientation = el["state"]["orientation"];
		}
	}

    const auto now = std::chrono::high_resolution_clock::now();

    if (now > lastPrintTime_ + print_delay) {
        lastPrintTime_ = now;

        for (auto& el : data[inverseKey_].items()) 
        {
            const std::string device_id = el.value()[deviceIdKey_];
            const json state = el.value()["state"];

            const float x = state["cursor_position"]["x"].get<float>();
            const float y = state["cursor_position"]["y"].get<float>();
            const float z = state["cursor_position"]["z"].get<float>();
            const float Vx = state["cursor_velocity"]["x"].get<float>();
            const float Vy = state["cursor_velocity"]["y"].get<float>();
            const float Vz = state["cursor_velocity"]["z"].get<float>();

            //std::cout << "Inverse Velocities - x: " << Vx << "\t y: " << Vy << "\t z: " << Vz << std::endl;
			m_hapticData.position[0] = x;
			m_hapticData.position[1] = y;
			m_hapticData.position[2] = z;


            //printf("ID : %s   |    Position : %s   |   Velocity : %s   |   Angles : %s   |   Angular Velocity : %s\n",
            //    device_id.c_str(),
            //    state["cursor_position"].dump().c_str(),
            //    state["cursor_velocity"].dump().c_str(),
            //    state["angular_position"].dump().c_str(),
            //    state["angular_velocity"].dump().c_str());


        }
        //print("\n");
        //fflush(stdout);
        //std::lock_guard<std::mutex> lock(dataMutex_);
        //devices_.clear();

        //for (auto& el : data[inverseKey_].items()) 
        //{
        //    const std::string deviceId = el.value()[deviceIdKey_];
        //    const json state = el.value()["state"];

        //    DeviceState ds;
        //    ds.position = state["cursor_position"].get<std::vector<double>>();
        //    ds.velocity = state["cursor_velocity"].get<std::vector<double>>();
        //    ds.angles = state["angular_position"].get<std::vector<double>>();
        //    ds.angularVelocity = state["angular_velocity"].get<std::vector<double>>();

        //    devices_[deviceId] = ds;

        //    // Optional console output
        //    std::cout << "ID: " << deviceId
        //        << " | Pos: " << state["cursor_position"]
        //        << " | Vel: " << state["cursor_velocity"]
        //        << " | Ang: " << state["angular_position"]
        //        << " | AngVel: " << state["angular_velocity"]
        //        << std::endl;
        //}

        //std::cout << std::endl;
    }

    ws.send(request.dump());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}



std::unordered_map<std::string, Haply_Inverse3Controller::DeviceState> Haply_Inverse3Controller::getDeviceStates() 
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    return devices_;
}


//void Haply_Inverse3Controller::Haptics(std::atomic<bool>& terminateHaptic, void* p_this)
//{



    //msg_info_when(m_logThread, "Haply_Inverse3Controller") << "Main Haptics thread created for id: " << m_idDevice;

    //auto _deviceCtrl = static_cast<Haply_Inverse3Controller*>(p_this);
    //
    //haply::inverse::thread thread{ *_deviceCtrl->m_client };

    //// Loop Timer
    //long targetSpeedLoop = 1; // Target loop speed: 1ms

    //// Use computer tick for timer
    //ctime_t refTicksPerMs = CTime::getRefTicksPerSec() / 1000;
    //ctime_t targetTicksPerLoop = targetSpeedLoop * refTicksPerMs /4;
    //if (m_logThread)
    //{
    //    std::cout << "refTicksPerMs: " << refTicksPerMs << std::endl;
    //    std::cout << "targetTicksPerLoop: " << targetTicksPerLoop << std::endl;
    //}

    //int cptLoop = 0;
    //ctime_t startTimePrev = CTime::getRefTime();
    //ctime_t summedLoopDuration = 0;

    //const Vec3& basePosition = _deviceCtrl->d_positionBase.getValue();
    //const Quat& baseOrientation = _deviceCtrl->d_orientationBase.getValue();
    //const SReal& scale = _deviceCtrl->d_scale.getValue();
    //const SReal& damping = d_dampingForce.getValue();

    //haply::inverse::result<bool> result;
    //Vec3 forceInDevice = { 0.0f, 0.0f, 0.0f };

    //while (!terminateHaptic)
    //{
    //    ctime_t startTime = CTime::getRefTime();
    //    summedLoopDuration += (startTime - startTimePrev);
    //    startTimePrev = startTime;

    //    // compute ForceFeedback
    //    if (m_simulationStarted)
    //    {
    //        // 1. Get the current position of the tool in device frame
    //        haply::inverse::latest latest_inv3 = m_client->latest(m_idDevice).unwrap(
    //            "Unable to retrieve cursor state");
    //        auto position = latest_inv3.state.inverse.state_haptic.cursor.position;
    //        
    //        // Read the latest_inv3 cached orientation quaternion and button
    //        // state from the handle thread.
    //        haply::inverse::latest latest_handle = m_client->latest(m_idHandle).unwrap(
    //            "Unable to retrieve handle state");

    //        bool button = latest_handle.state.verse_grip.data.info.button;
    //        
    //        // The handle device quaternion with components w, x, y, and z.
    //        auto q = latest_handle.state.verse_grip.quaternion;

    //        // 2. Compute the actual position of the tool in SOFA world
    //        Vec3 pos = { position[0], position[1], position[2] };
    //        Vec3 posInSWorld = basePosition + baseOrientation.rotate(pos * scale);
    //        Vec3 forceInSWorld = { 0.0f, 0.0f, 0.0f };

    //        // compute the forcefeedback given the current device position and 3D SOFA world constraints
    //        if (m_forceFeedback)
    //        {
    //            m_forceFeedback->computeForce(posInSWorld[0], posInSWorld[1], posInSWorld[2], 0, 0, 0, 0, forceInSWorld[0], forceInSWorld[1], forceInSWorld[2]);           
    //            //std::cout << "forceInSWorld: " << forceInSWorld[0] << " " << forceInSWorld[1] << " " << forceInSWorld[2];

    //            // project the force in the device frame
    //            forceInDevice = baseOrientation.inverseRotate(forceInSWorld);
    //            //std::cout << " | forceInDevice: " << forceInDevice[0] << " " << forceInDevice[1] << " " << forceInDevice[2];
    //            bool changed = false;
    //            bool isInContact = false;
    //            for (int i = 0; i < 3; ++i)
    //            {
    //                auto forceAbs = fabs(forceInDevice[i]);                    
    //                if (forceAbs > 0.0f) {
    //                    isInContact = true;

    //                    if (forceAbs > 5.f) {
    //                        forceInDevice[i] = 0.0f;
    //                        changed = true;
    //                    }
    //                }
    //            }

    //            if (changed)
    //                std::cout << "Max force: " << baseOrientation.inverseRotate(forceInSWorld) << std::endl;

    //            // Send the current force value to the device
    //            if (isInContact)
    //            {
    //                // Damping is an effective tool for smoothing velocityand thus can mitigate buzzing.A typical damping formula adds a retarding
    //                // force proportional to the velocity of the device
    //                float retardingForce[3] = { -latest_inv3.state.inverse.state_haptic.cursor.velocity[0]*damping, -latest_inv3.state.inverse.state_haptic.cursor.velocity[1] * damping, -latest_inv3.state.inverse.state_haptic.cursor.velocity[2] * damping };
    //                
    //                //std::cout << "retardingForce: " << retardingForce[0] << " " << retardingForce[1] << " " << retardingForce[2] << std::endl;
    //                auto cursor_force = haply::inverse::make_cursor_force(forceInDevice[0] + retardingForce[0], forceInDevice[1] + retardingForce[1], forceInDevice[2] + retardingForce[2]);
    //                m_client->cursor_set_force(m_idDevice, cursor_force);
    //            }
    //            else
    //            {
    //                auto cursor_force = haply::inverse::make_cursor_force(0.0f, 0.0f, 0.0f);
    //                m_client->cursor_set_force(m_idDevice, cursor_force);
    //            }
    //        }

    //        // Copy all data to the hapticData for further copy to SOFA Data 
    //        m_hapticData.position[0] = position[0];
    //        m_hapticData.position[1] = position[1];
    //        m_hapticData.position[2] = position[2];

    //        m_hapticData.orientation[0] = q[1];
    //        m_hapticData.orientation[1] = q[2];
    //        m_hapticData.orientation[2] = q[3];
    //        m_hapticData.orientation[3] = q[0];

    //        m_hapticData.force[0] = forceInDevice[0];
    //        m_hapticData.force[1] = forceInDevice[1];
    //        m_hapticData.force[2] = forceInDevice[2];
    //        
    //        m_hapticData.buttonStatus = button;

    //        if (m_logThread)
    //        {
    //            cptLoop++;
    //            if (cptLoop % 1000 == 0) 
    //            {
    //                float updateFreq = 1000 * 1000 / ((float)summedLoopDuration / (float)refTicksPerMs); // in Hz
    //                std::cout << "Iter: " << cptLoop << " | Average haptic loop frequency " << std::to_string(int(updateFreq)) 
    //                    << " | pos: [" << m_hapticData.position[0] << ", " << m_hapticData.position[1] << ", " << m_hapticData.position[2] << "]"
    //                    << " | q: [" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "]"
    //                    << " | F: [" << m_hapticData.force[0] << ", " << m_hapticData.force[1] << ", " << m_hapticData.force[2] << "]"
    //                    << std::endl;

    //                summedLoopDuration = 0;
    //            }
    //        }
    //    }

    //    ctime_t endTime = CTime::getRefTime();
    //    ctime_t duration = endTime - startTime;

    //    // If loop is quicker than the target loop speed. Wait here.
    //    while (duration < targetTicksPerLoop)
    //    {
    //        endTime = CTime::getRefTime();
    //        duration = endTime - startTime;
    //    }
    //}

    //msg_info_when(m_logThread, "Haply_Inverse3Controller") << "Haptics thread END!!";
//}


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
            std::cout << "START CONNEXION" << std::endl;
			connect();
			hapticLoopStarted = true;
        }

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

    // Debug: Draw device frames
    vparams->drawTool()->drawFrame(d_positionBase.getValue(), d_orientationBase.getValue(), sofa::type::Vec3f(1.0f, 1.0f, 2.0f));

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