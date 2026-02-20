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

#include <SofaHaplyRobotics/Haply_HapticThreadManager.h>

#include <sofa/helper/logging/Messaging.h>
#include <sofa/helper/system/thread/CTime.h>
#include <chrono>


namespace sofa::HaplyRobotics
{

using namespace sofa::helper::system::thread;


Haply_HapticThreadManager* Haply_HapticThreadManager::getInstance()
{
    if (s_hapticThread == nullptr)
    {
        s_hapticThread = new Haply_HapticThreadManager();
    }

    return s_hapticThread;
}

void Haply_HapticThreadManager::kill()
{
    if (s_hapticThread != nullptr)
    {
        if (s_hapticThread->logThread)
        {
            msg_warning("Haply_HapticThreadManager") << "kill s_hapticThread";
        }

        //s_hapticThread->m_devices.clear();        
        delete s_hapticThread;
        s_hapticThread = nullptr;
    }
}

Haply_HapticThreadManager::Haply_HapticThreadManager()
    : m_terminate(true)
    , hapticLoopStarted(false)
{

}

Haply_HapticThreadManager::~Haply_HapticThreadManager()
{
    if (m_terminate == false)
    {
        m_terminate = true;
        haptic_thread.join();
    }
}



void Haply_HapticThreadManager::createHapticThreads()
{
    if (hapticLoopStarted)
        return;

    m_terminate = false;
    haptic_thread = std::thread(&Haply_HapticThreadManager::Haptics, this, std::ref(this->m_terminate), this);
    hapticLoopStarted = true;
}

void Haply_HapticThreadManager::Haptics(std::atomic<bool>& terminate, void* p_this)
{
    if (logThread)
        msg_warning("Haply_HapticThreadManager") << "Main Haptics thread created";
    
    Haply_HapticThreadManager* threadMgr = static_cast<Haply_HapticThreadManager*>(p_this);

    // Loop Timer
    long targetSpeedLoop = 1; // Target loop speed: 1ms

    // Use computer tick for timer
    ctime_t refTicksPerMs = CTime::getRefTicksPerSec() / 1000;
    ctime_t targetTicksPerLoop = targetSpeedLoop * refTicksPerMs;

    int cptLoop = 0;
    ctime_t startTimePrev = CTime::getRefTime();
    ctime_t summedLoopDuration = 0;
    
    while (!terminate)
    {
        ctime_t startTime = CTime::getRefTime();
        summedLoopDuration += (startTime - startTimePrev);
        startTimePrev = startTime;

        // loop over the devices
        //for (auto device : m_devices) // mutex?
        //{            
        //    // Force feedback computation
        //    
        //}

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
        duration = 0;
        while (duration < targetTicksPerLoop)
        {
            endTime = CTime::getRefTime();
            duration = endTime - startTime;
        }
    }

    if (logThread)
        msg_warning("Haply_HapticThreadManager") << "Haptics thread END!!";
}


} // namespace sofa::HaplyRobotics
