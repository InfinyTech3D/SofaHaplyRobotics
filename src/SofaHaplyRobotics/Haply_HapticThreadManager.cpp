/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

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
