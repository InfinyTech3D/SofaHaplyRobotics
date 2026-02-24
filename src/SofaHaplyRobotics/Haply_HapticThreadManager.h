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
#pragma once

#include <SofaHaplyRobotics/config.h>
#include <sofa/type/Vec.h>
#include <sofa/type/SVector.h>

#include <string>
#include <mutex>

namespace sofa::HaplyRobotics
{
class Haply_HapticThreadManager;

/// Static singleton of HapticThreadManager
static Haply_HapticThreadManager* s_hapticThread = nullptr;

class SOFA_HAPLYROBOTICS_API Haply_HapticThreadManager
{
public:
    /// Static method to create the HapticThreadManager singleton. Will create it if first time called and return pointer to it.
    static Haply_HapticThreadManager* getInstance();

    /// Static method to kill the HapticThreadManager singleton if @sa s_hapticThread is not null.
    static void kill();


    /// Main Haptic thread methods
    void Haptics(std::atomic<bool>& terminate, void* p_this);

    /// Bool to notify thread to stop work
    std::atomic<bool> m_terminate;

    /// Method to notify that simulation is running
    void setSimulationStarted() { m_simulationStarted = true; }

    bool logThread = true;
private:
    Haply_HapticThreadManager();
    ~Haply_HapticThreadManager();

    /// Internal method to create the haptic thread only once. Will be called each time a device is successfully registered
    void createHapticThreads();

    /// haptic thread c++ object
    std::thread haptic_thread;

    bool hapticLoopStarted = false; ///< Bool to store the information is haptic thread is running or not.
    bool m_simulationStarted = false; ///< Bool to store the information that the simulation is running or not.
};


} // namespace sofa::HaplyRobotics


