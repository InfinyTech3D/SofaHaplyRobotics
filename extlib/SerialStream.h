#pragma once

#include "Stream.h"
#include "serialib.h"
#include <iostream>
#include <iomanip>
#include <sofa/helper/system/thread/CTime.h>

class SerialStream : public Haply::HardwareAPI::IO::Stream {
    const char* address;
    serialib serial;
    bool connected = false;
    int errorValue = 0;
public:
    SerialStream(const char* address) {
        this->address = address;

        char errorOpening = this->serial.openDevice(address, 115200);

        if (errorOpening != 1) {
            errorValue = int(errorOpening);
            std::cout << "Error opening device (code: " << std::dec << int(errorOpening) << ")" << std::endl;
            connected = false;
        }
        else {
            connected = true;
        }

        unsigned char d_buffer[4096];
        unsigned char d_n;
        using ctime = sofa::helper::system::thread::ctime_t;
        
        ctime timeSecuMs = 5000; // loop time secu: 5s
        ctime refTicksPerMs = sofa::helper::system::thread::CTime::getRefTicksPerSec() / 1000;
        ctime nbrTicksSecu = timeSecuMs * refTicksPerMs;

        ctime startTime = sofa::helper::system::thread::CTime::getRefTime();
        ctime duration = sofa::helper::system::thread::CTime::getRefTime() - startTime;

        do
        {
            d_n = this->serial.readBytes(d_buffer, 4096, 1000U, 100U);
            duration = sofa::helper::system::thread::CTime::getRefTime() - startTime;
        } while (d_n > 0 && duration < nbrTicksSecu);


#if defined (_DEBUG)
        std::cout << "Opened device: " << address << std::endl;
#endif
    }

    bool isConnected() const { return connected; }
    int errorConnection() const { return errorValue; }

    ~SerialStream() {
        serial.closeDevice();

#if defined (_DEBUG)
        std::cout << "Closed device: " << this->address << std::endl;
#endif
    }

    int ReadBytes(unsigned char* buffer, int n)
    {
        return serial.readBytes(buffer, n, 1000U, 100U);
    }

    char ReadByte()
    {
        char c;
        this->serial.readChar(&c, 1000U);
        return c;
    }

    void WriteBytes(unsigned char* buffer, int n)
    {
        this->serial.writeBytes(buffer, n);
    }
};