#pragma once

#include "Stream.h"
#include "serialib.h"
#include <iostream>
#include <iomanip>

class SerialStream : public Haply::HardwareAPI::IO::Stream {
    const char* address;
    serialib serial;
public:
    SerialStream(const char* address) {
        this->address = address;

        char errorOpening = this->serial.openDevice(address, 115200);

        if (errorOpening != 1)
            std::cout << "Error opening device (code: " << std::dec << int(errorOpening) << ")" << std::endl;

        unsigned char d_buffer[4096];
        unsigned char d_n;

        do
        {
            d_n = this->serial.readBytes(d_buffer, 4096, 1000U, 100U);
        } while (d_n > 0);

#if defined (_DEBUG)
        std::cout << "Opened device: " << address << std::endl;
#endif
    }

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