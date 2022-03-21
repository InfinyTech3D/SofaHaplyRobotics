#include "Inverse3.h"
#include <iostream>
#include <iomanip>

void Haply::HardwareAPI::Devices::Inverse3::WriteBytes(int n)
{
	this->stream->WriteBytes(this->w_buffer, n);
}

char Haply::HardwareAPI::Devices::Inverse3::ReadHeaderCode()
{
	return this->stream->ReadByte();
}

int Haply::HardwareAPI::Devices::Inverse3::ReadBytes(int n)
{
	return this->stream->ReadBytes(this->r_buffer, n);
}

#if defined (_DEBUG)
void Haply::HardwareAPI::Devices::Inverse3::LogInt(int i)
{
	std::cout << std::dec << i;
}

void Haply::HardwareAPI::Devices::Inverse3::LogByte(unsigned char b)
{
	std::cout << "0x" << std::uppercase << std::setw(2) << std::setfill('0') << std::hex << (int)b;
}

void Haply::HardwareAPI::Devices::Inverse3::LogBytes(unsigned char* buffer, int offset, int length)
{
    for (int i = 0; i < length; i++)
    {
        if (i > 0)
        {
            std::cout << ", ";
        }

        this->LogByte(buffer[i + offset]);
    }
}
#endif

Haply::HardwareAPI::Devices::Inverse3::Inverse3(IO::Stream* stream)
{
    unsigned char r_buffer[1024];

    this->stream = stream;
    this->r_buffer = new unsigned char[1024];
    this->w_buffer = new unsigned char[1024];
}

void Haply::HardwareAPI::Devices::Inverse3::SendDeviceWakeup()
{
    this->w_buffer[0] = 0x0A;
    this->WriteBytes(1);
}

void Haply::HardwareAPI::Devices::Inverse3::ReceiveDeviceInfo(bool dumpInfo)
{
    uint16_t device_id;
    unsigned char device_model_num;
    unsigned char hardware_version;
    unsigned char firmware_version;
    float quaternion[4];

    this->ReceiveDeviceInfo(&device_id, &device_model_num, &hardware_version, &firmware_version, quaternion, dumpInfo);
}

void Haply::HardwareAPI::Devices::Inverse3::ReceiveDeviceInfo(uint16_t* device_id, unsigned char* device_model_number, unsigned char* hardware_version, unsigned char* firmware_version, float* quaternion, bool dumpInfo)
{
    const char hc = this->ReadHeaderCode();

    this->ReadBytes(21);

    const unsigned char * const buffer = this->r_buffer;

    *device_id = ((uint16_t*)buffer)[0];
    *device_model_number = buffer[2];
    *hardware_version = buffer[3];
    *firmware_version = buffer[4];

    const float* const buffer_f = (const float* const)(buffer + 5);

    quaternion[0] = buffer_f[0];
    quaternion[1] = buffer_f[1];
    quaternion[2] = buffer_f[2];
    quaternion[3] = buffer_f[3];

    if (dumpInfo)
    {
        std::cout << "Device ID: " << *device_id << "\n";
        std::cout << "Device model number: " << (int)*device_model_number << "\n";
        std::cout << "Hardware version: " << (int)*hardware_version << "\n";
        std::cout << "Firmware version: " << (int)*firmware_version << "\n";
        std::cout << "Quaternion x: " << buffer_f[0] << "\n";
        std::cout << "Quaternion y: " << buffer_f[1] << "\n";
        std::cout << "Quaternion z: " << buffer_f[2] << "\n";
        std::cout << "Quaternion w: " << buffer_f[3] << "\n";
    }
}

void Haply::HardwareAPI::Devices::Inverse3::SendJointTorques()
{
    float t[3];
    t[0] = 0;
    t[1] = 0;
    t[2] = 0;
    this->SendJointTorques(t);
}

void Haply::HardwareAPI::Devices::Inverse3::SendJointTorques(float* torques)
{
    unsigned char* const buffer = this->w_buffer;

    buffer[0] = 0x1A;

    float* buffer_f = (float*)(buffer + 1);

    buffer_f[0] = torques[0];
    buffer_f[1] = torques[1];
    buffer_f[2] = torques[2];

    this->WriteBytes(13);
}

void Haply::HardwareAPI::Devices::Inverse3::ReceiveJointStates()
{
    float angles[3];
    float angularVelocities[3];

    this->ReceiveJointStates(angles, angularVelocities);
}

void Haply::HardwareAPI::Devices::Inverse3::ReceiveJointStates(float* angles, float* angularVelocities)
{
    const char hc = this->ReadHeaderCode();

    this->ReadBytes(24);

    const unsigned char* const buffer = this->r_buffer;

    const float* const buffer_f = (const float* const)(buffer);

    angles[0] = buffer_f[0];
    angles[1] = buffer_f[1];
    angles[2] = buffer_f[2];
    angularVelocities[0] = buffer_f[3];
    angularVelocities[1] = buffer_f[4];
    angularVelocities[2] = buffer_f[5];

#if defined (_DEBUG)
    std::cout << "Joint 1 Angle: " << buffer_f[0] << "\n";
    std::cout << "Joint 2 Angle: " << buffer_f[1] << "\n";
    std::cout << "Joint 3 Angle: " << buffer_f[2] << "\n";
    std::cout << "Joint 1 Angular Velocity: " << buffer_f[3] << "\n";
    std::cout << "Joint 2 Angular Velocity: " << buffer_f[4] << "\n";
    std::cout << "Joint 3 Angular Velocity: " << buffer_f[5] << "\n";
#endif
}

void Haply::HardwareAPI::Devices::Inverse3::SendEndEffectorForce()
{
    float f[3];
    f[0] = 0;
    f[1] = 0;
    f[2] = 0;
    this->SendEndEffectorForce(f);
}

void Haply::HardwareAPI::Devices::Inverse3::SendEndEffectorForce(float* force)
{
    unsigned char * const buffer = this->w_buffer;

    buffer[0] = 0x2A;

    float* buffer_f = (float*)(buffer + 1);

    buffer_f[0] = force[0];
    buffer_f[1] = force[1];
    buffer_f[2] = force[2];

    this->WriteBytes(13);
}

void Haply::HardwareAPI::Devices::Inverse3::ReceiveEndEffectorState()
{
    float position[3];
    float velocity[3];

    this->ReceiveEndEffectorState(position, velocity);
}

void Haply::HardwareAPI::Devices::Inverse3::ReceiveEndEffectorState(float* position, float* velocity)
{
    const char hc = this->ReadHeaderCode();

    this->ReadBytes(24);

    const unsigned char* const buffer = this->r_buffer;

    const float* const buffer_f = (const float* const)(buffer);

    position[0] = buffer_f[0];
    position[1] = buffer_f[1];
    position[2] = buffer_f[2];
    velocity[0] = buffer_f[3];
    velocity[1] = buffer_f[4];
    velocity[2] = buffer_f[5];

#if defined (_DEBUG)
    std::cout << "Position (x, y, z): " << buffer_f[0] << " | " << buffer_f[1] << " | " << buffer_f[2] << std::endl;
    std::cout << "Velocity (x, y, z): " << buffer_f[3] << " | " << buffer_f[4] << " | " << buffer_f[5] << std::endl;
#endif
}
