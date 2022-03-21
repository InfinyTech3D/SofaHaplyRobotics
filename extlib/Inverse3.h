#pragma once
#include "Stream.h"
#include <cstdint>

namespace Haply
{
	namespace HardwareAPI
	{
		namespace Devices
		{
			class Inverse3
			{
				IO::Stream* stream;
				unsigned char* w_buffer;
				unsigned char* r_buffer;
				void WriteBytes(int n);
				char ReadHeaderCode();
				int ReadBytes(int n);
#if defined (_DEBUG)
				void LogInt(int i);
				void LogByte(unsigned char b);
				void LogBytes(unsigned char* buffer, int offset, int length);
#endif
			public:
				Inverse3(IO::Stream* stream);

				void SendDeviceWakeup();

				void ReceiveDeviceInfo(bool dumpInfo = false);
				void ReceiveDeviceInfo(uint16_t* device_id, unsigned char* device_model_number, unsigned char* hardware_version, unsigned char* firmware_version, float* quaternion, bool dumpInfo = false);

				void SendJointTorques();
				void SendJointTorques(float* torques);

				void ReceiveJointStates();
				void ReceiveJointStates(float* angles, float* angularVelocities);

				void SendEndEffectorForce();
				void SendEndEffectorForce(float* force);

				void ReceiveEndEffectorState();
				void ReceiveEndEffectorState(float* position, float* velocity);
			};
		}
	}
}