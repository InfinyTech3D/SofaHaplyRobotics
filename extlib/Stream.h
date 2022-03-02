#pragma once
namespace Haply
{
	namespace HardwareAPI
	{
		namespace IO
		{
			class Stream {
			public:
				virtual ~Stream() {}

				virtual int ReadBytes(unsigned char* buffer, int n) = 0;
				virtual char ReadByte() = 0;
				virtual void WriteBytes(unsigned char* buffer, int n) = 0;
			};
		}
	}
};