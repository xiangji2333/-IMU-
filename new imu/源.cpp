#include <iostream>
#include <vector>
#include <windows.h>
#include<chrono>
#include<fstream>
#include<iomanip>


// 定义常量
const double kAccFactor = 16.0;
const double kGyroFactor = 2000.0;
const double kAngleFactor = 180.0;
const double pi = acos(-1.0);

std::vector<double>acc(3, 0.0);
std::vector<double>angle(3, 0.0);
std::vector<double>gyro(3, 0.0);

std::fstream in("out.txt", std::ios::out);

// 加速度数据解读
std::vector<double> getAcc(std::vector<uint8_t> dataHex) {
	std::vector<double> acc(3);
	for (int i = 0; i < 3; ++i) {
		int lowByte = dataHex[i * 2];
		int highByte = dataHex[i * 2 + 1];
		int sixteenBitData = (highByte << 8) | lowByte;
		acc[i] = sixteenBitData / 32768.0 * kAccFactor;
		if (acc[i] >= kAccFactor)
			acc[i] -= 2 * kAccFactor;
	}
	return acc;
}

// 角速度数据解读
std::vector<double> getGyro(std::vector<uint8_t> dataHex) {
	std::vector<double> gyro(3);
	for (int i = 0; i < 3; ++i) {
		int lowByte = dataHex[i * 2];
		int highByte = dataHex[i * 2 + 1];
		int sixteenBitData = (highByte << 8) | lowByte;
		gyro[i] = sixteenBitData / 32768.0 * kGyroFactor;
		if (gyro[i] >= kGyroFactor)
			gyro[i] -= 2 * kGyroFactor;
	}
	return gyro;
}

// 角度度数据解读
std::vector<double> getAngle(std::vector<uint8_t> dataHex) {
	std::vector<double> angle(3);
	for (int i = 0; i < 3; ++i) {
		int lowByte = dataHex[i * 2];
		int highByte = dataHex[i * 2 + 1];
		int sixteenBitData = (highByte << 8) | lowByte;
		angle[i] = sixteenBitData / 32768.0 * kAngleFactor;
		if (angle[i] >= kAngleFactor)
			angle[i] -= 2 * kAngleFactor;
	}
	return angle;
}
int CheckSum = 0,Bytenum = 0,FrameState = 0;
// imu信息解读
void dueData(const std::vector<uint8_t>& inputData) {
	std::vector<uint8_t> accData(8);
	std::vector<uint8_t> gyroData(8);
	std::vector<uint8_t> angleData(8);
	for (int data : inputData) {
		if (FrameState == 0) {
			if (data == 0x55 && Bytenum == 0) {
				CheckSum = data;
				Bytenum = 1;
				continue;
			}
			else if (data == 0x51 && Bytenum == 1) {
				CheckSum += data;
				FrameState = 1;
				Bytenum = 2;
			}
			else if (data == 0x52 && Bytenum == 1) {
				CheckSum += data;
				FrameState = 2;
				Bytenum = 2;
			}
			else if (data == 0x53 && Bytenum == 1) {
				CheckSum += data;
				FrameState = 3;
				Bytenum = 2;
			}
		}
		else if (FrameState == 1) { // acc
			if (Bytenum < 10) {
				accData[Bytenum - 2] = data;
				CheckSum += data;
				Bytenum += 1;
			}
			else {
				if (data == (CheckSum & 0xff)) {
					acc = getAcc(accData);
				}
				CheckSum = 0;
				Bytenum = 0;
				FrameState = 0;
			}
		}
		else if (FrameState == 2) { // gyro
			if (Bytenum < 10) {
				gyroData[Bytenum - 2] = data;
				CheckSum += data;
				Bytenum += 1;
			}
			else {
				if (data == (CheckSum & 0xff)) {
					gyro = getGyro(gyroData);
				}
				CheckSum = 0;
				Bytenum = 0;
				FrameState = 0;
			}
		}
		else if (FrameState == 3) { // angle
			if (Bytenum < 10) {
				angleData[Bytenum - 2] = data;
				CheckSum += data;
				Bytenum += 1;
			}
			else {
				if (data == (CheckSum & 0xff)) {
					angle = getAngle(angleData);
					std::vector<double> result(9);
					for (int i = 0; i < 3; ++i) {
						result[i] = acc[i];
						result[i + 3] = gyro[i];
						result[i + 6] = angle[i];
					}
					long long now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
					in << now << ' ' <<std::fixed<<std::setprecision(10)<< result[0]*9.8 << " " << result[1]*9.8 << " " << result[2]*9.8 <<' '
						<< result[6] << " " << result[7] << " " << result[8] << '\n';
					//测试输出
					//std::cout << "acc:" << std::fixed << std::setprecision(10) << result[0] << " " << result[1] << " " << result[2] << std::endl;
					//std::cout << "gyro:" << result[3] << " " << result[4] << " " << result[5] << std::endl;
					//std::cout << "angle:" << result[6] << " " << result[7] << " " << result[8] << std::endl;
				}
				CheckSum = 0;
				Bytenum = 0;
				FrameState = 0;
			}
		}
	}
}

int main() {
	// Open serial port
	HANDLE hSerial = CreateFile(
		TEXT("COM6"),                    // COM口设置
		GENERIC_READ | GENERIC_WRITE,
		0,
		nullptr,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		nullptr
	);

	if (hSerial == INVALID_HANDLE_VALUE) {
		std::cerr << "Error: Unable to open serial port." << std::endl;
		return 1;
	}

	DCB dcbSerialParams = { 0 };
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (!GetCommState(hSerial, &dcbSerialParams)) {
		std::cerr << "Error: Unable to get serial port parameters." << std::endl;
		CloseHandle(hSerial);
		return 1;
	}

	// 串口设置（可选）
	dcbSerialParams.BaudRate = CBR_9600;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;
	if (!SetCommState(hSerial, &dcbSerialParams)) {
		std::cerr << "Error: Unable to set serial port parameters." << std::endl;
		CloseHandle(hSerial);
		return 1;
	}

	std::cout << "Serial port opened successfully." << std::endl;

	//通信修改设置部分，但好像不太好使
	
	//const BYTE _200HZ[] = { 0xFF, 0xAA, 0x03, 0x0C , 0x00 };
	//const BYTE KEY_RATE[] = { 0xFF, 0xAA, 0x69, 0x06 , 0x00 }; // 进行写操作时应该先解锁相应读写位，速率0600，保存0000
	//const BYTE KEY_SAVE[] = { 0xFF, 0xAA, 0x69, 0x00 , 0x00 };
	//const BYTE RESTART[] = { 0xFF, 0xAA, 0x00, 0xFF , 0x00 };
	//const BYTE SAVE[] = { 0xFF, 0xAA, 0x00, 0x00 , 0x00 };

	//DWORD bytes_written = 0;
	//if (!WriteFile(hSerial, KEY_RATE, sizeof KEY_RATE, &bytes_written, NULL)) {
	//	std::cerr << "Failed to write KEY_RATE to serial port!" << std::endl;
	//	CloseHandle(hSerial);
	//	return 1;
	//}
	//bytes_written = 0;
	//if (!WriteFile(hSerial, _200HZ, sizeof _200HZ, &bytes_written, NULL)) {
	//	std::cerr << "Failed to write 200hz to serial port!" << std::endl;
	//	CloseHandle(hSerial);
	//	return 1;
	//}
	//bytes_written = 0;
	//if (!WriteFile(hSerial, KEY_SAVE, sizeof KEY_SAVE, &bytes_written, NULL)) {
	//	std::cerr << "Failed to write KEY_SAVE1 to serial port!" << std::endl;
	//	CloseHandle(hSerial);
	//	return 1;
	//}
	//bytes_written = 0;
	//if (!WriteFile(hSerial, SAVE, sizeof SAVE, &bytes_written, NULL)) {
	//	std::cerr << "Failed to write SAVE to serial port!" << std::endl;
	//	CloseHandle(hSerial);
	//	return 1;
	//}
	//bytes_written = 0;
	//if (!WriteFile(hSerial, KEY_SAVE, sizeof KEY_SAVE, &bytes_written, NULL)) {
	//	std::cerr << "Failed to write KEY_SAVE2 to serial port!" << std::endl;
	//	CloseHandle(hSerial);
	//	return 1;
	//}
	//bytes_written = 0;
	//if (!WriteFile(hSerial, RESTART, sizeof RESTART, &bytes_written, NULL)) {
	//	std::cerr << "Failed to write RESTART to serial port!" << std::endl;
	//	CloseHandle(hSerial);
	//	return 1;
	//}

	// Main loop
	std::vector<uint8_t> dataHex(33); // 接受imu信息
	while (true) {
		DWORD bytesRead;
		if (!ReadFile(hSerial, &dataHex[0], 33, &bytesRead, nullptr)) {
			std::cerr << "Error reading from serial port." << std::endl;
			break;
		}
		if (bytesRead > 0) {
			dueData(dataHex);
		}
		Sleep(5);
	}

	// 关闭串口
	CloseHandle(hSerial);
	in.close();
	return 0;
}
