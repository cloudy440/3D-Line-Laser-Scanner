/**
 * @file serial_comm.h
 * @brief 上位机与 STM32 串口通信模块接口
 */

#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <string>
#include <vector>
#include <cstdint>

class SerialComm {
public:
    static constexpr uint8_t CMD_STEP      = 0x01;
    static constexpr uint8_t CMD_MOVE      = 0x05;
    static constexpr uint8_t CMD_LASER_ON  = 0x03;
    static constexpr uint8_t CMD_LASER_OFF = 0x04;

    static constexpr uint8_t RESP_STEP_DONE = 0x81;
    static constexpr uint8_t RESP_MOVE_DONE = 0x82;
    static constexpr uint8_t RESP_ERROR     = 0xFF;

    static constexpr uint8_t FRAME_HEAD_1 = 0xAA;
    static constexpr uint8_t FRAME_HEAD_2 = 0xBB;
    static constexpr uint8_t FRAME_TAIL_1 = 0xCC;
    static constexpr uint8_t FRAME_TAIL_2 = 0xDD;
    static constexpr size_t FRAME_SIZE = 11;

    SerialComm();
    ~SerialComm();

    bool init(const std::string& portName, unsigned int baudRate = 115200);
    void close();

    float moveStepAndWait();
    float sendMoveCommand(int32_t steps);
    void setLaser(bool on);
    bool isConnected() const;

private:
    struct Impl;
    Impl* m_impl;

    std::vector<uint8_t> packCommand(uint8_t cmdType, float data = 0.0f);
    bool parseResponse(const std::vector<uint8_t>& buffer, uint8_t& cmdOut, float& dataOut);
    bool readFrameWithTimeout(std::vector<uint8_t>& response, int timeoutSeconds);

    static uint16_t calcCRC16(const uint8_t* data, size_t len);
};

#endif // SERIAL_COMM_H
