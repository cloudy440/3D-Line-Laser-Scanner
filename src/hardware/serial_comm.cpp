#include "hardware/serial_comm.h"
#include <iostream>
#include <stdexcept>
#include <cstring>
#include <thread>

#include <boost/asio.hpp>

#ifdef _WIN32
#include <windows.h>
#endif

// 使用 Pimpl 隔离 Boost.Asio 具体实现细节
struct SerialComm::Impl {
    boost::asio::io_context io;
    boost::asio::serial_port port{io};
    bool connected = false;
};

// 按协议定义计算 CRC16 校验码
uint16_t SerialComm::calcCRC16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }
    return crc;
}

SerialComm::SerialComm() : m_impl(new Impl()) {}
SerialComm::~SerialComm() { close(); delete m_impl; }

// 初始化串口参数并建立连接
bool SerialComm::init(const std::string& portName, unsigned int baudRate) {
    try {
        m_impl->port.open(portName);
        m_impl->port.set_option(boost::asio::serial_port_base::baud_rate(baudRate));
        m_impl->port.set_option(boost::asio::serial_port_base::character_size(8));
        m_impl->port.set_option(boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));
        m_impl->port.set_option(boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none));
        m_impl->port.set_option(boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none));

#ifdef _WIN32
        // 设置底层串口读超时，避免 read_some 长时间阻塞
        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        timeouts.ReadTotalTimeoutConstant = 200;
        SetCommTimeouts(m_impl->port.native_handle(), &timeouts);
#endif

        m_impl->connected = true;
        std::cout << "[SerialComm] Connected to " << portName
                  << " @ " << baudRate << " bps" << std::endl;
        return true;
    } catch (const boost::system::system_error& e) {
        std::cerr << "[SerialComm] Failed to open " << portName
                  << ": " << e.what() << std::endl;
        return false;
    }
}

void SerialComm::close() {
    if (m_impl->port.is_open()) {
        m_impl->port.close();
        m_impl->connected = false;
    }
}

bool SerialComm::isConnected() const { return m_impl->connected; }

// 按通信协议打包命令帧
std::vector<uint8_t> SerialComm::packCommand(uint8_t cmdType, float data) {
    std::vector<uint8_t> frame(FRAME_SIZE);
    frame[0] = FRAME_HEAD_1;  frame[1] = FRAME_HEAD_2;
    frame[2] = cmdType;
    std::memcpy(&frame[3], &data, 4);
    uint16_t crc = calcCRC16(&frame[2], 5);
    frame[7] = static_cast<uint8_t>(crc >> 8);
    frame[8] = static_cast<uint8_t>(crc & 0xFF);
    frame[9] = FRAME_TAIL_1;  frame[10] = FRAME_TAIL_2;
    return frame;
}

// 校验并解析 STM32 返回帧
bool SerialComm::parseResponse(const std::vector<uint8_t>& buf,
                               uint8_t& cmdOut, float& dataOut) {
    if (buf.size() < FRAME_SIZE) return false;
    if (buf[0] != FRAME_HEAD_1 || buf[1] != FRAME_HEAD_2) return false;
    if (buf[9] != FRAME_TAIL_1 || buf[10] != FRAME_TAIL_2) return false;
    uint16_t recvCRC = (static_cast<uint16_t>(buf[7]) << 8) | buf[8];
    if (recvCRC != calcCRC16(&buf[2], 5)) {
        std::cerr << "[SerialComm] CRC mismatch!" << std::endl;
        return false;
    }
    cmdOut = buf[2];
    std::memcpy(&dataOut, &buf[3], 4);
    return true;
}

// 在给定超时内读取完整协议帧
bool SerialComm::readFrameWithTimeout(std::vector<uint8_t>& response,
                                      int timeoutSeconds) {
    response.resize(FRAME_SIZE);
    size_t totalRead = 0;
    auto deadline = std::chrono::steady_clock::now()
                    + std::chrono::seconds(timeoutSeconds);

    while (totalRead < FRAME_SIZE) {
        if (std::chrono::steady_clock::now() > deadline) {
            std::cerr << "[SerialComm] Timeout after " << timeoutSeconds
                      << "s (" << totalRead << "/" << FRAME_SIZE << " bytes)"
                      << std::endl;
            return false;
        }

        boost::system::error_code ec;
        size_t n = m_impl->port.read_some(
            boost::asio::buffer(response.data() + totalRead,
                                FRAME_SIZE - totalRead), ec);
        if (ec) {
            std::cerr << "[SerialComm] Read error: " << ec.message() << std::endl;
            return false;
        }
        totalRead += n;

        if (n == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    return true;
}

// 发送单步命令并等待步进完成应答
float SerialComm::moveStepAndWait() {
    boost::asio::write(m_impl->port, boost::asio::buffer(packCommand(CMD_STEP)));

    std::vector<uint8_t> resp;
    if (!readFrameWithTimeout(resp, 5))
        throw std::runtime_error("[SerialComm] moveStepAndWait timeout!");

    uint8_t cmd; float angle;
    if (!parseResponse(resp, cmd, angle))
        throw std::runtime_error("[SerialComm] Invalid response!");
    if (cmd == RESP_ERROR)
        throw std::runtime_error("[SerialComm] STM32 error!");
    if (cmd != RESP_STEP_DONE)
        throw std::runtime_error("[SerialComm] Unexpected response!");
    return angle;
}

// 发送多步移动命令并等待完成应答
float SerialComm::sendMoveCommand(int32_t steps) {
    float data = static_cast<float>(steps);
    std::cout << "[SerialComm] MOVE " << steps << " steps..." << std::endl;
    boost::asio::write(m_impl->port, boost::asio::buffer(packCommand(CMD_MOVE, data)));

    std::vector<uint8_t> resp;
    if (!readFrameWithTimeout(resp, 60))
        throw std::runtime_error("[SerialComm] sendMoveCommand timeout!");

    uint8_t cmd; float angle;
    if (!parseResponse(resp, cmd, angle))
        throw std::runtime_error("[SerialComm] Invalid response!");
    if (cmd == RESP_ERROR)
        throw std::runtime_error("[SerialComm] STM32 RESP_ERROR for MOVE!");
    if (cmd != RESP_MOVE_DONE) {
        std::cerr << "[SerialComm] Unexpected cmd=0x"
                  << std::hex << (int)cmd << std::dec << std::endl;
        throw std::runtime_error("[SerialComm] Unexpected response for MOVE!");
    }
    std::cout << "[SerialComm] Move done. Angle=" << angle << " deg" << std::endl;
    return angle;
}

// 下发激光开关控制命令
void SerialComm::setLaser(bool on) {
    boost::asio::write(m_impl->port,
        boost::asio::buffer(packCommand(on ? CMD_LASER_ON : CMD_LASER_OFF)));
}
