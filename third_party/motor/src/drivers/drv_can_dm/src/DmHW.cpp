#include "DmHW.h"

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace damiao {

bool DmHW::init(const std::vector<MotorConfig>& motor_configs) {
    std::unordered_map<std::string, std::vector<MotorConfig>> bus_motors;
    for (const auto& config : motor_configs) {
        bus_motors[config.bus_name].push_back(config);
    }

    for (const auto& bus_motor_pair : bus_motors) {
        const std::string& bus_name = bus_motor_pair.first;
        const std::vector<MotorConfig>& motors = bus_motor_pair.second;

        for (const auto& motor : motors) {
            DmActData data{};
            data.motorType = motor.motor_type;
            data.mode = motor.control_mode;
            data.can_id = motor.can_id;
            data.mst_id = motor.master_id;
            data.pos = 0;
            data.vel = 0;
            data.effort = 0;
            data.cmd_pos = 0;
            data.cmd_vel = 0;
            data.cmd_effort = 0;
            bus_id2dm_data_[bus_name].insert(std::make_pair(motor.can_id, data));
        }

        auto motor_control =
            std::make_shared<Motor_Control>(bus_name, &bus_id2dm_data_[bus_name]);
        motor_ports_.push_back(motor_control);
    }

    return true;
}

void DmHW::read(const sysclock::time_point& time, const duration& period) {
    for (auto motor_port : motor_ports_) {
        motor_port->read();
    }
}

// 定义DmHW类的write成员函数，该函数用于在给定的时间和周期内更新机器人的硬件状态
void DmHW::write(const sysclock::time_point& time, const duration& period) {
    for (auto motor_port : motor_ports_) {
        motor_port->write();
    }
}

void DmHW::setCanBusThreadPriority(int thread_priority) {
    thread_priority_ = thread_priority;
}

void DmHW::setMotionParams(const MotionParams& params) {
    for (auto motor_port : motor_ports_) {
        motor_port->setMotionParams(params);
    }
}

void DmHW::sendMitCommand(const std::string& bus_name, uint16_t can_id, float p, float v, float t, float kp, float kd) {
    (void)bus_name;
    for (auto& motor_port : motor_ports_) {
        const auto& motors = motor_port->get_motors();
        if (motors.count(can_id)) {
            auto motor = motors.at(can_id);
            motor_port->control_mit(*motor, kp, kd, p, v, t);
            return;
        }
    }
    std::string error_message = "[DmHW] Error: Motor " + std::to_string(can_id) + " not found on any bus.";
    std::cerr << error_message << std::endl;
}

void DmHW::setZeroPosition() {
    for (auto motor_port : motor_ports_) {
        const auto& motors = motor_port->get_motors();
        for (const auto& motor_pair : motors) {
            auto motor = motor_pair.second;
            motor_port->set_zero_position(*motor);
            std::string message =
                "[DmHW] Set zero position for motor ID: " + std::to_string(motor->GetCanId());
            std::cout << message << std::endl;
        }
    }
    std::cout << "[DmHW] All motors zero position set successfully." << std::endl;
}

void DmHW::emergencyStop() {
    std::cout << "[DmHW] Emergency stop initiated..." << std::endl;

    for (auto motor_port : motor_ports_) {
        const auto& motors = motor_port->get_motors();
        for (const auto& motor_pair : motors) {
            auto motor = motor_pair.second;
            float current_pos = motor->Get_Position();
            motor_port->control_mit(*motor, 20.0, 4.0, current_pos, 0.0, 0.0);
            std::string message = "[DmHW] Emergency stop for motor ID: ";
            message += std::to_string(motor->GetCanId());
            message += " at position: ";
            message += std::to_string(current_pos);
            message += " rad";
            std::cout << message << std::endl;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "[DmHW] Emergency stop completed." << std::endl;
}

void DmHW::disable() {
    auto now_start = std::chrono::system_clock::now();
    std::time_t now_c_start = std::chrono::system_clock::to_time_t(now_start);
    std::cout << "[DmHW] Disabling all motors at " << std::ctime(&now_c_start);
    for (auto motor_port : motor_ports_) {
        motor_port->disable_all();
    }
    auto now_end = std::chrono::system_clock::now();
    std::time_t now_c_end = std::chrono::system_clock::to_time_t(now_end);
    std::cout << "[DmHW] All motors disabled at " << std::ctime(&now_c_end);
    auto elapsed = now_end - now_start;
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    std::cout << "[DmHW] disable() elapsed time: " << elapsed_ms << " ms" << std::endl;
}

const std::unordered_map<std::string, std::unordered_map<uint16_t, damiao::DmActData>>&
DmHW::getActuatorData() const {
    return bus_id2dm_data_;
}

// 周期性刷新线程
void DmHW::startAutoRead(unsigned int period_ms) {
    if (read_running_) {
        return;
    }
    read_running_ = true;
    read_thread_ = std::thread([this, period_ms]() {
        while (read_running_) {
            auto now = std::chrono::system_clock::now();
            double seconds = period_ms / 1000.0;
            auto period = std::chrono::duration<double>(seconds);
            this->read(now, period);
            std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
        }
    });
}

void DmHW::stopAutoRead() {
    read_running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
}

}  // namespace damiao
