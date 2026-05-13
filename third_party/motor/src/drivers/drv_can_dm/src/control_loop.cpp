#include "control_loop.h"

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

namespace damiao {

DmHWLoop::DmHWLoop(std::shared_ptr<DmHW> hardware_interface)
    : hardware_interface_(std::move(hardware_interface)), loopRunning_(true) {
    loop_hz_ = 500;
    cycle_time_error_threshold_ = 0.002;
    int thread_priority = 95;

    std::vector<damiao::MotorConfig> motor_configs = {{
        .bus_name = "can0",
        .can_id = 0x01,
        .master_id = 0x11,
        .motor_type = damiao::DM4310,
        .control_mode = damiao::MIT_MODE,
    }};

    hardware_interface_->setCanBusThreadPriority(thread_priority);
    hardware_interface_->init(motor_configs);
    last_time_ = clock::now();

    loop_thread_ = std::thread([&]() {
        while (loopRunning_) {
            update();
        }
    });
    sched_param sched{.sched_priority = thread_priority};
    if (pthread_setschedparam(loop_thread_.native_handle(), SCHED_FIFO, &sched) != 0) {
        std::cerr
            << "Failed to set thread priority (possible reason: user/group permissions are not set properly)."
            << std::endl;
    }
}

void DmHWLoop::update() {
    const auto current_time = clock::now();
    const duration desired_duration(1.0 / loop_hz_);
    duration time_span = current_time - last_time_;
    elapsed_time_ = time_span;
    last_time_ = current_time;

    const double cycle_time_error = (elapsed_time_ - desired_duration).count();
    if (cycle_time_error > cycle_time_error_threshold_) {
        std::cerr
            << "WARN: Cycle time exceeded error threshold by: "
            << cycle_time_error - cycle_time_error_threshold_
            << "s, cycle time: " << elapsed_time_.count()
            << "s, threshold: " << cycle_time_error_threshold_ << "s\n";
    }

    hardware_interface_->read(std::chrono::system_clock::now(), elapsed_time_);
    hardware_interface_->write(std::chrono::system_clock::now(), elapsed_time_);

    const auto sleep_till =
        current_time + std::chrono::duration_cast<clock::duration>(desired_duration);
    std::this_thread::sleep_until(sleep_till);
}

DmHWLoop::~DmHWLoop() {
    loopRunning_ = false;
    if (loop_thread_.joinable()) {
        loop_thread_.join();
    }
}

DmHWLoop::DmHWLoop(
        std::shared_ptr<DmHW> hardware_interface,
        const std::vector<MotorConfig>& motor_configs)
        : hardware_interface_(std::move(hardware_interface)),
            loopRunning_(true) {
    loop_hz_ = 500;
    cycle_time_error_threshold_ = 0.002;
    int thread_priority = 95;

    hardware_interface_->setCanBusThreadPriority(thread_priority);
    hardware_interface_->init(motor_configs);
    last_time_ = clock::now();

    loop_thread_ = std::thread([&]() {
        while (loopRunning_) {
            update();
        }
    });
    sched_param sched{.sched_priority = thread_priority};
    if (pthread_setschedparam(loop_thread_.native_handle(), SCHED_FIFO, &sched) != 0) {
        std::cerr
            << "Failed to set thread priority (possible reason: user/group permissions are not set properly)."
            << std::endl;
    }
}

}  // namespace damiao
