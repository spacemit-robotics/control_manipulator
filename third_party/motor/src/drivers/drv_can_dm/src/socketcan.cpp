/*******************************************************************************
                * BSD 3-Clause License
                *
                * Copyright (c) 2021, Qiayuan Liao
                * All rights reserved.
                *
                * Redistribution and use in source and binary forms, with or without
                * modification, are permitted provided that the following conditions are met:
                *
                * * Redistributions of source code must retain the above copyright notice, this
                *   list of conditions and the following disclaimer.
                *
                * * Redistributions in binary form must reproduce the above copyright notice,
                *   this list of conditions and the following disclaimer in the documentation
                *   and/or other materials provided with the distribution.
                *
                * * Neither the name of the copyright holder nor the names of its
                *   contributors may be used to endorse or promote products derived from
                *   this software without specific prior written permission.
                *
                * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
                * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
                * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
                * ARE
                * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
                * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
                * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
                * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
                * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
                * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
                * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
                *******************************************************************************/

//
// Created by qiayuan on 3/3/21.
//
#include "socketcan.h"

#include <sys/ioctl.h>
#include <sys/socket.h>

#include <iostream>
#include <string>
#include <utility>

namespace damiao {
/* ref:
 * https://github.com/JCube001/socketcan-demo
 * http://blog.mbedded.ninja/programming/operating-systems/linux/how-to-use-socketcan-with-c-in-linux
 * https://github.com/linux-can/can-utils/blob/master/candump.c
 */

SocketCAN::~SocketCAN() {
    if (this->isOpen()) {
        this->close();
    }
}

void SocketCAN::log_throttled_error(const std::string& interface_name) const {
    static auto last_log_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - last_log_time;

    if (elapsed.count() >= 5.0) {
        std::cerr << "Unable to write: Socket " << interface_name << " not open" << std::endl;
        last_log_time = now;
    }
}


// 在 CanBus::CanBus(const std::string& bus_name, CanDataPtr data_ptr, int thread_priority)
// : bus_name_(bus_name), data_ptr_(data_ptr)中调用如下

// while (!socket_can_.open(bus_name, boost::bind(&CanBus::frameCallback, this, _1), thread_priority)
// && ros::ok())
bool SocketCAN::open(
    const std::string& interface,
    boost::function<void(const can_frame& frame)> handler,
    int thread_priority) {
    // 在 static void* socketcan_receiver_thread(void* argv) 这个函数里
    // sock->reception_handler(rx_frame);
    reception_handler = std::move(handler);
    sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);  // 返回 socketcan 句柄
    if (sock_fd_ == -1) {
        std::cerr << "[ERROR] Error: Unable to create a CAN socket" << std::endl;
        return false;
    }
    char name[16] = {};
    strncpy(name, interface.c_str(), interface.size());
    strncpy(interface_request_.ifr_name, name, IFNAMSIZ);
    if (ioctl(sock_fd_, SIOCGIFINDEX, &interface_request_) == -1) {
        std::string message = "[ERROR] Unable to select CAN interface ";
        message += name;
        message += ": I/O control error";
        std::cerr << message << std::endl;
        close();
        return false;
    }
    address_.can_family = AF_CAN;
    address_.can_ifindex = interface_request_.ifr_ifindex;
    int rc = bind(sock_fd_, reinterpret_cast<struct sockaddr*>(&address_), sizeof(address_));
    if (rc == -1) {
        std::string message = "[ERROR] Failed to bind socket to ";
        message += name;
        message += " network interface";
        std::cerr << message << std::endl;
        close();
        return false;
    }
    return startReceiverThread(thread_priority);
}

void SocketCAN::close() {
    terminate_receiver_thread_ = true;
    while (receiver_thread_running_) {}

    if (!isOpen()) {
        return;
    }
    ::close(sock_fd_);
    sock_fd_ = -1;
}

bool SocketCAN::isOpen() const {
    return (sock_fd_ != -1);
}

void SocketCAN::write(can_frame* frame) const {
    if (!isOpen()) {
        log_throttled_error(interface_request_.ifr_name);
        return;
    }
    if (::write(sock_fd_, frame, sizeof(can_frame)) == -1) {
        log_throttled_error(interface_request_.ifr_name);
    }
}

static void* socketcan_receiver_thread(void* argv) {
    /*
     * The first and only argument to this function
     * is the pointer to the object, which started the thread.
     */
    auto* sock = reinterpret_cast<SocketCAN*>(argv);
    fd_set descriptors;
    int maxfd = sock->sock_fd_;
    struct timeval timeout {};
    can_frame rx_frame{};
    sock->receiver_thread_running_ = true;
    while (!sock->terminate_receiver_thread_) {
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        FD_ZERO(&descriptors);
        FD_SET(sock->sock_fd_, &descriptors);
        if (select(maxfd + 1, &descriptors, nullptr, nullptr, &timeout)) {
            ssize_t len = read(sock->sock_fd_, &rx_frame, CAN_MTU);
            if (len < 0) {
                continue;
            }
            if (sock->reception_handler != nullptr) {
                sock->reception_handler(rx_frame);
            }
        }
    }
    sock->receiver_thread_running_ = false;
    return nullptr;
}

bool SocketCAN::startReceiverThread(int thread_priority) {
    terminate_receiver_thread_ = false;
    int rc = pthread_create(&receiver_thread_id_, nullptr, &socketcan_receiver_thread, this);
    if (rc != 0) {
        std::cerr << "[ERROR] Unable to start receiver thread" << std::endl;
        return false;
    }
    std::string message = "[INFO] Successfully started receiver thread with ID ";
    message += std::to_string(static_cast<int64_t>(receiver_thread_id_));
    std::cout << message << std::endl;
    sched_param sched{.sched_priority = thread_priority};
    pthread_setschedparam(receiver_thread_id_, SCHED_FIFO, &sched);
    return true;
}

}  // namespace damiao
