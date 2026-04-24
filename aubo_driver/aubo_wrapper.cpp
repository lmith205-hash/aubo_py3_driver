#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h> 
#include <vector>
#include <iostream>
#include <functional>
#include "serviceinterface.h" // 引用 SDK 头文件

namespace py = pybind11;

// 静态回调包装函数 (用于将 C 风格回调转发给 C++ 类成员)
void c_speed_callback_wrapper(double speed, void* arg);

class AuboPy2 {
public:
    AuboPy2() {
        m_speed_callback = nullptr;
    }

    // 1. 登录
    bool login(std::string ip, int port) {
        int ret = service.robotServiceLogin(ip.c_str(), port, "aubo", "123456");
        return (ret == 0);
    }

    // 2. 退出
    void logout() {
        service.robotServiceSetRealTimeEndSpeedPush(false);
        service.robotServiceLogout();
    }

    // 3. 获取关节角
    std::vector<double> get_current_joints() {
        aubo_robot_namespace::JointParam jointParam;
        int ret = service.robotServiceGetJointAngleInfo(jointParam);
        
        std::vector<double> joints;
        if (ret == 0) {
            for(int i=0; i<6; i++) joints.push_back(jointParam.jointPos[i]);
        }
        return joints;
    }

    // 4. 开启透传 (Servo)
    bool enable_servo() {
        int ret = service.robotServiceEnterTcp2CanbusMode();
        return (ret == 0);
    }

    // 5. 关闭透传
    bool disable_servo() {
        int ret = service.robotServiceLeaveTcp2CanbusMode();
        return (ret == 0);
    }

    // 6. 发送透传关节角
    bool send_joints(std::vector<double> joints) {
        if (joints.size() != 6) return false;
        
        double target[6];
        for(int i=0; i<6; i++) target[i] = joints[i];

        int ret = service.robotServiceSetRobotPosData2Canbus(target);
        return (ret == 0);
    }

    // ==========================================
    //  新增接口：提前到位 (Arrival Ahead) 设置
    //  修正说明：根据报错，函数名需增加 "Mode" 后缀
    // ==========================================

    // 7. 设置距离模式提前到位
    bool set_arrival_ahead_distance(double distance) {
        // [Fix] robotServiceSetArrivalAheadDistance -> robotServiceSetArrivalAheadDistanceMode
        int ret = service.robotServiceSetArrivalAheadDistanceMode(distance);
        if (ret == 0) return true;
        return false;
    }

    // 8. 设置时间模式提前到位
    bool set_arrival_ahead_time(double sec) {
        // [Fix] robotServiceSetArrivalAheadTime -> robotServiceSetArrivalAheadTimeMode
        int ret = service.robotServiceSetArrivalAheadTimeMode(sec);
        if (ret == 0) return true;
        return false;
    }

    // 9. 取消提前到位 (设置为精准到位)
    bool set_no_arrival_ahead() {
        int ret = service.robotServiceSetNoArrivalAhead();
        return (ret == 0);
    }

    // ==========================================

    // 注册回调函数
    bool register_speed_callback(std::function<void(double)> callback) {
        m_speed_callback = callback;
        int ret1 = service.robotServiceSetRealTimeEndSpeedPush(true);
        int ret2 = service.robotServiceRegisterRealTimeEndSpeedCallback(c_speed_callback_wrapper, this);

        if (ret1 == 0 && ret2 == 0) {
            return true;
        } else {
            std::cerr << "[C++] Failed to register callback. Ret1: " << ret1 << " Ret2: " << ret2 << std::endl;
            return false;
        }
    }

    void trigger_speed_callback(double speed) {
        if (m_speed_callback) {
            py::gil_scoped_acquire acquire;
            m_speed_callback(speed);
        }
    }

private:
    ServiceInterface service;
    std::function<void(double)> m_speed_callback;
};

void c_speed_callback_wrapper(double speed, void* arg) {
    AuboPy2* wrapper = static_cast<AuboPy2*>(arg);
    if (wrapper) {
        wrapper->trigger_speed_callback(speed);
    }
}

PYBIND11_MODULE(aubo_py3, m) {
    py::class_<AuboPy2>(m, "AuboRobot")
        .def(py::init<>())
        .def("login", &AuboPy2::login)
        .def("logout", &AuboPy2::logout)
        .def("get_current_joints", &AuboPy2::get_current_joints)
        .def("enable_servo", &AuboPy2::enable_servo)
        .def("disable_servo", &AuboPy2::disable_servo)
        .def("send_joints", &AuboPy2::send_joints)
        .def("set_arrival_ahead_distance", &AuboPy2::set_arrival_ahead_distance)
        .def("set_arrival_ahead_time", &AuboPy2::set_arrival_ahead_time)
        .def("set_no_arrival_ahead", &AuboPy2::set_no_arrival_ahead)
        .def("register_speed_callback", &AuboPy2::register_speed_callback);
}