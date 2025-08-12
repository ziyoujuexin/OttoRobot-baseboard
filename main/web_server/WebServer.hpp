#pragma once

#include "MotionController.hpp"
#include <memory>

class WebServer {
public:
    WebServer(MotionController& motion_controller);
    void start();

private:
    MotionController& m_motion_controller;
    // Add other private members and methods for web server implementation
};
