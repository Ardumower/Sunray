// Python wrappers for Sunray

//  install pybind11 with:  
//     sudo apt install pybind11-dev 
//     pip install "pybind11[global]"

// to run the python interpreter as sudo:
//  1. which python  =>    /home/alex/miniconda3/envs/py38/bin/python
//  2. sudo /home/alex/miniconda3/envs/py38/bin/python test.py


// pysunray.cpp
#include <pybind11/pybind11.h>
//#include <pybind11/eigen.h>
#include "BleUartServer.h"
#include "dabble/DabbleESP32.h"
#include "robot.h"

namespace py = pybind11;
//constexpr auto byref = py::return_value_policy::reference_internal;


// https://github.com/hfutcgncas/normalSpeed/issues/3
// pip install "pybind11[global]"
// conda install pybind11 -c conda-forge

// https://github.com/pybind/pybind11/issues/890
// https://www.heise.de/hintergrund/Python-meets-C-C-Teil-2-SWIG-und-pybind11-6158432.html?seite=2



PYBIND11_MODULE(pysunray, m) {
    py::class_<RobotDriver>(m, "RobotDriver")
    ;

    py::class_<MotorDriver>(m, "MotorDriver")
    ;

    // SerialRobotDriver
    py::class_<SerialRobotDriver, RobotDriver>(m, "SerialRobotDriver")    
    .def(py::init<>())  
    .def("begin", &SerialRobotDriver::begin)         
    .def("run", &SerialRobotDriver::run)         
    ;

    py::class_<SerialMotorDriver, MotorDriver>(m, "SerialMotorDriver")    
    .def("begin", &SerialMotorDriver::begin)         
    .def("run", &SerialMotorDriver::run)         
    ;

    // CanRobotDriver
    py::class_<CanRobotDriver, RobotDriver>(m, "CanRobotDriver")    
    .def(py::init<>())  
    .def("begin", &CanRobotDriver::begin)         
    .def("run", &CanRobotDriver::run)         
    ;
    
    py::class_<CanMotorDriver, MotorDriver>(m, "CanMotorDriver")    
    .def("begin", &CanMotorDriver::begin)         
    .def("run", &CanMotorDriver::run)         
    ;


    // motor
    py::class_<Motor>(m, "Motor")
    .def(py::init<>())    
    .def("begin", &Motor::begin)
    .def("run", &Motor::run)             
    .def("setLinearAngularSpeed", &Motor::setLinearAngularSpeed)         
    .def("setMowState", &Motor::setMowState)         
    ;

    
    // BLE uart server
    py::class_<Print>(m, "Print")
    .def(py::init<>())
    ;

    py::class_<Stream, Print>(m, "Stream")
    ;  

    py::class_<HardwareSerial, Stream>(m, "HardwareSerial")
    .def(py::init<>())
    ;  

    py::class_<BleUartServer, HardwareSerial>(m, "BleUartServer")
    .def(py::init<>())  
    .def("read", &HardwareSerial::read)    
    .def("available", &HardwareSerial::available)
    .def("begin", &HardwareSerial::begin)         
    //.def_readonly("v_data", &MyClass::v_data, byref)
    //.def_readonly("v_gamma", &MyClass::v_gamma, byref)
    ;

    py::class_<ModuleParent>(m, "ModuleParent")
    ;

    py::class_<DabbleClass>(m, "DabbleClass")
    .def(py::init<>())  
    .def("begin", &DabbleClass::begin)   
    .def("processInput", &DabbleClass::processInput)   
    //.def_readonly("v_data", &MyClass::v_data, byref)
    //.def_readonly("v_gamma", &MyClass::v_gamma, byref)
    ;

    py::class_<GamePadModule, ModuleParent>(m, "GamePadModule")
    .def(py::init<>())  
    .def("isUpPressed", &GamePadModule::isUpPressed)    
    .def("isDownPressed", &GamePadModule::isDownPressed)
    .def("isLeftPressed", &GamePadModule::isLeftPressed)
    .def("isRightPressed", &GamePadModule::isRightPressed)
    .def("isSquarePressed", &GamePadModule::isSquarePressed)
    .def("isCirclePressed", &GamePadModule::isCirclePressed)
    .def("isCrossPressed", &GamePadModule::isCrossPressed)
    .def("isTrianglePressed", &GamePadModule::isTrianglePressed)
    .def("isStartPressed", &GamePadModule::isStartPressed)
    .def("getAngle", &GamePadModule::getAngle)
    .def("getRadius", &GamePadModule::getRadius)
    .def("getXaxisData", &GamePadModule::getXaxisData)
    .def("getYaxisData", &GamePadModule::getYaxisData)
    ;


    // globals
    m.attr("_motor") = &motor;
    m.attr("_SerialBLE") = &SerialBLE;
    m.attr("_robotDriver") = &robotDriver;
    m.attr("_motorDriver") = &motorDriver;    
    m.attr("_Dabble") = &Dabble;
    m.attr("_GamePad") = &GamePad;

}


