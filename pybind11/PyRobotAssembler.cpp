/**
   @author YoheiKakiuchi
*/
#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>

#include "../src/irsl_choreonoid/Coordinates.h"
#include "../src/RobotAssembler.h"
#include "../src/RobotAssemblerInfo.h"

using namespace cnoid;
namespace ra = cnoid::robot_assembler;
namespace py = pybind11;

namespace cnoid {

void exportPyRobotAssembler(py::module &m)
{
    //
    // RoboasmCoords
    //
    py::class_< ra::RoboasmCoords, ra::RoboasmCoordsPtr, coordinates > roboasmcoords_cls(m, "RoboasmCoords", py::dynamic_attr());
    roboasmcoords_cls.def(py::init<const std::string &>())
    .def_property_readonly("name", &ra::RoboasmCoords::name)
    .def_property_readonly("point_name", &ra::RoboasmCoords::point_name)
    .def_property_readonly("worldcoords", (coordinates & (ra::RoboasmCoords::*)()) &ra::RoboasmCoords::worldcoords)
    .def("assoc", &ra::RoboasmCoords::assoc)
    .def("dissoc", &ra::RoboasmCoords::dissoc)
    .def("dissocFromParent", &ra::RoboasmCoords::dissocFromParent)
    .def("update", &ra::RoboasmCoords::update)
    .def("updateDescendants", &ra::RoboasmCoords::updateDescendants)
    .def("newcoords", (void (ra::RoboasmCoords::*)()) &ra::RoboasmCoords::newcoords)
    .def("newcoords", (void (ra::RoboasmCoords::*)(const coordinates &)) &ra::RoboasmCoords::newcoords)
    .def_property_readonly("parent", &ra::RoboasmCoords::parent)
    .def_property_readonly("hasParent", &ra::RoboasmCoords::hasParent)
    .def_property_readonly("hasDescendants", &ra::RoboasmCoords::hasDescendants)
    .def_property_readonly("isConnectingPoint", &ra::RoboasmCoords::isConnectingPoint)
    .def_property_readonly("isActuator", &ra::RoboasmCoords::isActuator)
    .def_property_readonly("isParts", &ra::RoboasmCoords::isParts)
    .def_property_readonly("isRobot", &ra::RoboasmCoords::isRobot)
    .def_property_readonly("allDescendants", [](ra::RoboasmCoords &self) {
        ra::coordsPtrList lst;
        self.allDescendants(lst);
        return lst;
    })
    .def_property_readonly("connectingPoints", [](ra::RoboasmCoords &self) {
        ra::connectingPointPtrList lst;
        self.connectingPoints(lst);
        return lst;
    })
    .def_property_readonly("activeConnectingPoints", [](ra::RoboasmCoords &self) {
        ra::connectingPointPtrList lst;
        self.activeConnectingPoints(lst);
        return lst;
    })
    .def_property_readonly("inactiveConnectingPoints", [](ra::RoboasmCoords &self) {
        ra::connectingPointPtrList lst;
        self.inactiveConnectingPoints(lst);
        return lst;
    })
    .def_property_readonly("actuators", [](ra::RoboasmCoords &self) {
        ra::connectingPointPtrList lst;
        self.actuators(lst);
        return lst;
    })
    .def_property_readonly("activeActuators", [](ra::RoboasmCoords &self) {
        ra::connectingPointPtrList lst;
        self.activeActuators(lst);
        return lst;
    })
    .def_property_readonly("inactiveActuators", [](ra::RoboasmCoords &self) {
        ra::connectingPointPtrList lst;
        self.inactiveActuators(lst);
        return lst;
    })
    .def_property_readonly("allParts", [](ra::RoboasmCoords &self) {
        ra::partsPtrList lst;
        self.allParts(lst);
        return lst;
    })
    ;

    //
    // RoboasmConnectingPoint
    //
    py::class_< ra::RoboasmConnectingPoint, ra::RoboasmConnectingPointPtr, ra::RoboasmCoords> roboasmconnectingpoint_cls(m, "RoboasmConnectingPoint", py::dynamic_attr());
    roboasmconnectingpoint_cls
    .def_property_readonly("info", [](ra::RoboasmConnectingPoint &self) {
        return self.info;
    })
    //.def_property_readonly("point_name", &ra::RoboasmCoords::point_name)
    .def_property_readonly("isActuator", &ra::RoboasmConnectingPoint::isActuator)
    .def_property_readonly("hasConfiguration", &ra::RoboasmConnectingPoint::hasConfiguration)
    .def_property_readonly("isActive", &ra::RoboasmConnectingPoint::isActive)
    .def_property_readonly("isConnected", &ra::RoboasmConnectingPoint::isConnected)
    .def_property_readonly("isInverted", &ra::RoboasmConnectingPoint::isInverted)
    .def_property_readonly("currentConfiguration", &ra::RoboasmConnectingPoint::currentConfiguration)
    .def_property_readonly("configurationID", &ra::RoboasmConnectingPoint::configurationID)
    ;
    //
    // RoboasmParts
    //
    py::class_< ra::RoboasmParts,  ra::RoboasmPartsPtr, ra::RoboasmCoords> roboasmparts_cls(m, "RoboasmParts", py::dynamic_attr());
    roboasmparts_cls
    .def_property_readonly("info", [](ra::RoboasmParts &self) {
        return self.info;
    })
    //.def_property_readonly("point_name", &ra::RoboasmCoords::point_name)
    ;
    //
    // RoboasmRobot
    //
    py::class_< ra::RoboasmRobot,  ra::RoboasmRobotPtr, ra::RoboasmCoords> roboasmrobot_cls(m, "RoboasmRobot", py::dynamic_attr());
    roboasmrobot_cls
    .def("attach", [](ra::RoboasmRobot &self, ra::RoboasmPartsPtr parts,  const std::string &parts_point,
                      const std::string &robot_point, const std::string &config, bool just_align) {
        return self.attach(parts, parts_point, robot_point, config, just_align);
    }, py::arg("parts"), py::arg("parts_point"), py::arg("robot_point"), py::arg("config"), py::arg("just_align") = false)
    .def("createRoboasm", &ra::RoboasmRobot::createRoboasm)
    .def("writeConfig", &ra::RoboasmRobot::writeConfig)
    .def("detach", &ra::RoboasmRobot::detach)
    .def("rootParts", &ra::RoboasmRobot::rootParts)
    ;

    //
    // RoboasmUtil
    //
    py::class_< ra::RoboasmUtil > roboasmutil_cls(m, "RoboasmUtil");
    roboasmutil_cls.def(py::init<const std::string &>())
    .def(py::init<ra::SettingsPtr>())
    .def_property_readonly("settings", &ra::RoboasmUtil::settings)
    .def("makeParts", (ra::RoboasmPartsPtr (ra::RoboasmUtil::*)(const std::string &_ky)) &ra::RoboasmUtil::makeParts)
    .def("makeParts", (ra::RoboasmPartsPtr (ra::RoboasmUtil::*)(const std::string &_ky, const std::string &_nm)) &ra::RoboasmUtil::makeParts)
    .def("makeRobotFromKey", [](ra::RoboasmUtil &self, const std::string &parts_key, const std::string &name) {
        return self.makeRobot(name, parts_key);
    }, py::arg("parts_key"), py::arg("name") = std::string())
    //makeRobotFromParts
    //makeRobotFromHistory
    .def("makeRobot", (ra::RoboasmRobotPtr (ra::RoboasmUtil::*)(ra::RoboasmFile &_rafile)) &ra::RoboasmUtil::makeRobot)
    .def("makeRobot", [](ra::RoboasmUtil &self, ra::cnoidRAFile &raf) {
        return self.makeRobot(raf);
    })
    .def("makeRobotFromFile", &ra::RoboasmUtil::makeRobotFromFile)
    ;

    //
    // cnoidRAFile
    //
    py::class_< ra::cnoidRAFile > rafile_cls(m, "cnoidRAFile");
    rafile_cls.def(py::init<const std::string &>())// .roboasm file-name 
    .def("isValid", &ra::cnoidRAFile::isValid)
    .def("parseRoboasm", [](ra::cnoidRAFile &self, const std::string &filename, bool parse_config) {
        return self.parseRoboasm(filename, parse_config);
    }, py::arg("filename"), py::arg("parseConfig") = true)
    .def("dumpRoboasm", &ra::cnoidRAFile::dumpRoboasm)
    .def("historyToMap", &ra::cnoidRAFile::historyToMap, py::arg("map") = nullptr)
    .def("addInfo", &ra::cnoidRAFile::historyToMap, py::arg("map") = nullptr)
    .def("updateRobotByInfo", &ra::cnoidRAFile::updateRobotByInfo)
    .def("makeRobot", &ra::cnoidRAFile::makeRobot,
         py::arg("util"), py::arg("name") = std::string(), py::arg("rename") = false)
    ;
}

} // namespace cnoid
