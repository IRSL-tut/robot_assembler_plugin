/**
   @author YoheiKakiuchi
*/
#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>

#include "../src/irsl_choreonoid/Coordinates.h"
#include "../src/RobotAssembler.h"

using namespace cnoid;
namespace ra = cnoid::robot_assembler;
namespace py = pybind11;

namespace cnoid {

void exportPyRobotAssembler(py::module &m)
{
    //
    // RoboasmCoords
    //
    py::class_< ra::RoboasmCoords, ra::RoboasmCoordsPtr, coordinates > roboasmcoords_cls(m, "RoboasmCoords");
    roboasmcoords_cls.def(py::init<const std::string &>());

    //
    // RoboasmConnectingPoint
    //
    py::class_< ra::RoboasmConnectingPoint, ra::RoboasmConnectingPointPtr, ra::RoboasmCoords> roboasmconnectingpoint_cls(m, "RoboasmConnectingPoint");

    //
    // RoboasmParts
    //
    py::class_< ra::RoboasmParts,  ra::RoboasmPartsPtr, ra::RoboasmCoords> roboasmparts_cls(m, "RoboasmParts");

    //
    // RoboasmRobot
    //
    py::class_< ra::RoboasmRobot,  ra::RoboasmRobotPtr, ra::RoboasmCoords> roboasmrobot_cls(m, "RoboasmRobot");

    //
    // RoboasmUtil
    //
    py::class_< ra::RoboasmUtil > roboasmutil_cls(m, "RoboasmUtil");
    roboasmutil_cls.def(py::init<const std::string &>())
    .def(py::init<ra::SettingsPtr>())
    .def_property_readonly("settings", &ra::RoboasmUtil::settings)
    .def("makeParts", (ra::RoboasmPartsPtr (ra::RoboasmUtil::*)(const std::string &_ky)) &ra::RoboasmUtil::makeParts)
    .def("makeParts", (ra::RoboasmPartsPtr (ra::RoboasmUtil::*)(const std::string &_ky, const std::string &_nm)) &ra::RoboasmUtil::makeParts)
    //.def("makeRobot"
    //.def("makeRobotFromFile"
    ;
}

} // namespace cnoid
