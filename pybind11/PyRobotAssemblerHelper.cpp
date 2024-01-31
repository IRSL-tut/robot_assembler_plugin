/**
   @author YoheiKakiuchi
*/
#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>

#include "../src/irsl_choreonoid/Coordinates.h"
#include "../src/RobotAssemblerHelper.h"
#include "../src/RobotAssemblerBody.h"
#include "../src/RobotAssemblerInfo.h"

using namespace cnoid;
namespace ra = cnoid::robot_assembler;
namespace py = pybind11;

namespace cnoid {

void exportPyRobotAssemblerHelper(py::module &m)
{
    //
    // RASceneConnectingPoint
    //

    //
    // RASceneParts
    //
    py::class_< ra::RASceneParts, ra::RAScenePartsPtr, SgPosTransform > ra_parts_cls(m, "RASceneParts");
    ra_parts_cls.def(py::init<ra::RoboasmPartsPtr, const std::string &>())
    .def_property_readonly("parts", &ra::RASceneParts::parts)
    .def("drawBoundingBox", &ra::RASceneParts::drawBoundingBox)
    .def("updateColor", &ra::RASceneParts::updateColor)
    .def("updateCoords", &ra::RASceneParts::updateCoords)
    ;

    //
    // RASceneRobot
    //
    py::class_< ra::RASceneRobot, ra::RASceneRobotPtr, SgPosTransform > ra_robot_cls(m, "RASceneRobot");
    ra_robot_cls.def(py::init<ra::RoboasmRobotPtr>())
    .def(py::init<ra::RoboasmRobotPtr, const std::string &>())
    ;

    //
    // RoboasmBodyCreator
    //
    py::class_< ra::RoboasmBodyCreator > body_creator_cls(m, "RoboasmBodyCreator");
    body_creator_cls.def(py::init<>())
    .def(py::init<const std::string &>())//project_dir
    // setName
    // setMergeFixedJoint
    .def("createBodyRaw", [](ra::RoboasmBodyCreator &self, ra::RoboasmRobotPtr robot, MappingPtr info, const std::string &name, bool resetAngle) {
        return self.createBody(robot, info, name, resetAngle);
    }, py::arg("robot"), py::arg("info") = nullptr, py::arg("name") = std::string(), py::arg("resetAngle") = true)
    .def("createBody", [](ra::RoboasmBodyCreator &self, ra::RoboasmRobotPtr robot, ra::cnoidRAFile &rafile, const std::string &name, bool resetAngle) {
        Body *bd = self.createBody(robot, rafile.info, name, resetAngle);
        MappingPtr roboasm = new Mapping(); rafile.historyToMap(roboasm); rafile.addInfo(roboasm);
        bd->info()->insert("roboasm", roboasm); return bd;
    }, py::arg("robot"), py::arg("rafile"), py::arg("name") = std::string(), py::arg("resetAngle") = true)
    ;
}

} // namespace cnoid
