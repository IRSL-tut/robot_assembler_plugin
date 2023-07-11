/**
   @author YoheiKakiuchi
*/
#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>

#include "../src/irsl_choreonoid/Coordinates.h"
#include "../src/RobotAssemblerHelper.h"

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
}

} // namespace cnoid
