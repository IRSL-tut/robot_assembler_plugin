/**
   @author YoheiKakiuchi
*/

#include <cnoid/PyUtil>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyRobotAssemblerSettings(py::module& m);
void exportPyRobotAssembler(py::module& m);
void exportPyRobotAssemblerHelper(py::module& m);

}

PYBIND11_MODULE(RobotAssembler, m)
{
    m.doc() = "python-binding for RobotAssembler";

    py::module::import("cnoid.Body");
    py::module::import("cnoid.Util");
    //py::module::import("cnoid.IRSLCoords");

    exportPyRobotAssemblerSettings(m);
    exportPyRobotAssembler(m);
    exportPyRobotAssemblerHelper(m);
}
