/**
   @author YoheiKakiuchi
*/
#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>

#include "../src/irsl_choreonoid/Coordinates.h"
#include "../src/RobotAssemblerHelper.h"
#include "../src/RobotAssemblerBody.h"
#include "../src/RobotAssemblerInfo.h"

#include <exception>

using namespace cnoid;
namespace ra = cnoid::robot_assembler;
namespace py = pybind11;

namespace cnoid {

bool isVariable(const std::string &str)
{
    if(str.size() <= 6) {
        return false;
    }
    int n = str.size();
    if (str[0] != '$') {
        return false;
    }
    if (str[1] != '$') {
        return false;
    }
    if (str[2] != '<') {
        return false;
    }
    if (str[n-3] != '>') {
        return false;
    }
    if (str[n-2] != '$') {
        return false;
    }
    if (str[n-1] != '$') {
        return false;
    }
    return true;
}
ValueNode *deepCopy(ValueNode *nv) {
    if(nv->isValid()) {
        if(nv->isScalar()) {
            if (nv->isString()) {
                ValueNode *res = new ScalarNode("$$<str>$$");
                return res;
            }
            return nv->clone();
        }
        if(nv->isListing()) {
            ValueNode *res = new Listing();
            Listing *l = nv->toListing();
            Listing *dst = res->toListing();
            for(auto it = l->begin(); it != l->end(); it++) {
                dst->append(deepCopy(*it));
            }
            return res;
        }
        if(nv->isMapping()) {
            ValueNode *res = new Mapping();
            Mapping *m = nv->toMapping();
            Mapping *dst = res->toMapping();
            for(auto it = m->begin(); it != m->end(); it++) {
                // it->first // key
                // it->second // value
                std::string key(it->first);
                key += "$$<key>$$";
                dst->insert(key, deepCopy(it->second));
            }
            return res;
        }
        // invalid
        std::cout << "invalid 1" << std::endl;
    }
    // invalid
    std::cout << "invalid 2" << std::endl;
    return nullptr;
};

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

    // RobotAssemblerBody
    // Utility for Body
    m.def("mergeLink", [](Link *p, Link *c, Body *bd) {
        return ra::mergeLink(p, c, bd);
    }, py::arg("plink"), py::arg("clink"), py::arg("body") = nullptr);
    m.def("addRootOffset", [](Body *bd, const coordinates &cds) {
        Isometry3 T; cds.toPosition(T);
        return ra::addRootOffset(bd, T);
    });
    m.def("testYaml", [](std::string fname) {
        //ValueNode *res = new ValueNode();
        YAMLReader yaml_reader;
        if (! yaml_reader.load(fname)) {
            std::cerr << "File Loading error : " << fname << std::endl;
            throw std::exception();
        }
        for(int i = 0; i < yaml_reader.numDocuments(); i++) {
            ValueNode *val = yaml_reader.document(i);
            if (val->isValid()) {
                ////
                return deepCopy(val);
            }
        }
        throw std::exception();
    });

}

} // namespace cnoid
