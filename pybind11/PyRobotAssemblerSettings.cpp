/**
   @author YoheiKakiuchi
*/

#include <sstream>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "../src/irsl_choreonoid/Coordinates.h"
#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/SceneGraph>

#include "../src/RobotAssemblerSettings.h"
#include "../src/RobotAssemblerBody.h"

using namespace cnoid;
namespace ra = cnoid::robot_assembler;
namespace py = pybind11;

namespace cnoid {

void exportPyRobotAssemblerSettings(py::module &m)
{
    m.doc() = "python-binding for RobotAssembler";

    //
    // Geometry
    //
    py::class_< ra::Geometry > geometry_cls(m, "RobotAssemblerGeometry");
    py::enum_<ra::Geometry::Type>(geometry_cls, "Type")
    //'.value("{}", ra::Geometry::Type::{})'
    .value("None", ra::Geometry::Type::None)
    .value("Mesh", ra::Geometry::Type::Mesh)
    .value("Box", ra::Geometry::Type::Box)
    .value("Cylinder", ra::Geometry::Type::Cylinder)
    .value("Sphere", ra::Geometry::Type::Sphere)
    .value("Cone", ra::Geometry::Type::Cone)
    .value("Capsule", ra::Geometry::Type::Capsule)
    .value("Ellipsoid", ra::Geometry::Type::Ellipsoid)
    .value("Dummy", ra::Geometry::Type::Dummy)
    .export_values();
    geometry_cls.def(py::init<>())
    .def_property_readonly("url", [](ra::Geometry &self) { return self.url; })
    .def_property_readonly("coords", [](ra::Geometry &self) { return self.coords; })
    .def_property_readonly("scale", [](ra::Geometry &self) { return self.scale; })
    .def_property_readonly("color", [](ra::Geometry &self) { return self.color; })
    .def_property_readonly("type", [](ra::Geometry &self) { return self.type; })
    .def_property_readonly("parameter", [](ra::Geometry &self) { return self.parameter; })
    .def("is_mesh", [](ra::Geometry &self) { return (self.type == ra::Geometry::Mesh); })
    ;
    //
    // ConnectingPoint
    //
    py::class_< ra::ConnectingPoint > connecting_point_cls(m, "RobotAssemblerConnectingPoint");
    py::enum_<ra::ConnectingPoint::PartsType>(connecting_point_cls, "PartsType")
    //'.value("{}", ra::ConnectingPoint::PartsType::{})'
    .value("Parts", ra::ConnectingPoint::PartsType::Parts)
    .value("Rotational", ra::ConnectingPoint::PartsType::Rotational)
    .value("Linear", ra::ConnectingPoint::PartsType::Linear)
    .value("Fixed", ra::ConnectingPoint::PartsType::Fixed)
    .value("Sphere", ra::ConnectingPoint::PartsType::Sphere)
    .value("Plane", ra::ConnectingPoint::PartsType::Plane)
    .value("Spherical", ra::ConnectingPoint::PartsType::Spherical)
    .value("Free", ra::ConnectingPoint::PartsType::Free)
    .value("UNDEFINED", ra::ConnectingPoint::PartsType::UNDEFINED)
    .export_values();
    connecting_point_cls
    .def_property_readonly("name", [](ra::ConnectingPoint &self) { return self.name; })//PointBase
    .def_property_readonly("coords", [](ra::ConnectingPoint &self) { return self.coords; })//PointBase
    .def_property_readonly("description", [](ra::ConnectingPoint &self) { return self.description; })//PointBase
    .def_property_readonly("type", [](ra::ConnectingPoint &self) { return self.getType(); })
    ;
    //
    // Actuator
    //
    py::class_< ra::Actuator, ra::ConnectingPoint > actuator_cls(m, "RobotAssemblerActuator");
    actuator_cls
    .def_property_readonly("axis", [](ra::Actuator &self) { return self.axis; })
    .def_property_readonly("limit", [](ra::Actuator &self) { return self.limit; })
    .def_property_readonly("vlimit", [](ra::Actuator &self) { return self.vlimit; })
    .def_property_readonly("tqlimit", [](ra::Actuator &self) { return self.tqlimit; })
    ;
    //
    // ExtraInfo
    //
    py::class_< ra::ExtraInfo > extra_info_cls(m, "RobotAssemblerExtraInfo");
    py::enum_<ra::ExtraInfo::Type>(extra_info_cls, "Type")
    //'.value("{}", ra::ExtraInfo::Type::{})'.format(nm, nm))
    .value("None", ra::ExtraInfo::Type::None)
    .value("IMU", ra::ExtraInfo::Type::IMU)
    .value("Acceleration", ra::ExtraInfo::Type::Acceleration)
    .value("RateGyro", ra::ExtraInfo::Type::RateGyro)
    .value("Touch", ra::ExtraInfo::Type::Touch)
    .value("Force", ra::ExtraInfo::Type::Force)
    .value("Color", ra::ExtraInfo::Type::Color)
    .value("Distance", ra::ExtraInfo::Type::Distance)
    .value("Camera", ra::ExtraInfo::Type::Camera)
    .value("Depth", ra::ExtraInfo::Type::Depth)
    .value("RGBD", ra::ExtraInfo::Type::RGBD)
    .value("Ray", ra::ExtraInfo::Type::Ray)
    .value("Position", ra::ExtraInfo::Type::Position)
    .value("Light", ra::ExtraInfo::Type::Light)
    .value("PointLight", ra::ExtraInfo::Type::PointLight)
    .value("SpotLight", ra::ExtraInfo::Type::SpotLight)
    .value("DegitalIO", ra::ExtraInfo::Type::DegitalIO)
    .value("AnalogIO", ra::ExtraInfo::Type::AnalogIO)
    .export_values();
    extra_info_cls
    .def_property_readonly("name", [](ra::ExtraInfo &self) { return self.name; })
    .def_property_readonly("type", [](ra::ExtraInfo &self) { return self.type; })
    .def_property_readonly("coords", [](ra::ExtraInfo &self) { return self.coords; })
    .def_property_readonly("description", [](ra::ExtraInfo &self) { return self.description; })
    .def_property_readonly("device_mapping", [](ra::ExtraInfo &self) { return self.device_mapping; })
    .def_property_readonly("parameters", [](ra::ExtraInfo &self) { return self.parameters; })
    ;
    //
    // Parts
    //
    py::class_< ra::Parts > parts_cls(m, "RobotAssemblerParts");
    parts_cls.def(py::init<>())
    .def_property_readonly("number_of_visuals", [](ra::Parts &self) { return self.visual.size(); })//
    .def_property_readonly("number_of_collisions", [](ra::Parts &self) { return self.collision.size(); })//
    .def_property_readonly("number_of_connecting_points", [](ra::Parts &self) { return self.connecting_points.size(); })//
    .def_property_readonly("number_of_actuators", [](ra::Parts &self) { return self.actuators.size(); })//
    .def_property_readonly("number_of_extra_data", [](ra::Parts &self) { return self.extra_data.size(); })//
    .def_property_readonly("mass", [](ra::Parts &self) { return self.mass; })
    .def_property_readonly("center_of_mass", [](ra::Parts &self) { return self.COM; })
    .def_property_readonly("inertia_tensor", [](ra::Parts &self) { return self.inertia_tensor; })
    .def_property_readonly("type", [](ra::Parts &self) { return self.type; })
    .def_property_readonly("class_name", [](ra::Parts &self) { return self.class_name; })
    .def_property_readonly("description", [](ra::Parts &self) { return self.description; })
    .def("visual", [](ra::Parts &self, int idx) {
        try {  return py::cast(self.visual.at(idx)); } catch ( ... ) { }
        return py::cast(nullptr);
    })
    .def("collision", [](ra::Parts &self, int idx) {
        try {  return py::cast(self.collision.at(idx)); } catch ( ... ) { }
        return py::cast(nullptr);
    })
    .def("connecting_points", [](ra::Parts &self, int idx) {
        try {  return py::cast(self.connecting_points.at(idx)); } catch ( ... ) { }
        return py::cast(nullptr);
    })
    .def("actuators", [](ra::Parts &self, int idx) {
        try {  return py::cast(self.actuators.at(idx)); } catch ( ... ) { }
        return py::cast(nullptr);
    })
    .def("extra_data", [](ra::Parts &self, int idx) {
        try {  return py::cast(self.extra_data.at(idx)); } catch ( ... ) { }
        return py::cast(nullptr);
    })
    .def("create_visual", [] (ra::Parts &self,
                             const std::string &_dir, const std::vector<double> &_col) {
        SgGroup *ptr = new SgGroup();
        ptr->setName("xxx");
        Vector3f col = Vector3f::Zero();
        for(int i = 0; i < _col.size() && i < 3; i++) col(i) = _col[i];
        ra::createSceneFromGeometry(ptr, self.visual, _dir, col); // Body
        return ptr;
    }, py::arg("project_dir") = std::string(), py::arg("color") = std::vector<double>())
    ;
    //
    // Settings
    //
    py::class_< ra::Settings, ra::SettingsPtr > settings_cls(m, "RobotAssemblerSettings");
    settings_cls.def(py::init<>())
    .def_property_readonly("number_of_parts", [](ra::Settings &self) { return self.mapParts.size(); } )
    .def_property_readonly("parts_names", [](ra::Settings &self) {
        std::vector<std::string> lst_;
        for( auto it = self.mapParts.begin(); it != self.mapParts.end(); it++ ) {
            lst_.push_back(it->first);
        }
        return lst_; })
    .def("parts", [](ra::Settings &self, const int idx) {
        int index = 0;
        for( auto it = self.mapParts.begin(); it != self.mapParts.end(); it++, index++ ) {
            if (index == idx) return py::cast(it->second);
        }
        return py::cast(nullptr);
    })
    .def("parts", [](ra::Settings &self, const std::string &name) {
        try {  return py::cast(self.mapParts.at(name));  } catch ( ... ) { }
        return py::cast(nullptr);
    })
    .def("parseYaml", &ra::Settings::parseYaml)
    ;
}

} // namespace cnoid
