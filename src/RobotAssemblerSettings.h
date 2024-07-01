#ifndef CNOID_ROBOT_ASSEMBLER_SETTINGS_H
#define CNOID_ROBOT_ASSEMBLER_SETTINGS_H

#include "irsl_choreonoid/Coordinates.h"

#include <string>
#include <vector>
#include <memory>
#include <map>
//#include <set>
#include <limits>

#include <cnoid/ValueTree>

#include "exportdecl_lib.h"

namespace cnoid {
namespace robot_assembler {

typedef long ConnectingTypeID;
typedef long ConnectingConfigurationID;
struct ConnectingConfiguration;
struct ConnectingTypeMatch;
class PointBase;
class ConnectingPoint;
class Actuator;
struct ExtraInfo;
struct Geometry;
class Parts;
typedef std::shared_ptr<Parts> PartsPtr;
typedef std::map<std::string, std::string> StringMap;

#if 0
struct ConnectingConstraint
{
    enum Type {
        None,
        Rotational_X = 1 << 0, // param(0), limit(2), dof(1)
        Rotational_Y = 1 << 1, // param(0), limit(2), dof(1)
        Rotational_Z = 1 << 2, // param(0), limit(2), dof(1)
        Rotational   = 1 << 3, // param(3), limit(2), dof(1) ? angle_axis
        Rotational_XY = 1 << 4, // param(0), limit(4), dof(2)
        Rotational_YZ = 1 << 5, // param(0), limit(4), dof(2)
        Rotational_ZX = 1 << 6, // param(0), limit(4), dof(2)
        Rotational_2D = 1 << 7, // param(6), limit(4), dof(2) ? angle_axis * angle_axis ??
        Free_Rotate   = 1 << 8, // param(0), limit(6), dof(3)
        Linear_X = 1 << 9,  // param(0), limit(2), dof(1)
        Linear_Y = 1 << 10, // param(0), limit(2), dof(1)
        Linear_Z = 1 << 11, // param(0), limit(2), dof(1)
        Linear   = 1 << 12, // param(6), limit(2), dof(1)
        Plane_XY = 1 << 13, // param(0), limit(4), dof(2)
        Plane_YZ = 1 << 14, // param(0), limit(4), dof(2)
        Plane_ZX = 1 << 15, // param(0), limit(4), dof(2)
        Plane    = 1 << 16, // param(6), limit(4), dof(2)
        Free_Translate = 1 << 17, // param(0), limit(6), dof(3)
        ParemetricLine,
        ParametricSurface,
        ParametricVolume,
    };
  std::string name;
  std::string description
  Type type;
  //type // plane
  std::vector<double> parameter;
};
struct ConstraintTypeMatch
{
    ConnectingTypeID pair[2];
    ConnectingConstraint constraint;
};
struct ConstraintConfiguration
{
    coordinates coords;
    std::vector<double> parameter;
};
#endif
struct CNOID_EXPORT ConnectingType
{
    std::string name;
    //
    ConnectingTypeID index;
};
struct CNOID_EXPORT ConnectingConfiguration
{
    std::string name;
    std::string description;
    coordinates coords;
    //
    ConnectingConfigurationID index;
};
struct CNOID_EXPORT ConnectingTypeMatch
{
    ConnectingTypeID pair[2];
    std::vector<ConnectingConfigurationID> allowed_configuration;
    //
    long index;
};
class CNOID_EXPORT PointBase
{
public:
    std::string name;
    coordinates coords;
    std::string description;
};
class CNOID_EXPORT ConnectingPoint : public PointBase
{
public:
    enum PartsType {
        Parts = 0, // Parts
        Rotational = 1 << 0, // Actuator
        Linear     = 1 << 1,
        Fixed      = 1 << 2,
        Sphere     = 1 << 3,
        Plane      = 1 << 4,
        Spherical  = 1 << 5,
        Free       = 1 << 6,
        UNDEFINED  = 1 << 7
    };

    ConnectingPoint() : type(Parts) {}
    virtual PartsType getType() { return type; }
    //[TODO]
    // reverse configuration
    // reverse-search coords->configuration
    std::vector<ConnectingTypeID> type_list;
protected:
    PartsType type;
};
class CNOID_EXPORT Actuator : public ConnectingPoint
{
public:
    Actuator() {
        axis.Zero();
        limit[0] = - std::numeric_limits<double>::infinity();
        limit[1] = std::numeric_limits<double>::infinity();
        vlimit[0] = - std::numeric_limits<double>::infinity();
        vlimit[1] =  std::numeric_limits<double>::infinity();
        tqlimit[0] = - std::numeric_limits<double>::infinity();
        tqlimit[1] =  std::numeric_limits<double>::infinity();
        type = UNDEFINED;
    }
    Actuator(PartsType _tp) : Actuator() { type = _tp; }

    Vector3 axis;
    double limit[2];
    double vlimit[2];
    double tqlimit[2];
};
struct CNOID_EXPORT ExtraInfo
{
    enum Type {
        None,
        IMU,
        Acceleration,
        RateGyro,
        Touch,
        Force,
        Color,
        Distance,
        Camera,
        Depth,
        RGBD,
        Ray,
        Position,
        Light,
        PointLight,
        SpotLight,
        DegitalIO,
        AnalogIO,
    };
    std::string name;
    Type type;
    coordinates coords;
    std::string description;
    std::string device_mapping;
    std::vector<double> parameters;
};
struct CNOID_EXPORT Geometry
{
    enum Type {
        None,
        Mesh,
        Box,
        Cylinder,
        Sphere,
        Cone,
        Capsule,
        Ellipsoid,
        Dummy // can not detect collision
    };
    Geometry() : scale(1.0), color(Vector3f::Zero()) {}
    coordinates coords;
    std::string url;
    double scale;
    Type type;
    std::vector<double> parameter;
    Vector3f color; // should be material?
    Parts *parent_parts;
    //std::string description;
};
class CNOID_EXPORT Parts
{
public:
    Parts() : hasMassParam(false), mass(0.0)
    {
        COM.setZero();
        inertia_tensor.setZero();
    }

    std::string type;
    std::string class_name;
    std::string description;
    std::vector<Geometry> visual;
    std::vector<Geometry> collision;

    bool hasMassParam;
    double mass;
    Vector3 COM; // center of mass
    Matrix3 inertia_tensor;

    std::vector<ConnectingPoint> connecting_points;
    std::vector<Actuator> actuators;
    std::vector<ExtraInfo> extra_data;
};
////
class CNOID_EXPORT Settings
{
public:
    Settings();
    ~Settings();

    std::vector<ConnectingType> listConnectingType;
    std::vector<ConnectingConfiguration> listConnectingConfiguration;
    std::vector<ConnectingTypeMatch> listConnectingTypeMatch;

    std::map<std::string, Parts> mapParts;

    bool parseYaml(const std::string &filename);
    ConnectingTypeMatch *searchMatch(ConnectingTypeID _a, ConnectingTypeID _b);
    ConnectingTypeMatch *searchConnection(ConnectingTypeID _a, ConnectingTypeID _b,
                                          ConnectingConfigurationID _tp);
    ConnectingType *searchConnectingType(const std::string &_name);
    ConnectingConfiguration *searchConnectingConfiguration(const std::string &_name);
    //int searchMatch(ConnectingTypeID a, ConnectingTypeID b);
    // match / invert?
    // A, A => parent/child <-
    // B, C => parent/child <-
    ConnectingTypeID searchConnectingTypeID(const std::string &name) {
        ConnectingType *res = searchConnectingType(name);
        if (!!res) {
            return res->index;
        }
        return -1;
    }
    ConnectingConfigurationID searchConnectingConfigurationID(const std::string &name) {
        ConnectingConfiguration *res = searchConnectingConfiguration(name);
        if (!!res) {
            return res->index;
        }
        return -1;
    }
// TODO
// add ConnectingType
// add ConnectingConfiguration
// add ConnectingTypeMatch
    bool validateParts(const Parts &pt);
    bool parsePartsFromString(const std::string &settings, std::vector<Parts> &results);
    bool parsePartsFromYaml(const std::string &filename, std::vector<Parts> &results);
    bool parsePartsFromNode(ValueNode *val, std::vector<Parts> &results);// depend cnoid
    bool insertParts(const Parts &pt);
    bool insertPartsFromString(const std::string &settings);
    bool insertPartsFromYaml(const std::string &filename);
    bool insertPartsFromNode(ValueNode *val);// depend cnoid

private:
    class Impl;
    Impl *impl;
};
typedef std::shared_ptr<Settings> SettingsPtr;

class CNOID_EXPORT AttachHistoryItem
{
public:
    AttachHistoryItem() : initial_parts(false) {}
    std::string parts_name;
    std::string parts_type;
    std::string parts_point;
    std::string parts_point_url; // parts_name/parts_point | backward-compatibility
    std::string parent;
    std::string parent_point;
    std::string parent_point_url; // parent_name/parent_point | backward-compatibility
    coordinates connecting_offset; //
    std::string config_name; // config-name | backward-compatibility
    bool initial_parts;

    std::ostream &print(std::ostream &ostr)
    {
        ostr << "(";
        if (parts_name.size() > 0) {
            ostr << "(:parts-name " << parts_name << ")";
        }
        if (parts_type.size() > 0) {
            ostr << "(:parts-type " << parts_type << ")";
        }
        if (parts_point.size() > 0) {
            ostr << "(:parts-point " << parts_point << ")";
        }
        if (parts_point_url.size() > 0) {
            ostr << "(:parts-point-url " << parts_point_url << ")";
        }
        if (parent.size() > 0) {
            ostr << "(:parent " << parent << ")";
        }
        if (parent_point.size() > 0) {
            ostr << "(:parent-point " << parent_point << ")";
        }
        if (parent_point_url.size() > 0) {
            ostr << "(:parent-point-url " << parent_point_url << ")";
        }
        if (config_name.size() > 0) {
            ostr << "(:config-name " << config_name << ")";
        }
        ostr << "(:connecting-offset ((";
        AngleAxis ax_(connecting_offset.rot);
        ostr << connecting_offset.pos(0) << " ";
        ostr << connecting_offset.pos(1) << " ";
        ostr << connecting_offset.pos(2) << ") (";
        ostr << ax_.axis()(0) << " ";
        ostr << ax_.axis()(1) << " ";
        ostr << ax_.axis()(2) << " ";
        ostr << ax_.angle() << "))";
        if(initial_parts) {
            ostr << "(:initial-parts t)";
        }
        ostr << ")";
        return ostr;
    }
};
typedef std::vector<AttachHistoryItem> AttachHistory;
//typedef std::map<std::string, AttachHistoryItem*> historyItemMap;
//typedef std::pair<std::string, AttachHistoryItem*> historyItemPair;

struct CNOID_EXPORT AssembleConfig
{
    std::string robot_name;
    coordinates initial_coords;
    StringMap actuator_name;
    StringMap actuator_axis_name;
    std::map<std::string, Vector3> actuator_axis_vector;

    bool isValid() {
        if(robot_name.size() > 0) return true;
        if(!initial_coords.isInitial()) return true;
        if(actuator_name.size() > 0) return true;
        if(actuator_axis_name.size() > 0) return true;
        if(actuator_axis_vector.size() > 0) return true;
        return false;
    }
};
class CNOID_EXPORT RoboasmFile
{
public:
    RoboasmFile() {}
    RoboasmFile(const std::string &_filename)
    {
        valid_ = this->parseRoboasm(_filename);
    }

    bool isValid() { return valid_; }
    AttachHistory  history;
    AssembleConfig  config;

    virtual bool parseRoboasm(const std::string &_filename, bool parse_config = true);
    virtual bool dumpRoboasm(const std::string &_filename);
    virtual bool parseRoboasmFromString(const std::string &yml_string, bool parse_config = true);
    virtual bool dumpRoboasmToString(std::string &result_yml);
protected:
    bool valid_;
};

} }
#endif
