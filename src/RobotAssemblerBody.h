#ifndef CNOID_ROBOT_ASSEMBLER_BODY_H
#define CNOID_ROBOT_ASSEMBLER_BODY_H

#include "RobotAssembler.h"
#include <cnoid/Body>
#include <cnoid/ValueTree>

namespace cnoid {
namespace robot_assembler {

extern const Vector3f default_body_color;

void CNOID_EXPORT createSceneFromGeometry(SgGroup *sg_main, std::vector<Geometry> &geom_list,
                             const std::string &_proj_dir = std::string(), const Vector3f &_color = Vector3f::Zero());
void CNOID_EXPORT createSceneFromGeometry(SgGroup *sg_main, std::vector<Geometry> &geom_list, const Vector3f &_color);

class CNOID_EXPORT RoboasmBodyCreator
{
public:
    RoboasmBodyCreator(const std::string &_proj_dir = std::string());

    void setName(const std::string &_nm) { name = _nm; }
    void setMergeFixedJoint(bool on = true) { merge_fixed_joint = on; };
    BodyPtr createBody(RoboasmRobotPtr _rb, const std::string &_name = std::string());
    BodyPtr createBody(RoboasmRobotPtr _rb, MappingPtr _info)
    {
        info = _info;
        return createBody(_rb);
    }
protected:
    std::string name;
    std::string project_directory;
    BodyPtr body;
    int joint_counter;
    bool merge_fixed_joint;
    std::map<std::string, std::string> map_link_cnoid_roboasm;  // key: cnoid_name, value: roboasm_name
    //std::map<std::string, std::string> map_joint_cnoid_roboasm; // key: cnoid_name, value: roboasm_name
    //std::vector<std::string> joint_list; // cnoid_name
    MappingPtr info; // for accessing local info
    RoboasmRobotPtr currentRobot; // for accessing local info

    BodyPtr _createBody(RoboasmRobotPtr _rb, const std::string &_name);
    Link *createLink(RoboasmPartsPtr _pt, bool _is_root = false);
    bool appendChildLink(BodyPtr _bd, Link *_lk, RoboasmPartsPtr _pt);
    bool mergeFixedJoint(BodyPtr _bd);
};

} }
#endif
