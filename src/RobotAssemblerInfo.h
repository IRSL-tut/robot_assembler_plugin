#ifndef CNOID_ROBOT_ASSEMBLER_INFO_H
#define CNOID_ROBOT_ASSEMBLER_INFO_H

#include "RobotAssembler.h"
#include "ValueTreeUtil.h"
#include <cnoid/ValueTree>
#include "irsl_debug.h"

namespace cnoid {
namespace robot_assembler {
MappingPtr parseInfo(const std::string &_fname);
MappingPtr createInfo(RoboasmRobotPtr _rb);
bool mergeInfo(Mapping *_dst, Mapping *_src);

inline Mapping *getMapping(Mapping *_mp, const std::string &_k1)
{
    if(!_mp) return nullptr;
    Mapping *tmp_ = _mp->findMapping(_k1);
    if(!tmp_->isValid()) return nullptr;
    return tmp_;
}
inline Mapping *getMapping(Mapping *_mp, const std::string &_k1, const std::string &_k2)
{
    if(!_mp) return nullptr;
    Mapping *tmp_ = _mp->findMapping(_k1);
    if(!tmp_->isValid()) return nullptr;
    Mapping *ret = tmp_->findMapping(_k2);
    if(!ret->isValid()) return nullptr;
    return ret;
}

class cnoidRAFile : public RoboasmFile
{
public:
    cnoidRAFile() {};
    cnoidRAFile(const std::string &_filename) : RoboasmFile()
    {
        valid_ = this->parseRoboasm(_filename);
    }
    MappingPtr info;
    //ignore config
    virtual bool parseRoboasm(const std::string &_filename, bool parse_config = true) override
    {
        RoboasmFile::parseRoboasm(_filename, false);
        info = parseInfo(_filename);
        DEBUG_STREAM(" info: " << !!info);
        if(history.size() < 0 && !info) return false;
        return true;
    }
    virtual bool dumpRoboasm(const std::string &_filename) override;
    MappingPtr historyToMap(MappingPtr _main = nullptr);
    bool updateRobotByInfo(RoboasmRobotPtr _rb);

    Mapping *getRobotInfo()
    {
        return getMapping(info, "robot-info");
    }
    Mapping *getPartsInfo(const std::string &_ptname)
    {
        return getMapping(info, "parts-info", _ptname);
    }
    Mapping *getActuatorInfo(const std::string &_actname)
    {
        return getMapping(info, "actuator-info", _actname);
    }
    bool getRobotName(std::string &_res)
    {
        Mapping *mp_ = getRobotInfo();
        if(!mp_) return false;
        return readFromMapping(mp_, "name", _res);
    }
    bool getRobotCoords(coordinates &_res)
    {
        Mapping *mp_ = getRobotInfo();
        if(!mp_) return false;
        return readFromMapping(mp_, "initial-coords", _res);
    }
};
} }
#endif

#if 0
robot-info:
  
parts-info:
  partsname0: // name of parts
    name:
    color:
actuator-info:
  actuatorname0:
    name:
    axis:
    limit:
    vlimit:
    tqlimit:
#endif
