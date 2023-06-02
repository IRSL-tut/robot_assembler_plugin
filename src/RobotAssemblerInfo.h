#ifndef CNOID_ROBOT_ASSEMBLER_INFO_H
#define CNOID_ROBOT_ASSEMBLER_INFO_H

#include "RobotAssembler.h"
#include "ValueTreeUtil.h"
#include <cnoid/ValueTree>

namespace cnoid {
namespace robot_assembler {
MappingPtr CNOID_EXPORT parseInfo(const std::string &_fname);
MappingPtr CNOID_EXPORT createInfo(RoboasmRobotPtr _rb);
bool CNOID_EXPORT mergeInfo(Mapping *_dst, Mapping *_src);

inline void addMapping(Mapping *_main, const std::string &_k1, Mapping *_tgt)
{
    if(!_main) return;
    Mapping *tmp_ = _main->findMapping(_k1);
    if(!tmp_->isValid()) {
        _main->insert(_k1, _tgt);
    } else {
        for(auto it = _tgt->begin(); it != _tgt->end(); it++) {
            tmp_->insert(it->first, it->second);
        }
    }
}
inline void addMapping(Mapping *_main, const std::string &_k1, const std::string &_k2, Mapping *_tgt)
{
    if(!_main) return;
    Mapping *tmp_ = _main->findMapping(_k1);
    if(!tmp_->isValid()) {
        Mapping *m1_ = new Mapping();
        m1_->insert(_k2, _tgt);
        _main->insert(_k1, m1_);
        return;
    }
    addMapping(tmp_, _k2, _tgt);
}
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
inline Mapping *getRobotInfo(Mapping *info)
{
    return getMapping(info, "robot-info");
}
inline Mapping *getPartsInfo(Mapping *info, const std::string &_ptname)
{
    return getMapping(info, "parts-info", _ptname);
}
inline Mapping *getActuatorInfo(Mapping *info, const std::string &_actname)
{
    return getMapping(info, "actuator-info", _actname);
}
inline Mapping *getDeviceInfo(Mapping *info, const std::string &_devname)
{
    return getMapping(info, "device-info", _devname);
}
inline bool getRobotName(Mapping *info, std::string &_res)
{
    Mapping *mp_ = getRobotInfo(info);
    if(!mp_) return false;
    return readFromMapping(mp_, "name", _res);
}
inline bool getRobotCoords(Mapping *info, coordinates &_res)
{
    Mapping *mp_ = getRobotInfo(info);
    if(!mp_) return false;
    return readFromMapping(mp_, "initial-coords", _res);
}

inline bool getPartsName(Mapping *info, const std::string &_pt, std::string &_res)
{
    Mapping *mp_ = getPartsInfo(info, _pt);
    if(!mp_) return false;
    return readFromMapping(mp_, "name", _res);
}
inline bool getPartsColor(Mapping *info, const std::string &_pt, Vector3f &_res)
{
    Mapping *mp_ = getPartsInfo(info, _pt);
    if(!mp_) return false;
    return readFromMapping(mp_, "color", _res);
}
inline bool getActuatorName(Mapping *info, const std::string &_ac, std::string &_res)
{
    Mapping *mp_ = getActuatorInfo(info, _ac);
    if(!mp_) return false;
    return readFromMapping(mp_, "name", _res);
}
inline bool getActuatorLimit(Mapping *info, const std::string &_ac, const std::string &_ky, double &a, double &b)
{
    Mapping *mp_ = getActuatorInfo(info, _ac);
    if(!mp_) return false;
    return readFromMapping(mp_, _ky, a, b);
}
inline bool getDeviceName(Mapping *info, const std::string &_pt, const std::string &_dev, std::string &_res)
{
    std::string nm_ = _pt + "/" + _dev;
    Mapping *mp_ = getDeviceInfo(info, nm_);
    if(!mp_) return false;
    return readFromMapping(mp_, "name", _res);
}
class CNOID_EXPORT cnoidRAInfo
{
public:
    cnoidRAInfo(MappingPtr _info = nullptr) : info(_info) {}
    MappingPtr info;

    Mapping *getRobotInfo()
    {
        return cnoid::robot_assembler::getRobotInfo(info);
    }
    Mapping *getPartsInfo(const std::string &_ptname)
    {
        return cnoid::robot_assembler::getPartsInfo(info, _ptname);
    }
    Mapping *getActuatorInfo(const std::string &_actname)
    {
        return cnoid::robot_assembler::getActuatorInfo(info, _actname);
    }
    bool getRobotName(std::string &_res)
    {
        return cnoid::robot_assembler::getRobotName(info, _res);
    }
    bool getRobotCoords(coordinates &_res)
    {
        return cnoid::robot_assembler::getRobotCoords(info, _res);
    }
    bool getPartsName(const std::string &_pt, std::string &_res)
    {
        return cnoid::robot_assembler::getPartsName(info, _pt, _res);
    }
    bool getPartsColor(const std::string &_pt, Vector3f &_res)
    {
        return cnoid::robot_assembler::getPartsColor(info, _pt, _res);
    }
    bool getActuatorName(const std::string &_ac, std::string &_res)
    {
        return cnoid::robot_assembler::getActuatorName(info, _ac, _res);
    }
    bool getActuatorLimit(const std::string &_ac, const std::string &_ky, double &a, double &b)
    {
        return cnoid::robot_assembler::getActuatorLimit(info, _ac, _ky, a, b);
    }
    bool getDeviceName(const std::string &_pt, const std::string &_dev, std::string &_res)
    {
        return cnoid::robot_assembler::getDeviceName(info, _pt, _dev, _res);
    }
    bool renameInfo(StringMap &_rmap);
};

class CNOID_EXPORT cnoidRAFile : public RoboasmFile, public cnoidRAInfo
{
public:
    cnoidRAFile() {};
    cnoidRAFile(const std::string &_filename) : RoboasmFile(), cnoidRAInfo()
    {
        valid_ = this->parseRoboasm(_filename);
    }
    //ignore config
    virtual bool parseRoboasm(const std::string &_filename, bool parse_config = true) override
    {
        RoboasmFile::parseRoboasm(_filename, false);
        info = parseInfo(_filename);
        if(history.size() < 0 && !info) return false;
        return true;
    }
    virtual bool dumpRoboasm(const std::string &_filename) override;
    MappingPtr historyToMap(MappingPtr _main = nullptr);
    bool updateRobotByInfo(RoboasmRobotPtr _rb);
};
} }
#endif

#if 0
robot-info:
  name:
  initial-coords:
parts-info:
  partsname0: // name of parts
    name:
    color:
actuator-info:
  actuatorname0: // name of actuator
    name:
    axis:
    id:
    direction:
    offset:
    initial:
    limit:
    vlimit:
    tqlimit:
device-info:
  devicename0:
    name:
#endif
