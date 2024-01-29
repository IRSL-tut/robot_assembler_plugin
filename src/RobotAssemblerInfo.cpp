#include "RobotAssemblerInfo.h"
#include <cnoid/YAMLWriter>
#include "irsl_debug.h"

using namespace cnoid;
using namespace cnoid::robot_assembler;

MappingPtr cnoid::robot_assembler::parseInfo(const std::string &_fname)
{
    DEBUG_STREAM(" rd: " << _fname);
    YAMLReader yrdr;
    if (!yrdr.load(_fname)) return nullptr;
    DEBUG_STREAM("load: " << _fname);
    if(yrdr.numDocuments() < 1) return nullptr;
    DEBUG_STREAM("doc: " << yrdr.numDocuments());
    for(int i = 0; i < yrdr.numDocuments(); i++) {
        ValueNode *nd = yrdr.document(i);
        if(nd->isValid() && nd->isMapping()) {
            DEBUG_STREAM("map: " << i);
            MappingPtr ret = nd->toMapping();
            if (ret->find("robot-info")->isValid() ||
                ret->find("parts-info")->isValid() ||
                ret->find("actuator-info")->isValid()) {
                DEBUG_STREAM("OK: " << _fname);
                return ret;
            }
        }
    }
    return nullptr;
}
MappingPtr cnoid::robot_assembler::createInfo(RoboasmRobotPtr _rb)
{
    RoboasmPartsPtr root_ = _rb->rootParts();
    if(!root_) return nullptr;
    MappingPtr ret = new Mapping();
    MappingPtr rb_info = new Mapping();
    addToMapping(rb_info, "name", _rb->name());
    coordinates cds_ = *root_;
    addCoordsToMapping(rb_info, "initial-coords", cds_);
    ret->insert("robot-info", rb_info);
    MappingPtr pt_info = new Mapping();
    bool added = false;
    partsPtrList lst;
    _rb->allParts(lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        if((*it)->color.isZero()) continue;
        MappingPtr mp_ = new Mapping();
        addToMapping(mp_, "color", (*it)->color);
        pt_info->insert((*it)->name(), mp_);
        added = true;
    }
    if(added) {
        ret->insert("parts-info", pt_info);
    }
    return ret;
}
bool cnoid::robot_assembler::mergeInfo(Mapping *_dst, Mapping *_src)
{
    if(!_dst || !_src) return false;
    Mapping *d_parts = _dst->findMapping("parts-info");
    Mapping *s_parts = _src->findMapping("parts-info");
    if(d_parts->isValid() && s_parts->isValid()) {
        for(auto it = s_parts->begin(); it != s_parts->end(); it++) {
            Mapping *mm_ = d_parts->findMapping(it->first);
            if(!mm_->isValid()) {
                d_parts->insert(it->first, it->second);
            }
        }
    } else if (!d_parts->isValid() && s_parts->isValid()) {
        _dst->insert("parts-info", s_parts);
    }
    Mapping *d_act = _dst->findMapping("actuator-info");
    Mapping *s_act = _src->findMapping("actuator-info");
    if(d_act->isValid() && s_act->isValid()) {
        for(auto it = s_parts->begin(); it != s_parts->end(); it++) {
            Mapping *mm_ = d_parts->findMapping(it->first);
            if(!mm_->isValid()) {
                d_parts->insert(it->first, it->second);
            }
        }
    } else if (!d_act->isValid() && s_act->isValid()) {
        _dst->insert("actuator-info", s_act);
    }
    return true;
}
//
static inline void _rename(std::string &_in, StringMap _rmap)
{
    auto it = _rmap.find(_in);
    if (it != _rmap.end()) {
        _in = it->second;
    }
}
static inline void _rename_connecting_point(std::string &_in, StringMap _rmap)
{
    // position of "/"
    size_t len = _in.size();
    size_t p_ = _in.rfind("/");
    if (p_ >= len) return;

    std::string str_pt_ = _in.substr(0, p_);
    std::string str_cp_ = _in.substr(p_+1);
    _rename(str_pt_, _rmap);
    _in = str_pt_ + "/" + str_cp_;
}
bool cnoidRAInfo::renameInfo(StringMap &_rmap)
{
    bool res_ = true;
    {   // rename parts-info
        Mapping *tmp = info->findMapping("parts-info");
        if(tmp->isValid()) {
            std::vector<std::string> keys;
            // store keys
            for(auto it = tmp->begin(); it != tmp->end(); it++) {
                keys.push_back(it->first);
            }
            // rename keys
            for(auto it = keys.begin(); it != keys.end(); it++) {
                ValueNodePtr v = tmp->extract(*it);
                std::string new_name(*it);
                _rename(new_name, _rmap);
                tmp->insert(new_name, v);
            }
        }
    }
    {   // rename actuator-info
        Mapping *tmp = info->findMapping("actuator-info");
        if(tmp->isValid()) {
            std::vector<std::string> keys;
            // store keys
            for(auto it = tmp->begin(); it != tmp->end(); it++) {
                keys.push_back(it->first);
            }
            // rename keys
            for(auto it = keys.begin(); it != keys.end(); it++) {
                ValueNodePtr v = tmp->extract(*it);
                std::string new_name(*it);
                _rename_connecting_point(new_name, _rmap);
                tmp->insert(new_name, v);
            }
        }
    }
    return res_;
}
//
bool cnoidRAFile::dumpRoboasm(const std::string &_filename)
{
    MappingPtr mp_ = historyToMap();
    addInfo(mp_);

    YAMLWriter ywtr;
    ywtr.setMessageSink(std::cerr);
    //ywtr.setDoubleFormat("%12.12f");
    if(!ywtr.openFile(_filename)) {
        return false;
    }
    ywtr.putNode(mp_);
    ywtr.flush();
    ywtr.closeFile();
    return true;
}
MappingPtr cnoidRAFile::addInfo(MappingPtr _main)
{
    if (!_main) {
        _main = new Mapping();
    }
    {
        Mapping *tmp = info->findMapping("robot-info");
        if(tmp->isValid()) {
            _main->insert("robot-info", tmp);
        }
    }
    {
        Mapping *tmp = info->findMapping("parts-info");
        if(tmp->isValid()) {
            _main->insert("parts-info", tmp);
        }
    }
    {
        Mapping *tmp = info->findMapping("actuator-info");
        if(tmp->isValid()) {
            _main->insert("actuator-info", tmp);
        }
    }
    {
        Mapping *tmp = info->findMapping("device-info");
        if(tmp->isValid()) {
            _main->insert("device-info", tmp);
        }
    }
    return _main;
}
MappingPtr cnoidRAFile::historyToMap(MappingPtr _main)
{
    ListingPtr hist = new Listing();
    for(auto it = history.begin(); it != history.end(); it++) {
        MappingPtr hitm = new Mapping();
        addToMapping(hitm, "parts-name", (*it).parts_name);
        addToMapping(hitm, "parts-type", (*it).parts_type);
        if(!(*it).initial_parts) {
            if((*it).parts_point.size() > 0)
                addToMapping(hitm, "parts-point", (*it).parts_point);
            if((*it).parts_point.size() == 0 && (*it).parts_point_url.size() > 0)
                addToMapping(hitm, "parts-point-url", (*it).parts_point_url);
            if((*it).parent.size() > 0)
                addToMapping(hitm, "parent", (*it).parent);
            if((*it).parent_point.size() > 0)
                addToMapping(hitm, "parent-point", (*it).parent_point);
            if( ((*it).parent.size() == 0 || (*it).parent_point.size() == 0) &&
                (*it).parent_point_url.size() > 0 ) {
                addToMapping(hitm, "parent-point-url", (*it).parent_point_url);
            }
            if((*it).connecting_offset.isInitial()) {
                    // no config
            } else {
                if(!(*it).connecting_offset.isInitial(1e-12)) {
                    addCoordsToMapping(hitm, "connecting-offset", (*it).connecting_offset);
                }
            }
        } else {
            addToMapping(hitm, "initial-parts", true);
        }
        hist->append(hitm);
    }
    //
    MappingPtr main_map;
    if (!_main) {
        main_map = new Mapping();
    } else {
        main_map = _main;
    }
    main_map->insert("history", hist);
    return main_map;
}
bool cnoidRAFile::updateRobotByInfo(RoboasmRobotPtr _rb)
{
    DEBUG_PRINT();
    coordinates cds_;
    if(getRobotCoords(cds_)) {
        DEBUG_STREAM(" initial-coords : " << cds_);
        RoboasmPartsPtr root_ = _rb->rootParts();
        if(!!root_) {
            DEBUG_STREAM(" set initial to root(" << root_->name() << ")");
            root_->newcoords(cds_);
        }
        _rb->updateDescendants();
    }
    partsPtrList lst;
    _rb->allParts(lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        Mapping *mp_ = getPartsInfo((*it)->name());
        if(!mp_) continue;
        DEBUG_STREAM(" name: " << (*it)->name());
        Vector3f col_;
        if(readFromMapping(mp_, "color", col_)) {
            (*it)->color = col_;
        }
    }
    return true;
}
RoboasmRobotPtr cnoidRAFile::makeRobot(RoboasmUtil &util, const std::string &name, bool rename)
{
    if(rename) {
        StringMap rmap_;
        util.renamePartsHistory(this->history, rmap_);
        this->renameInfo(rmap_);
    }
    std::string name_;
    if(!name.empty()) {
        name_ = name;
    } else if(!this->getRobotName(name_)) {
        name_ = "AssembleRobot";
    }
    RoboasmRobotPtr rb_ = util.makeRobot(name_, this->history);
    if(!!rb_) {
        this->updateRobotByInfo(rb_);
    }
    return rb_;
}
