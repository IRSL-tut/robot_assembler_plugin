#include "RobotAssembler.h"

#include <iostream>
#include <sstream>
#include <algorithm> //std::find

#ifndef _WIN32
// get pid
#include <sys/types.h>
#include <unistd.h>
#else
#include <process.h>
#endif

//#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;
using namespace cnoid::robot_assembler;

void static print(coordinates &cds)
{
    std::cout << "((" << cds.pos(0) << " "
              << cds.pos(1) << " " << cds.pos(2);
    Vector3 rpy; cds.getRPY(rpy);
    std::cout << ") (" << rpy(0)  << " " << rpy(1)  << " "
              << rpy(2) << "))";
}

//// [roboasm coords] ////
RoboasmCoords::RoboasmCoords(const std::string &_name)
    : parent_ptr(nullptr)
{
    name_str = _name;
}
RoboasmCoords::~RoboasmCoords()
{
    //DEBUG_STREAM(" [" << this << "] " << name_str);
}
// virtual ??
void RoboasmCoords::update()
{
    if(!!parent_ptr) {
#if 0
        std::cout << "this  : " << this << std::endl;
        std::cout << "parent: " << parent_ptr << std::endl;
        std::cout << "world(org_this) :";
        print(buf_worldcoords); std::cout << std::endl;
#endif
        parent_ptr->copyWorldcoords(buf_worldcoords);
#if 0
        std::cout << "world(parent)   :";
        print(buf_worldcoords); std::cout << std::endl;
        std::cout << "this (coords)   :";
        print(*dynamic_cast<coordinates *>(this)); std::cout << std::endl;
#endif
        buf_worldcoords.transform(*this);
#if 0
        std::cout << "world(new_this) :";
        print(buf_worldcoords); std::cout << std::endl;
#endif
        updateDescendants();
    } else {
        buf_worldcoords = *this;
        updateDescendants();
    }
}
void RoboasmCoords::assoc(RoboasmCoordsPtr c)
{
    if ( ! _existing_descendant(c) ) {
        //c->_replaceParent(this);
        if (!!(c->parent_ptr)) {
            c->parent_ptr->_dissoc(c.get());
        }
        c->parent_ptr = this;
        coordinates newcoords;
        buf_worldcoords.transformation(newcoords, c->buf_worldcoords);
        c->newcoords(newcoords);
        descendants.insert(c);
    }
}
bool RoboasmCoords::_dissoc(RoboasmCoords *c)
{
    if (c->parent_ptr == this) {
        _erase_descendant(c);
        c->parent_ptr = nullptr;
        c->newcoords(c->buf_worldcoords);
        return true;
    }
    return false;
}
bool RoboasmCoords::isDescendant(RoboasmCoordsPtr c)
{
    if(this == c.get()) return false;
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        if(*it == c) return true;
        if((*it)->isDescendant(c)) return true;
    }
    return false;
}
RoboasmCoordsPtr RoboasmCoords::isDescendant(RoboasmCoords *c)
{
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        if((*it).get() == c) return *it;
        return (*it)->isDescendant(c);
    }
    return nullptr;
}
void RoboasmCoords::toRootList(coordsList &lst)
{
    lst.push_back(this);
    if (!!parent_ptr) {
        parent_ptr->toRootList(lst);
    }
}
void RoboasmCoords::toRootList(coordsPtrList &lst)
{
    if (!!parent_ptr) {
        RoboasmCoordsPtr ptr = parent_ptr->isDirectDescendant(this);
        if(!!ptr) {
            lst.push_back(ptr);
            parent_ptr->toRootList(lst);
        }
    }
}
void RoboasmCoords::directDescendants(coordsPtrList &lst)
{
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        lst.push_back(*it);
    }
}
template <typename T>
void RoboasmCoords::directDescendants(std::vector< std::shared_ptr < T > >&lst)
{
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        std::shared_ptr<T> p = std::dynamic_pointer_cast<T> (*it);
        if (!!p) lst.push_back(p);
    }
}
template void RoboasmCoords::directDescendants<RoboasmConnectingPoint>(connectingPointPtrList &lst);
template void RoboasmCoords::directDescendants<RoboasmParts>(partsPtrList &lst);
void RoboasmCoords::allDescendants(coordsList &lst)
{
    // including self
    lst.push_back(this);
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        (*it)->allDescendants(lst);
    }
}
void RoboasmCoords::allDescendants(coordsPtrList &lst)
{
    //lst.push_back(this); // not including self
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        lst.push_back(*it);
        (*it)->allDescendants(lst);
    }
}
template <typename T>
void RoboasmCoords::allDescendants(coordsPtrList &lst)
{
    //lst.push_back(this); // not including self
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        std::shared_ptr<T> p = std::dynamic_pointer_cast<T> (*it);
        if (!!p) {
            lst.push_back(*it);
        }
        (*it)->allDescendants<T>(lst);
    }
}
template <typename T>
void RoboasmCoords::allDescendants(std::vector< std::shared_ptr < T > >&lst)
{
    //lst.push_back(this); // not including self
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        std::shared_ptr<T> p = std::dynamic_pointer_cast<T> (*it);
        if (!!p) {
            lst.push_back(p);
        }
        (*it)->allDescendants<T>(lst);
    }
}
// templateSettings
//template void RoboasmCoords::allDescendants<RoboasmCoords>(coordsPtrList &lst);
template void RoboasmCoords::allDescendants<RoboasmConnectingPoint>(coordsPtrList &lst);
template void RoboasmCoords::allDescendants<RoboasmParts>(coordsPtrList &lst);
//template void RoboasmCoords::allDescendants<RoboasmRobot>(coordsPtrList &lst);
template void RoboasmCoords::allDescendants<RoboasmConnectingPoint>(connectingPointPtrList &lst);
template void RoboasmCoords::allDescendants<RoboasmParts>(partsPtrList &lst);
void RoboasmCoords::searchCondition(coordsPtrList &lst,
                                    std::function<bool(RoboasmCoordsPtr _rc)> addlist,
                                    std::function<bool(RoboasmCoordsPtr _rc)> finish)
{
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        if(addlist(*it)) lst.push_back(*it);
        if(!!finish && finish(*it)) continue;
        (*it)->searchCondition(lst, addlist, finish);
    }
}
void RoboasmCoords::toNextLink(coordsPtrList &lst, bool parts)
{
    if (parts) {
        searchCondition(lst,
                        [](RoboasmCoordsPtr _rc) { return _rc->isParts(); },
                        [](RoboasmCoordsPtr _rc) { return (_rc->isConnectingPoint() && _rc->toConnectingPoint()->isActuator()); } );
    } else {
        searchCondition(lst,
                        [](RoboasmCoordsPtr _rc) { return true; },
                        [](RoboasmCoordsPtr _rc) { return (_rc->isConnectingPoint() && _rc->toConnectingPoint()->isActuator()); } );
    }
}
void RoboasmCoords::connectingPoints(connectingPointPtrList &activelst,
                                     connectingPointPtrList &inactivelst)
{
    connectingPointPtrList tmp;
    allDescendants<RoboasmConnectingPoint> (tmp);
    for(auto it = tmp.begin(); it != tmp.end(); it++) {
        if (!(*it)->hasDescendants()) {
            activelst.push_back(*it);
        } else {
            inactivelst.push_back(*it);
        }
    }
}
void RoboasmCoords::activeConnectingPoints(connectingPointPtrList &lst)
{
    connectingPointPtrList tmp;
    allDescendants<RoboasmConnectingPoint> (tmp);
    for(auto it = tmp.begin(); it != tmp.end(); it++) {
        if (!(*it)->hasDescendants()) {
            lst.push_back(*it);
        }
    }
}
void RoboasmCoords::inactiveConnectingPoints(connectingPointPtrList &lst)
{
    connectingPointPtrList tmp;
    allDescendants<RoboasmConnectingPoint> (tmp);
    for(auto it = tmp.begin(); it != tmp.end(); it++) {
        if ((*it)->hasDescendants()) {
            lst.push_back(*it);
        }
    }
}
void RoboasmCoords::actuators(connectingPointPtrList &lst)
{
    connectingPointPtrList tmp;
    allDescendants<RoboasmConnectingPoint> (tmp);
    for(auto it = tmp.begin(); it != tmp.end(); it++) {
        if ((*it)->isActuator()) {
            lst.push_back(*it);
        }
    }
}
void RoboasmCoords::actuators(connectingPointPtrList &activelst,
                              connectingPointPtrList &inactivelst)
{
    connectingPointPtrList tmp;
    allDescendants<RoboasmConnectingPoint> (tmp);
    for(auto it = tmp.begin(); it != tmp.end(); it++) {
        if ((*it)->isActuator()) {
            if(!(*it)->hasDescendants()) {
                activelst.push_back(*it);
            } else {
                inactivelst.push_back(*it);
            }
        }
    }
}
void RoboasmCoords::activeActuators(connectingPointPtrList &lst)
{
    connectingPointPtrList tmp;
    allDescendants<RoboasmConnectingPoint> (tmp);
    for(auto it = tmp.begin(); it != tmp.end(); it++) {
        if ((*it)->isActuator() &&
            !(*it)->hasDescendants()) {
            lst.push_back(*it);
        }
    }
}
void RoboasmCoords::inactiveActuators(connectingPointPtrList &lst)
{
    connectingPointPtrList tmp;
    allDescendants<RoboasmConnectingPoint> (tmp);
    for(auto it = tmp.begin(); it != tmp.end(); it++) {
        if ((*it)->isActuator() &&
            (*it)->hasDescendants()) {
            lst.push_back(*it);
        }
    }
}
RoboasmCoordsPtr RoboasmCoords::find(const std::string &name)
{
    RoboasmCoordsPtr ret;
    coordsPtrList lst;
    allDescendants (lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        if ((*it)->name() == name) {
            ret = *it;
            break;
        }
    }
    return ret;
}
template <typename T>
RoboasmCoordsPtr RoboasmCoords::find(const std::string &name)
{
    RoboasmCoordsPtr ret;
    coordsPtrList lst;
    allDescendants<T> (lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        if ((*it)->name() == name) {
            ret = *it;
            break;
        }
    }
    return ret;
}
// template settings
//template RoboasmCoords RoboasmCoords::find<RoboasmCoords>(const std::string &name); // use not template function
template RoboasmCoordsPtr RoboasmCoords::find<RoboasmParts>(const std::string &name);
template RoboasmCoordsPtr RoboasmCoords::find<RoboasmConnectingPoint>(const std::string &name);
//template RoboasmCoords RoboasmCoords::find<RoboasmRobot>(const std::string &name); //
// protected
#if 0
void RoboasmCoords::_replaceParent(RoboasmCoords *p)
{
    if (!!parent_ptr) {
        parent_ptr->dissoc(this);
        parent_ptr = p;
    } else {
        parent_ptr = p;
    }
}
#endif
bool RoboasmCoords::_existing_descendant(RoboasmCoordsPtr c)
{
    auto res = descendants.find(c);
    if (res != descendants.end()) {
        return true;
    }
    return false;
}
bool RoboasmCoords::_existing_descendant(RoboasmCoords *c)
{
    auto res = descendants.end();
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        if ((*it).get() == c) {
            res = it;
            break;
        }
    }
    if (res != descendants.end()) {
        return true;
    }
    return  false;
}
bool RoboasmCoords::_erase_descendant(RoboasmCoords *c)
{
    auto res = descendants.end();
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        if ((*it).get() == c) {
            res = it;
            break;
        }
    }
    if (res != descendants.end()) {
        descendants.erase(res);
        return true;
    }
    return false;
}

//// [roboasm connecting point] ////
RoboasmConnectingPoint::RoboasmConnectingPoint(const std::string &_name,
                                               ConnectingPoint *_info)
    : RoboasmCoords(_name), current_configuration_id(-1),
      current_configuration(nullptr), current_type_match(nullptr)
{
    info = _info;
}
RoboasmConnectingPoint::~RoboasmConnectingPoint()
{
    //DEBUG_STREAM(" [" << this << "] " << name_str);
}
bool RoboasmConnectingPoint::checkValidity()
{
    if(!parent_ptr) {
        ERROR_STREAM(" A connecting point should have a parent : " << *this);
        return false;
    }
    if(parent_ptr->isRobot()) {
        ERROR_STREAM(" A parent of a connecting point should not be a robot : " << *this);
        return false;
    }
    // parts -> con
    // parts -> con -> con -> parts
    coordsPtrList lst;
    directDescendants(lst);
    if(parent_ptr->isConnectingPoint()) {
        if (lst.size() != 1) {
            ERROR_STREAM(" Size of descendants should be 1, when a parent is a connecting point : " << lst.size());
            return false;
        }
        if( !(lst[0]->isParts()) ) {
            ERROR_STREAM(" A descendant should be a parts , when a parent is a connecting point : " << lst[0]);
            return false;
        }
    } else if(parent_ptr->isParts()) {
        if (lst.size() > 1) {
            ERROR_STREAM(" Size of descendants should not be more than 1, when a parent is a parts : " << lst.size());
            return false;
        } else if (lst.size() == 1) {
            if ( !(lst[0]->isConnectingPoint()) ) {
                ERROR_STREAM(" A descendant should be a connecting point , when a parent is a parts : " << lst[0]);
                return false;
            }
        }
    } else {
        ERROR_STREAM(" unkown parent's type : " << *this);
        return false;
    }
    return true;
}
bool RoboasmConnectingPoint::applyJointAngle(double angle)
{
    if (!isActuator()) return false;
    Actuator *ainfo_ = dynamic_cast<Actuator*>(info);
    //DEBUG_STREAM(" >> jname: " << name() << " <= " << angle);
    if(isInverted()) { // OK???
        angle = -angle;
    }
    if (ainfo_->getType() == ConnectingPoint::Rotational) {
        coordinates newcds (default_coords);
        //DEBUG_STREAM(" >> cds: " << default_coords);
        //DEBUG_STREAM(" >> ang: " << angle);
        //DEBUG_STREAM(" >>  ax: " << ainfo_->axis);
        newcds.rotate(angle, ainfo_->axis);
        //DEBUG_STREAM(" >> new: " << newcds);
        this->newcoords(newcds);
    } else if (ainfo_->getType() == ConnectingPoint::Linear) {
        Vector3 tr_(ainfo_->axis);
        tr_ *= angle;
        coordinates newcds (default_coords);
        newcds.translate(tr_);
        this->newcoords(newcds);
    }
    return true;
}
void RoboasmConnectingPoint::resetJointAngle()
{
    this->newcoords(default_coords);
}

//// [roboasm parts] ////
RoboasmParts::RoboasmParts(const std::string &_name, Parts *_info)
    : RoboasmCoords(_name), info(_info), color(Vector3f::Zero())
{
    createConnectingPoints();
}
RoboasmParts::~RoboasmParts()
{
    //DEBUG_STREAM(" [" << this << "] " << name_str);
}
bool RoboasmParts::checkValidity()
{
    if (!!parent_ptr) { // check parent
        if(!parent_ptr->isRobot() && !parent_ptr->isConnectingPoint()) {
            ERROR_STREAM(" A parent of a parts is a robot or a connecting-point / self: "  << *this << " / parent: " << *parent_ptr );
            return false;
        }
    }
    coordsPtrList lst;
    directDescendants(lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        if((*it)->isParts() || (*it)->isRobot()) {
            ERROR_STREAM(" A descendant of a parts is not a parts nor a robot / self: "  << *this << " / parent: " << (*it) );
            return false;
        }
    }
    return true;
}
bool RoboasmParts::dumpConnectionFromParent(AttachHistory &history)
{// rewrite using parentParts()
    //DEBUG_STREAM(" [" << history.size() << "] : " << name());
    AttachHistoryItem itm;
    if(!parent_ptr) return false;
    if(!parent_ptr->isRobot() &&
       !(!!parent_ptr->parent() && !!parent_ptr->parent()->parent()))
        return false;
    if(!info) return false;
    //DEBUG_STREAM(" [" << history.size() << "]  0");
    if(!parent_ptr->isRobot()) {
        itm.parts_name = name();
        itm.parts_type = info->type;
        itm.parts_point_url = parent_ptr->name(); // backward compatibility
        itm.parts_point = parent_ptr->point_name();
        itm.parent = parent_ptr->parent()->parent()->name();
        itm.parent_point_url = parent_ptr->parent()->name();  // backward compatibility
        itm.parent_point = parent_ptr->parent()->point_name();
        parent_ptr->parent()->worldcoords().transformation(
            itm.connecting_offset, parent_ptr->worldcoords());
    } else {
        itm.initial_parts = true;
        itm.parts_name = name();
        itm.parts_type = info->type;
    }
    history.push_back(itm);
    partsPtrList lst;
    childParts(lst);
    //DEBUG_STREAM(" " << lst.size());
    for(auto it = lst.begin(); it != lst.end(); it++) {
        if(!(*it)->dumpConnectionFromParent(history)) {
            return false;
        }
    }
    return true;
}
void RoboasmParts::childParts(partsPtrList &lst)
{
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        //DEBUG_STREAM(" : " << (*it)->name());
        RoboasmConnectingPointPtr ptr = std::dynamic_pointer_cast<RoboasmConnectingPoint>(*it);
        if(ptr->isConnected()) {
            coordsPtrList l0;
            ptr->directDescendants(l0);
            //DEBUG_STREAM(" 0 : " << l0.size());
            if(l0.size() > 0) {
                coordsPtrList l1;
                l0[0]->directDescendants(l1);
                //DEBUG_STREAM(" 10 : " << l1.size());
                if(l1.size() > 0) {
                    //DEBUG_STREAM(" 11 : " << (l1[0])->name());
                    RoboasmPartsPtr pp = std::dynamic_pointer_cast<RoboasmParts>(l1[0]);
                    if(!!pp) lst.push_back(pp);
                }
            }
        }
    }
}
bool RoboasmParts::parentParts(RoboasmPartsPtr &_res_parent,
                               RoboasmConnectingPointPtr &_res_parent_point,
                               RoboasmConnectingPointPtr &_res_self_point)
{
    if(!parent_ptr) return false;
    if(parent_ptr->isRobot()) return false;
    if(!parent_ptr->parent()) return false;
    if(!parent_ptr->parent()->parent()) return false;
    RoboasmParts *p_parts = parent_ptr->parent()->parent()->toParts();
    if(!p_parts) return false;
    RoboasmConnectingPoint *p_pt = parent_ptr->parent()->toConnectingPoint();
    if(!p_pt) return false;
    RoboasmConnectingPoint *self_pt = parent_ptr->toConnectingPoint();
    if(!self_pt) return false;
    RoboasmCoords *pp = parent_ptr->parent()->parent()->parent();
    if(!pp) return false;
    _res_parent = RoboasmUtil::toParts(pp->isDirectDescendant(p_parts));
    if(!_res_parent) return false;
    _res_parent_point = RoboasmUtil::toConnectingPoint(p_parts->isDirectDescendant(p_pt));
    if(!_res_parent_point) {
        _res_parent = nullptr;
        return false;
    }
    _res_self_point = RoboasmUtil::toConnectingPoint(p_pt->isDirectDescendant(self_pt));
    if(!_res_self_point) {
        _res_parent = nullptr;
        _res_parent_point = nullptr;
        return false;
    }
    return true;
}
void RoboasmParts::createConnectingPoints(bool use_name_as_namespace)
{
    std::string namespace_;
    if(use_name_as_namespace) {
        namespace_ = name();
    }
    createConnectingPoints(namespace_);
}
void RoboasmParts::createConnectingPoints(const std::string &_namespace)
{
    for(auto it = info->connecting_points.begin();
        it != info->connecting_points.end(); it++) {
        assocConnectingPoint(&(*it), _namespace);
    }
    for(auto it = info->actuators.begin();
        it != info->actuators.end(); it++) {
        assocConnectingPoint(&(*it), _namespace);
    }
    updateDescendants();
}
void RoboasmParts::assocConnectingPoint(ConnectingPoint* cp, const std::string &_namespace)
{
    std::string nm;
    if (_namespace.size() > 0) {
        nm = _namespace + "/" + cp->name;
    } else {
        nm = cp->name;
    }
    RoboasmConnectingPointPtr ptr = std::make_shared<RoboasmConnectingPoint> (nm, cp);
    ptr->newcoords(cp->coords);
    this->assoc(ptr);
    ptr->default_coords = *ptr;   // initialize only here
}

//// [roboasm robot] ////
RoboasmRobot::RoboasmRobot(const std::string &_name, RoboasmPartsPtr parts,
                           SettingsPtr _settings)
    : RoboasmCoords(_name), settings(_settings)
{
    assoc(parts);
    updateDescendants();
}
RoboasmRobot::~RoboasmRobot()
{
    //DEBUG_STREAM(" [" << this << "] " << name_str);
}
bool RoboasmRobot::reverseParentChild(RoboasmPartsPtr _parent, RoboasmConnectingPointPtr _chld)
{
    // check _chld is descendants of _parent
    if(!_parent->isDirectDescendant(_chld)) {
        // [todo]
        return false;
    }
    // check _chld has no descendants
    if(_chld->hasDescendants()) {
        // [todo]
        return false;
    }
    // check _parent has no parent
    if(_parent->hasParent()) {
        // [todo]
        return false;
    }
    _chld->dissocFromParent();
    _chld->assoc(_parent);
    return true;
}
bool RoboasmRobot::changeRoot(RoboasmConnectingPointPtr _chld)
{
    coordsPtrList lst;
    _chld->toRootList(lst); // it contains coords from self to root-parts
    DEBUG_STREAM(" lst.size() = " << lst.size());
    if(lst.size() < 1) {
        DEBUG_STREAM(" lst.size() = " << lst.size());
        return false;
    }
    for(auto it = lst.begin(); it != lst.end(); it++) {
        DEBUG_STREAM(" dissocFromParent " << (*it)->name());
        if ( !(*it)->dissocFromParent() ) {
            DEBUG_STREAM(" dissoc failed " << (*it)->name());
            return false;
        }
    }
    DEBUG_STREAM(" assoc:");
    size_t len_1 = lst.size() - 1;
    for(size_t i = 0; i < len_1; i++) {
        DEBUG_STREAM(" " << lst[i]->name() << " assoc: " << lst[i+1]->name());
        lst[i]->assoc(lst[i+1]);
        if(lst[i]->isConnectingPoint() && lst[i+1]->isConnectingPoint()) {
            DEBUG_STREAM(" swap configuration: " << lst[i]->name() << " <=> " << lst[i+1]->name());
            lst[i]->toConnectingPoint()->swapConfiguration(lst[i+1]->toConnectingPoint());
        }
    }
    return true;
}
bool RoboasmRobot::checkCorrectPoint(RoboasmCoordsPtr robot_or_parts,
                                     RoboasmConnectingPointPtr _parts_point,
                                     RoboasmConnectingPointPtr _robot_point)
{
    if(!!_parts_point && _parts_point->hasDescendants()) {
        // [todo]
        return false;
    }
    if(!!_robot_point && _robot_point->hasDescendants()) {
        // [todo]
        return false;
    }
    if(!robot_or_parts->isDescendant(_parts_point)) {
        // [todo]
        return false;
    }
    if(!isDescendant(_robot_point)) {
        // [todo]
        return false;
    }
    return true;
}
RoboasmRobotPtr RoboasmRobot::detach(RoboasmPartsPtr _pt)
{
    partsPtrList lst;
    allParts(lst);
    auto it = std::find(lst.begin(), lst.end(), _pt);
    if(it == lst.end()) return nullptr;
    if(_pt->parent() == this) return nullptr;

    // pt->parent (parts-point)
    // pt->parent->parent (parent-point)
    if( !(_pt->parent()) || !(_pt->parent()->parent()) ) return nullptr;

    RoboasmCoordsPtr pt_p = _pt->parent()->parent()->isDirectDescendant(_pt->parent());
    bool res = _pt->parent()->dissocFromParent();
    if(!res) return nullptr;

    res = _pt->dissocFromParent();
    if(!res) return nullptr;
    if(pt_p->hasDescendants()) return nullptr;

    _pt->assoc(pt_p);

    RoboasmRobotPtr ret = std::make_shared<RoboasmRobot>("divided_robot", _pt, settings);
    return ret;
}
bool RoboasmRobot::checkAttachByName(RoboasmCoordsPtr robot_or_parts,
                                     const std::string &name_parts_point,
                                     const std::string &name_robot_point,
                                     const std::string &name_config,
                                     RoboasmConnectingPointPtr &_res_parts_point,
                                     RoboasmConnectingPointPtr &_res_robot_point,
                                     ConnectingConfiguration * &_res_config,
                                     ConnectingTypeMatch * &_res_match)
{
    // search configuration
    ConnectingConfiguration *cc_ = settings->searchConnectingConfiguration(name_config);
    if (!cc_) {
        if ( name_config == "default") {
            cc_ = nullptr;
        } else {
            ERROR_STREAM(" no connecting point : " << name_config);
            return false;
        }
    }
    _res_config = cc_;
    _res_parts_point = std::dynamic_pointer_cast<RoboasmConnectingPoint> (
        robot_or_parts->find<RoboasmConnectingPoint>(name_parts_point));
    if(!_res_parts_point) {
        ERROR_STREAM(" can not find parts point : " << name_parts_point);
        return false;
    } else if(_res_parts_point->hasDescendants()) {
        ERROR_STREAM(" invalid parts point : " << name_parts_point);
        return false;
    }
    _res_robot_point = std::dynamic_pointer_cast<RoboasmConnectingPoint> (
        find<RoboasmConnectingPoint>(name_robot_point));
    if(!_res_robot_point) {
        ERROR_STREAM(" can not find robot point : " << name_robot_point);
        return false;
    } else if(_res_robot_point->hasDescendants()) {
        ERROR_STREAM(" invalid robot point : " << name_robot_point);
        return false;
    }
    return checkAttach(robot_or_parts, _res_parts_point, _res_robot_point,
                       _res_config, _res_match, false);
}
bool RoboasmRobot::checkAttach(RoboasmCoordsPtr robot_or_parts,
                               RoboasmConnectingPointPtr _parts_point,
                               RoboasmConnectingPointPtr _robot_point,
                               ConnectingConfiguration * &_config,
                               ConnectingTypeMatch * &_res_match, bool check)
{
    if (check && !checkCorrectPoint(robot_or_parts, _parts_point, _robot_point)) {
        // [todo]
        return false;
    }
    std::vector<ConnectingTypeID> &rtp = _robot_point->info->type_list;
    std::vector<ConnectingTypeID> &ptp = _parts_point->info->type_list;
    ConnectingTypeMatch *tm_ = nullptr;
    for(int i = 0; i < rtp.size(); i++) {
        for(int j = 0; j < ptp.size(); j++) {
            if( _config == nullptr ) {
                tm_ = settings->searchMatch(rtp[i], ptp[j]);
            } else {
                tm_ = settings->searchConnection(rtp[i], ptp[j], _config->index);
            }
            if (!!tm_) break;
        }
    }
    if (!tm_) {
        // [todo]
        return false;
    }
    _res_match = tm_;
    if( _config == nullptr ) {
        _config = &( settings->listConnectingConfiguration[tm_->allowed_configuration.front()] );
    }
    return true;
}
bool RoboasmRobot::searchMatch(RoboasmCoordsPtr robot_or_parts,
                               RoboasmConnectingPointPtr _parts_point, RoboasmConnectingPointPtr _robot_point,
                               std::vector<ConnectingTypeMatch*> &_res_match_lst, bool check)
{
    if (!check || !checkCorrectPoint(robot_or_parts, _parts_point, _robot_point)) {
        // [todo]
        return false;
    }
    std::vector<ConnectingTypeID> &rtp = _robot_point->info->type_list;
    std::vector<ConnectingTypeID> &ptp = _parts_point->info->type_list;
    _res_match_lst.clear();
    for(int i = 0; i < rtp.size(); i++) {
        for(int j = 0; j < ptp.size(); j++) {
            ConnectingTypeMatch *tm_ = settings->searchMatch(rtp[i], ptp[j]);
            if (!!tm_) _res_match_lst.push_back(tm_);
        }
    }
    if (_res_match_lst.size() < 1) {
        // [todo]
        return false;
    }
    return true;
}
bool RoboasmRobot::attach(RoboasmCoordsPtr robot_or_parts,
                          RoboasmConnectingPointPtr _parts_point,
                          RoboasmConnectingPointPtr _robot_point,
                          coordinates &_conf_coords, ConnectingConfiguration *_config,
                          ConnectingTypeMatch *_match, bool just_align)
{
    bool isrobot = false;
    RoboasmPartsPtr parts_;
    RoboasmRobotPtr robot_;
    {
        parts_ = std::dynamic_pointer_cast<RoboasmParts> (robot_or_parts);
        if (!parts_) {
            robot_ = std::dynamic_pointer_cast<RoboasmRobot> (robot_or_parts);
            if (!robot_) {
                std::cerr << "this is parts nor robot!" << std::endl;
                return false;
            } else {
                isrobot = true;
            }
        }
    }
    if (!!_config) {
        DEBUG_SIMPLE("config : " << _config->index);
        DEBUG_SIMPLE("config : " << _config->name);
        DEBUG_SIMPLE("config : " << _config->coords);
        _conf_coords = _config->coords;
    }
    coordinates r_point_w = _robot_point->worldcoords();
    //coordinates p_base_w  = _parts->worldcoords();
    //coordinates p_point_w = _parts_point->worldcoords();
    DEBUG_SIMPLE( "r_point_w(org): " << r_point_w);
    //DEBUG_SIMPLE( "p_base_w: " ;  print(p_base_w << std::endl;
    //DEBUG_SIMPLE( "p_point_w: " ;  print(p_point_w << std::endl;

    r_point_w.transform(_conf_coords); // new version | move parent point

    DEBUG_SIMPLE( "r_point_w(with_conf): " << r_point_w);
    coordinates p_base_to_point;
    if (_parts_point->parent() == robot_or_parts.get()) {
        // attach parts
        p_base_to_point = *static_cast<coordinates *>(_parts_point.get());
        DEBUG_SIMPLE( "p_base_to_point(parts): "  << p_base_to_point);
    } else {
        // attach robot
        robot_or_parts->worldcoords().transformation(p_base_to_point, _parts_point->worldcoords());
        DEBUG_SIMPLE( "p_base_to_point(robot): "  << p_base_to_point);
    }
    //p_base_to_point.transform(_conf_coords); // eus version | move parts point
    //p_base_w.transformation(p_base_to_point, p_point_w);
    p_base_to_point.inverse();
    DEBUG_SIMPLE( "(inv)p_base_to_point: "  << p_base_to_point);
    r_point_w.transform(p_base_to_point);
    DEBUG_SIMPLE( "r_point_w: " << r_point_w);

    robot_or_parts->newcoords(r_point_w);
    _parts_point->update();

    // _parts_point->worldcoords() == _robot_point->worldcoords()
    DEBUG_SIMPLE( "_robot_point->worldcoords(): " << _robot_point->worldcoords());
    DEBUG_SIMPLE( "_parts_point->worldcoords(): " << _parts_point->worldcoords());
    DEBUG_SIMPLE( "_parts->worldcoords(): " << robot_or_parts->worldcoords());

    if (just_align) return true;

    //DEBUG_SIMPLE( "reverse" );
    if(isrobot) {
        // [todo if Robot]
        //robot_
        RoboasmPartsPtr pp;
        coordsPtrList plst;
        robot_->directDescendants(plst);
        for(int i = 0; i < plst.size(); i++) {
            pp = std::dynamic_pointer_cast<RoboasmParts>(plst[i]);
            if(!!pp) break;
        }
        if(!pp) {
            // [todo]
            DEBUG_STREAM(" no parts in directDescendants of robot");
        }
        if(!changeRoot(_parts_point)) {
            DEBUG_STREAM(" changeRoot failed");
        }
        // _parts_point
    } else {
        reverseParentChild(parts_, _parts_point);
    }
    //DEBUG_SIMPLE( "assoc" );
    _robot_point->assoc(_parts_point);
    _parts_point->default_coords = *_parts_point; // update default_coords
    //DEBUG_SIMPLE( "update" );
    updateDescendants();

    // only _robot_point (connection-parent) has configuration
    if(!!_config) {
        _robot_point->current_configuration_id = _config->index;
        _robot_point->configuration_str = _config->name;
    } else {
        _robot_point->current_configuration_id = -1;
        // _robot_point->configuration_str =  // dump coords to string
    }
    _robot_point->configuration_coords = _conf_coords;
    _robot_point->current_type_match = _match;
    _robot_point->current_configuration = _config;

    return true;
}
bool RoboasmRobot::checkValidity()
{
    if (!!parent_ptr) {
        ERROR_STREAM(" A robot should not have p parent : " << *parent_ptr);
        return false;
    }
    RoboasmPartsPtr pp;
    {
        coordsPtrList lst;
        directDescendants(lst);
        if(lst.size() != 1) {
            ERROR_STREAM(" Size of robot's direct descendants is 1 : " << lst.size());
            return false;
        }
        pp = std::dynamic_pointer_cast<RoboasmParts>(lst[0]);
        if (!pp) {
            ERROR_STREAM(" Robot's direct descendant is only a parts : " << *(lst[0]));
            return false;
        }
    }
    {
        partsPtrList lst;
        allParts(lst);
        for(auto it = lst.begin(); it != lst.end(); it++) {
            if(!((*it)->checkValidity())) {
                ERROR_STREAM(" not valid(parts) : " << *(*it));
                return false;
            }
        }
    }
    {
        connectingPointPtrList lst;
        connectingPoints(lst);
        for(auto it = lst.begin(); it != lst.end(); it++) {
            if(!((*it)->checkValidity())) {
                ERROR_STREAM(" not valid(connecting point) : " << *(*it));
                return false;
            }
        }
    }
    return true;
}
bool RoboasmRobot::createRoboasm(RoboasmFile &_roboasm)
{
    RoboasmPartsPtr init_pt_ = rootParts();
    if(!init_pt_) return false;
    init_pt_->dumpConnectionFromParent(_roboasm.history);
    return writeConfig(_roboasm.config);
}
bool RoboasmRobot::writeConfig(AssembleConfig &_config)
{
    _config.robot_name = name();
    RoboasmPartsPtr root_ = rootParts();
    if(!root_) return false;
    _config.initial_coords = *root_;
    return true;
}
void RoboasmRobot::connectedPoints(connectingPointPtrList &lst)
{
    // extract con0, where parts0 -> con0 -> con1 -> parts1
    connectingPointPtrList tmp;
    connectingPoints(tmp);
    for(auto it = tmp.begin(); it != tmp.end(); it++) {
        if(!(*it)->hasParent()) continue; // not valid
        if((*it)->parent()->isParts() && (*it)->hasDescendants()) {
            lst.push_back(*it);
        }
    }
}
bool RoboasmRobot::applyJointAngle(const std::string &_nm, double angle)
{
    RoboasmCoordsPtr pt = this->find(_nm);
    if(pt->isActuator()) {
        pt->toConnectingPoint()->applyJointAngle(angle);
        return true;
    }
    return false;
}
//// [RoboasmUtil] ////
RoboasmUtil::RoboasmUtil(const std::string &filename)
{
    SettingsPtr p = std::make_shared<Settings>();
    if (p->parseYaml(filename)) {
        parts_counter = 0;
#ifndef _WIN32
        pid = getpid();
#else
        pid = _getpid();
#endif
        current_settings = p;
    } else {
        current_settings = nullptr;
    }
}
RoboasmUtil::RoboasmUtil(SettingsPtr settings)
{
    if(!!settings) {
        parts_counter = 0;
#ifndef _WIN32
        pid = getpid();
#else
        pid = _getpid();
#endif
    }
    current_settings = settings;
}
RoboasmUtil::~RoboasmUtil()
{
    //DEBUG_STREAM(" [" << this << "] " << name_str);
}
bool RoboasmUtil::isReady()
{
    return (!!current_settings);
}
RoboasmPartsPtr RoboasmUtil::makeParts(const std::string &_parts_key)
{
    std::ostringstream os;
    os << _parts_key << "_" << pid << "_" << parts_counter;
    parts_counter++;
    return makeParts(_parts_key, os.str());
}
RoboasmPartsPtr RoboasmUtil::makeParts
(const std::string &_parts_key, const std::string &_parts_name)
{
    auto res = current_settings->mapParts.find(_parts_key);
    if (res == current_settings->mapParts.end()) {
        //[todo]
        std::cerr << "key:" << _parts_key << " not found!" << std::endl;
        return nullptr;
    }
    RoboasmPartsPtr ret = std::make_shared<RoboasmParts> (_parts_name, &(res->second));
    return ret;
}
RoboasmRobotPtr RoboasmUtil::makeRobot(const std::string &_name, const std::string &_parts_key,
                                       const Vector3f &_color)
{
    return makeRobot(_name, _parts_key, std::string(), _color);
}
RoboasmRobotPtr RoboasmUtil::makeRobot(const std::string &_name, const std::string &_parts_key,
                                       const std::string &_parts_name, const Vector3f &_color)
{
    RoboasmPartsPtr pt_;
    if (_parts_name.size() > 0) {
        pt_ = makeParts(_parts_key, _parts_name);
    } else {
        pt_ = makeParts(_parts_key);
    }
    if ( !pt_ ) {
        return nullptr;
    }
    pt_->color = _color;
    return makeRobot(_name, pt_);
}
RoboasmRobotPtr RoboasmUtil::makeRobot(const std::string &_name, RoboasmPartsPtr _parts)
{
    return std::make_shared<RoboasmRobot>(_name, _parts, current_settings);
}

bool RoboasmUtil::canMatch(RoboasmConnectingPointPtr _a, RoboasmConnectingPointPtr _b)
{
    std::vector<ConnectingTypeID> &rtp = _a->info->type_list;
    std::vector<ConnectingTypeID> &ptp = _b->info->type_list;
    bool match = false;
    for(int i = 0; i < rtp.size(); i++) {
        for(int j = 0; j < ptp.size(); j++) {
            ConnectingTypeMatch *tm_ = current_settings->searchMatch(rtp[i], ptp[j]);
            if (!!tm_) {
                match = true;
                break;
            }
        }
    }
    return match;
}
RoboasmRobotPtr RoboasmUtil::makeRobot(const std::string &_name, AttachHistory &_history)
{
    RoboasmRobotPtr ret;
    if(_history.size() < 1) {
        return nullptr;
    }
    ret = makeRobot(_name,
                    _history[0].parts_type,
                    _history[0].parts_name);
    if(!ret) {
        return nullptr;
    }
    for(int i = 1; i < _history.size(); i++) {
        RoboasmPartsPtr parts_ = makeParts(_history[i].parts_type,
                                           _history[i].parts_name);
        if(!parts_) {
            DEBUG_STREAM(" parts make error, name: " << _history[i].parts_name
                         << "| type: " << _history[i].parts_type);
            return nullptr;
        }
        // parts_point_url, parent_point_url, config_name <=: old version
        // require (parts_name and parts_point) OR parts_point_url
        // require (parent and parent_point) OR parent_point_url
        // require connecting_offset OR config_name
        std::string parts_point_url_;
        std::string parent_point_url_;
        if (_history[i].parts_point_url.size() > 0) {
            parts_point_url_ = _history[i].parts_point_url;
        } else if (_history[i].parts_point.size() > 0 &&
                   _history[i].parts_name.size() > 0) {
            parts_point_url_ = _history[i].parts_name +
                               "/" + _history[i].parts_point;
        } else {
            ERROR_STREAM(" there is no valid parts_point_url");
            return nullptr;
        }
        if (_history[i].parent_point_url.size() > 0) {
            parent_point_url_ = _history[i].parent_point_url;
        } else if (_history[i].parent_point.size() > 0 &&
                   _history[i].parent.size() > 0) {
            parent_point_url_ = _history[i].parent +
                                "/" + _history[i].parent_point;
        } else {
            ERROR_STREAM(" there is no valid parent_point_url");
            return nullptr;
        }
        if (_history[i].config_name.size() > 0) {
            if (! ret->attach(parts_, parts_point_url_, parent_point_url_,
                              _history[i].config_name) ) {
                DEBUG_STREAM(" attach error, parts-point-url: " << parts_point_url_
                             << " | robot-point-url: " << parent_point_url_
                             << " | config-name: " << _history[i].config_name);
                return nullptr;
            }
        } else {
            if (! ret->attach(parts_, parts_point_url_, parent_point_url_,
                              _history[i].connecting_offset) ) {
                DEBUG_STREAM(" attach error, parts-point-url: " << parts_point_url_
                             << " | robot-point-url: " << parent_point_url_
                             << " | offset: " << _history[i].connecting_offset);
                return nullptr;
            }
        }
    }
    ret->updateDescendants();
    return ret;
}
RoboasmRobotPtr RoboasmUtil::makeRobot(RoboasmFile &_roboasm_file)
{
    RoboasmRobotPtr ret;
    std::string name_;
    if( _roboasm_file.config.robot_name.size() > 0) {
        name_ = _roboasm_file.config.robot_name;
    } else {
        name_ = "AssebleRobot";
    }
    ret = makeRobot(name_, _roboasm_file.history);
    if (!ret) {
        return nullptr;
    }
    // assemble-config
    if (!_roboasm_file.config.initial_coords.isInitial(1e-12)) {
        DEBUG_STREAM(" initial-coords : " << _roboasm_file.config.initial_coords);
        RoboasmPartsPtr root_ = ret->rootParts();
        if(!!root_) {
            DEBUG_STREAM(" set initial to root(" << root_->name() << ")");
            root_->newcoords(_roboasm_file.config.initial_coords);
        }
    }
    ret->updateDescendants();
    return ret;
}
static inline void _rename(std::string &_in, StringMap _rmap)
{
    auto it = _rmap.find(_in);
    if (it != _rmap.end()) {
        _in = it->second;
    }
}
bool RoboasmUtil::renamePartsHistory(AttachHistory &_hist, StringMap &_rmap)
{
    bool res_ = true;
    for(auto it = _hist.begin(); it != _hist.end(); it++) {
        // make new name
        std::ostringstream os;
        os << it->parts_type << "_" << pid << "_" << parts_counter;
        parts_counter++;
        _rmap.insert(std::make_pair(it->parts_name, os.str()));
        it->parts_name = os.str();
        // rename parent
        if(it->parent.size() > 0) {
            _rename(it->parent, _rmap);
        }
    }
    return res_;
}
std::ostream& cnoid::robot_assembler::operator<< (std::ostream& ostr, const cnoid::robot_assembler::RoboasmCoords &output)
{
    ostr << "[" << *((cnoid::coordinates *)&output) << "] " << output.name() << " : ";
    ostr << output.worldcoords();
    return ostr;
}
std::ostream& cnoid::operator<< (std::ostream& ostr, const cnoid::coordinates &cds)
{
    ostr << "((" << cds.pos(0) << " "
         << cds.pos(1) << " " << cds.pos(2);
    cnoid::Vector3 rpy; cds.getRPY(rpy);
    ostr << ") (" << rpy(0)  << " " << rpy(1)  << " "
         << rpy(2) << "))";
    return ostr;
}
