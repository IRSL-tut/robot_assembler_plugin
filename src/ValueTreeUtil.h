#ifndef CNOID_ROBOT_ASSEMBLER_PLUGIN_VTUTIL_H
#define CNOID_ROBOT_ASSEMBLER_PLUGIN_VTUTIL_H
#include <cnoid/ValueTree>
#include <cnoid/YAMLReader>
#include <cmath>

#include <sstream>

namespace cnoid {

inline bool parseFromString(std::vector<double> &_vec, const std::string &_input)
{
    YAMLReader yrdr;
    if (!yrdr.parse(_input)) return false;
    if(yrdr.numDocuments() < 1) return false;
    ValueNode *nd = yrdr.document(0);
    if(!nd->isValid() || !nd->isListing()) return false;
    Listing *lst = nd->toListing();
    if(lst->size() < 1) return false;
    for(int i = 0; i < lst->size(); i++) {
        _vec[i] = lst->at(i)->toDouble();
    }
    return true;
}
inline bool parseFromString(Vector3f &_vec, const std::string &_input)
{
    YAMLReader yrdr;
    if (!yrdr.parse(_input)) return false;
    if(yrdr.numDocuments() < 1) return false;
    ValueNode *nd = yrdr.document(0);
    if(!nd->isValid() || !nd->isListing()) return false;
    Listing *lst = nd->toListing();
    if(lst->size() < 3) return false;
    _vec(0) = lst->at(0)->toDouble();
    _vec(1) = lst->at(1)->toDouble();
    _vec(2) = lst->at(2)->toDouble();
    return true;
}
inline bool parseFromString(Vector3  &_vec, const std::string &_input)
{
    YAMLReader yrdr;
    if (!yrdr.parse(_input)) return false;
    if(yrdr.numDocuments() < 1) return false;
    ValueNode *nd = yrdr.document(0);
    if(!nd->isValid() || !nd->isListing()) return false;
    Listing *lst = nd->toListing();
    if(lst->size() < 3) return false;
    _vec(0) = lst->at(0)->toDouble();
    _vec(1) = lst->at(1)->toDouble();
    _vec(2) = lst->at(2)->toDouble();
    return true;
}
inline bool parseFromString(Vector3  &_vec, double &a, const std::string &_input)
{
    YAMLReader yrdr;
    if (!yrdr.parse(_input)) return false;
    if(yrdr.numDocuments() < 1) return false;
    ValueNode *nd = yrdr.document(0);
    if(!nd->isValid() || !nd->isListing()) return false;
    Listing *lst = nd->toListing();
    if(lst->size() < 4) return false;
    _vec(0) = lst->at(0)->toDouble();
    _vec(1) = lst->at(1)->toDouble();
    _vec(2) = lst->at(2)->toDouble();
    a       = lst->at(3)->toDouble();
    return true;
}
inline bool parseFromString(double &a, double &b, const std::string &_input)
{
    YAMLReader yrdr;
    if (!yrdr.parse(_input)) return false;
    if(yrdr.numDocuments() < 1) return false;
    ValueNode *nd = yrdr.document(0);
    if(!nd->isValid() || !nd->isListing()) return false;
    Listing *lst = nd->toListing();
    if(lst->size() < 2) return false;
    a = lst->at(0)->toDouble();
    b = lst->at(1)->toDouble();
    return true;
}
inline bool parseFromString(double &_flt, const std::string &_input)
{
    std::istringstream iss(_input);
    iss >> _flt;
    return true;
}
inline bool parseFromString(coordinates &_cds, const std::string &_input_trans,
                            const std::string &_input_rot)
{
    bool c_pos = false;
    bool c_rot = false;
    Vector3 pos;
    if((c_pos = parseFromString(pos, _input_trans))) {
        _cds.set(pos);
    }
    Vector3 ax_; double ag_;
    if((c_rot = parseFromString(ax_, ag_, _input_rot))) {
        AngleAxis aa(ag_, ax_);
        _cds.set(aa);
    }
    if(!c_pos && !c_rot) return false;
    return true;
}

inline void addToMapping(Mapping *_mp, const std::string &_key, ValueNode *_tgt)
{
    ValueNode *vn = _mp->find(_key);
    if(vn->isValid()) {
        _mp->remove(_key);
    }
    _mp->insert(_key, _tgt);
}
inline void addToMapping(Mapping *_mp, const std::string &_key, std::vector<double> &_vec, double eps = 1e-12)
{
    ValueNode *vn = _mp->find(_key);
    if(vn->isValid()) {
        _mp->remove(_key);
    }
    ListingPtr lst = new Listing();
    lst->setFlowStyle();
    lst->setFloatingNumberFormat("%12.12f");
    for(int i = 0; i < _vec.size(); i++) {
        if(std::fabs(_vec[i]) < eps) {
            lst->append(0.0);
        } else {
            lst->append(_vec[i]);
        }
    }
    _mp->insert(_key, lst);
}
inline void addToMapping(Mapping *_mp, const std::string &_key, double a, const Vector3 &_vec, double eps = 1e-12)
{
    ValueNode *vn = _mp->find(_key);
    if(vn->isValid()) {
        _mp->remove(_key);
    }
    ListingPtr lst = new Listing();
    lst->setFlowStyle();
    lst->setFloatingNumberFormat("%12.12f");
    if(std::fabs(_vec(0)) < eps) {
        lst->append(0.0);
    } else {
        lst->append(_vec(0));
    }
    if(std::fabs(_vec(1)) < eps) {
        lst->append(0.0);
    } else {
        lst->append(_vec(1));
    }
    if(std::fabs(_vec(2)) < eps) {
        lst->append(0.0);
    } else {
        lst->append(_vec(2));
    }
    if(std::fabs(a) < eps) {
        lst->append(0.0);
    } else {
        lst->append(a);
    }
    _mp->insert(_key, lst);
}
inline void addToMapping(Mapping *_mp, const std::string &_key, const Vector3 &_vec, double eps = 1e-12)
{
    ValueNode *vn = _mp->find(_key);
    if(vn->isValid()) {
        _mp->remove(_key);
    }
    ListingPtr lst = new Listing();
    lst->setFlowStyle();
    lst->setFloatingNumberFormat("%12.12f");
    if(std::fabs(_vec(0)) < eps) {
        lst->append(0.0);
    } else {
        lst->append(_vec(0));
    }
    if(std::fabs(_vec(1)) < eps) {
        lst->append(0.0);
    } else {
        lst->append(_vec(1));
    }
    if(std::fabs(_vec(2)) < eps) {
        lst->append(0.0);
    } else {
        lst->append(_vec(2));
    }
    _mp->insert(_key, lst);
}
inline void addToMapping(Mapping *_mp, const std::string &_key, const Vector3f &_vec, double eps = 1e-12)
{
    ValueNode *vn = _mp->find(_key);
    if(vn->isValid()) {
        _mp->remove(_key);
    }
    ListingPtr lst = new Listing();
    lst->setFlowStyle();
    lst->setFloatingNumberFormat("%7.7f");
    if(std::fabs(_vec(0)) < eps) {
        lst->append(0.0);
    } else {
        lst->append(_vec(0));
    }
    if(std::fabs(_vec(1)) < eps) {
        lst->append(0.0);
    } else {
        lst->append(_vec(1));
    }
    if(std::fabs(_vec(2)) < eps) {
        lst->append(0.0);
    } else {
        lst->append(_vec(2));
    }
    _mp->insert(_key, lst);
}
inline void addToMapping(Mapping *_mp, const std::string &_key, double a, double b, double eps = 1e-12)
{
    ValueNode *vn = _mp->find(_key);
    if(vn->isValid()) {
        _mp->remove(_key);
    }
    ListingPtr lst = new Listing();
    lst->setFlowStyle();
    lst->setFloatingNumberFormat("%12.12f");
    if(std::fabs(a) < eps) {
        lst->append(0.0);
    } else {
        lst->append(a);
    }
    if(std::fabs(b) < eps) {
        lst->append(0.0);
    } else {
        lst->append(b);
    }
    _mp->insert(_key, lst);
}
inline void addToMapping(Mapping *_mp, const std::string &_key, double _flt)
{
    ValueNode *vn = _mp->find(_key);
    if(vn->isValid()) {
        _mp->remove(_key);
    }
    _mp->setFloatingNumberFormat("%12.12f");//
    _mp->write(_key, _flt);
}
inline void addToMapping(Mapping *_mp, const std::string &_key, const std::string &_str)
{
    ValueNode *vn = _mp->find(_key);
    if(vn->isValid()) {
        _mp->remove(_key);
    }
    _mp->write(_key, _str);
}
inline void addToMapping(Mapping *_mp, const std::string &_key, bool _bl)
{
    ValueNode *vn = _mp->find(_key);
    if(vn->isValid()) {
        _mp->remove(_key);
    }
    _mp->write(_key, _bl);
}
inline void addCoordsToMapping(Mapping *_mp, const std::string &_key, const coordinates &_cds)
{
    if(_cds.isInitial()) return;
    ValueNode *vn = _mp->find(_key);
    if(vn->isValid()) {
        _mp->remove(_key);
    }
    MappingPtr mm_ = new Mapping();
    if(!_cds.pos.isZero()) {
        addToMapping(mm_, "translation", _cds.pos);
    }
    AngleAxis aa_(_cds.rot);
    if(aa_.angle() != 0) {
        addToMapping(mm_, "rotation", aa_.angle(), aa_.axis());
    }
    addToMapping(_mp, _key, mm_);
}
inline bool readFromMapping(Mapping *_mp, const std::string &_key, std::vector<double> &_vec)
{
    ValueNode *vn = _mp->find(_key);
    if (!vn->isValid() || !vn->isListing()) {
        return false;
    }
    Listing *lst = vn->toListing();
    if (lst->size() == 0) return false;
    for(int i = 0; i < lst->size(); i ++) {
        _vec[i] = lst->at(i)->toDouble();
    }
    return true;
}
inline bool readFromMapping(Mapping *_mp, const std::string &_key, Vector3 &_vec, double &a)
{
    ValueNode *vn = _mp->find(_key);
    if (!vn->isValid() || !vn->isListing()) {
        return false;
    }
    Listing *lst = vn->toListing();
    if(lst->size() < 4) return false;
    _vec(0) = lst->at(0)->toDouble();
    _vec(1) = lst->at(1)->toDouble();
    _vec(2) = lst->at(2)->toDouble();
    a       = lst->at(3)->toDouble();
    return true;
}
inline bool readFromMapping(Mapping *_mp, const std::string &_key, Vector3 &_vec)
{
    ValueNode *vn = _mp->find(_key);
    if (!vn->isValid() || !vn->isListing()) {
        return false;
    }
    Listing *lst = vn->toListing();
    if(lst->size() < 3) return false;
    _vec(0) = lst->at(0)->toDouble();
    _vec(1) = lst->at(1)->toDouble();
    _vec(2) = lst->at(2)->toDouble();
    return true;
}
inline bool readFromMapping(Mapping *_mp, const std::string &_key, Vector3f &_vec)
{
    ValueNode *vn = _mp->find(_key);
    if (!vn->isValid() || !vn->isListing()) {
        return false;
    }
    Listing *lst = vn->toListing();
    if(lst->size() < 3) return false;
    _vec(0) = lst->at(0)->toDouble();
    _vec(1) = lst->at(1)->toDouble();
    _vec(2) = lst->at(2)->toDouble();
    return true;
}
inline bool readFromMapping(Mapping *_mp, const std::string &_key, double &a, double &b)
{
    ValueNode *vn = _mp->find(_key);
    if (!vn->isValid() || !vn->isListing()) {
        return false;
    }
    Listing *lst = vn->toListing();
    if(lst->size() < 2) return false;
    a = lst->at(0)->toDouble();
    b = lst->at(1)->toDouble();
    return true;
}
inline bool readFromMapping(Mapping *_mp, const std::string &_key, double &_flt)
{
    ValueNode *vn = _mp->find(_key);
    if (!vn->isValid() || !vn->isScalar()) {
        return false;
    }
    _flt = vn->toDouble();
    return true;
}
inline bool readFromMapping(Mapping *_mp, const std::string &_key, std::string &_str)
{
    ValueNode *vn = _mp->find(_key);
    if (!vn->isValid() || !vn->isScalar()) {
        return false;
    }
    _str = vn->toString();
    return true;
}
inline bool readFromMapping(Mapping *_mp, const std::string &_key, coordinates &_cds)
{
    ValueNode *vn = _mp->find(_key);
    if (!vn->isValid() || !vn->isMapping()) {
        return false;
    }
    Mapping *_mm = vn->toMapping();
    Vector3 pos(Vector3::Zero());
    Vector3 ax(Vector3::Zero());
    double ang = 0.0;
    bool trs = false;
    bool rot = false;
    if((trs = readFromMapping(_mm, "translation", pos))) {
        _cds.pos = pos;
    }
    if((rot = readFromMapping(_mm, "rotation", ax, ang))) {
        AngleAxis aa(ang, ax);
        _cds.set(aa);
    }
    if(!trs && !rot) return false;
    return true;
}
};
#endif
