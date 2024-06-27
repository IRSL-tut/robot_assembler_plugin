#include "RobotAssemblerSettings.h"
#include <cnoid/YAMLReader> //
#include <cnoid/YAMLWriter> //
#include <iostream>
#include <cmath>
#include <algorithm> // transform
#include <sstream>

//#include <cnoid/UTF8>
//#include <cnoid/stdx/filesystem>

//#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;
using namespace cnoid::robot_assembler;

namespace {
class UnitConfig
{
public:
    enum Unit {
        None,
        radian,
        degree,
        m,
        mm,
        g,
        kg,
    };
public:
    UnitConfig() : angle_scale(1.0), length_scale(1.0), mass_scale(1.0), inertia_scale(1.0) { }

    void setScale(Unit angle, Unit length, Unit mass)
    {
        if (angle == degree) {
            // degree to radian
            angle_scale = M_PI/180.0;
        }
        if (length == mm) {
            // mm to m
            length_scale = 0.001;
        }
        if (mass == g) {
            // g to kg
            mass_scale = 0.001;
        }
        inertia_scale = mass_scale * length_scale * length_scale;
    }

    // angle in config to [rad]
    double convAngle(double config_angle) {
        return (config_angle * angle_scale);
    }
    // length in config to [m]
    double convLength(double config_length) {
        return (config_length * length_scale);
    }
    // mass in config to [kg]
    double convMass(double config_mass) {
        return (config_mass * mass_scale);
    }
    // inertia in config to [kg*m^2]
    double convInertia(double config_inertia) {
        return (config_inertia * inertia_scale);
    }
    // length vector3 -> vector3
    void convPoint(const Vector3 &in, Vector3 &out) {
        out = length_scale * in;
    }
    void convPoint(const std::vector<double> &in, Vector3 &out) {
        Vector3 vec(in[0], in[1], in[2]);
        convPoint(vec, out);
    }
    // length vector
    void convLengthVector(const std::vector<double> &in, std::vector<double>  &out) {
        out.clear();
        out.resize(in.size());
        for(int i = 0; i < in.size(); i++) out[i] = length_scale * in[i];
    }
    // vector4 -> angleAxis
    void convRotationAngle(const Vector4 &in, AngleAxis &out) {
        Vector3 ax_(in(0), in(1), in(2));
        double an_ = convAngle(in(3));
        AngleAxis aa_(an_, ax_);
        out = aa_;
    }
    void convRotationAngle(const std::vector<double> &in, AngleAxis &out) {
        Vector4 vec(in[0], in[1], in[2], in[3]);
        convRotationAngle(vec, out);
    }
    // Matrix3 -> Matrix3
    void convInertiaTensor(const Matrix3 &in, Matrix3 &out) {
        out = inertia_scale * in;
    }
    void convInertiaTensor(const std::vector<double> in, Matrix3 &out) {
        Matrix3 mat = Matrix3::Zero();
        if (in.size() >= 9) {
            mat(0, 0) = in[0]; mat(0, 1) = in[1]; mat(0, 2) = in[2];
            mat(1, 0) = in[3]; mat(1, 1) = in[4]; mat(1, 2) = in[5];
            mat(2, 0) = in[6]; mat(2, 1) = in[7]; mat(2, 2) = in[8];
        } else if (in.size() >= 6) {
            // xx, xy, yz, yy, yz, zz
            mat(0, 0) = in[0]; mat(0, 1) = in[1]; mat(0, 2) = in[2];
            mat(1, 0) = in[1]; mat(1, 1) = in[3]; mat(1, 2) = in[4];
            mat(2, 0) = in[2]; mat(2, 1) = in[4]; mat(2, 2) = in[5];
        }
        convInertiaTensor(mat, out);
    }
private:
    double angle_scale;
    double length_scale;
    double mass_scale;
    double inertia_scale;
};

void printNode(std::ostream &out, ValueNode *vn, int indent = 0)
{
    if (vn->isScalar()) {
        for(int i = 0; i < indent; i++) out << "  ";
        out << vn->toString() << std::endl;
    } else if (vn->isListing()) {
        Listing *lst = vn->toListing();
        for(int i = 0; i < indent; i++) out << "  ";
        out << "[" << std::endl;
        for(int i = 0; i < lst->size(); i++) {
            printNode(out, lst->at(i), indent + 1);
        }
        for(int i = 0; i < indent; i++) out << "  ";
        out << "]" << std::endl;
    } else if (vn->isMapping()) {
        for(int i = 0; i < indent; i++) out << "  ";
        out << "{" << std::endl;
        Mapping *mp = vn->toMapping();
        for(auto it = mp->begin(); it != mp->end(); it++) {
            for(int i = 0; i < indent; i++) out << "  ";
            out << it->first << ":" << std::endl;
            printNode(out, it->second, indent + 1);
        }
        for(int i = 0; i < indent; i++) out << "  ";
        out << "}" << std::endl;
    } else {
        out << "type error???" << std::endl;
    }
}
bool readString(ValueNode *val, std::string &str, std::ostream &out = std::cerr, bool require = true)
{
    if(!val->isScalar()) {
        out << "<<scalar expected but:" << std::endl;
        printNode(out, val);
        out << ">>" << std::endl;
        if (!require) return true;
        return false;
    }
    str = val->toString();
    return true;
}
bool readDouble(ValueNode *val, double &dbl, std::ostream &out = std::cerr, bool require = true)
{
    if(!val->isScalar()) {
        out << "<<scalar expected but:" << std::endl;
        printNode(out, val);
        out << ">>" << std::endl;
        if (!require) return true;
        return false;
    }
    dbl = val->toDouble();
    return true;
}
bool readVector(ValueNode *val, std::vector<double> &vec, std::ostream &out = std::cerr, bool require = true)
{
    if( ! val->isListing() ) {
        out << "<<list expected but:" << std::endl;
        printNode(out, val);
        out << ">>" << std::endl;
        if (!require) return true;
        return false;
    }
    Listing *lst = val->toListing();
    for(int i = 0; i < lst->size(); i++) {
        if(lst->at(i)->isScalar()) {
            vec.push_back(lst->at(i)->toDouble());
        }
    }
    return true;
}
bool mapString(Mapping *mp, const std::string &key, std::string &str, std::ostream &out = std::cerr, bool require = true)
{
    ValueNode *val = mp->find(key);
    if( ! val->isValid() ) {
        if (!require) return true;
        out << "<< key " << key << " not found:" << std::endl;
        printNode(out, mp);
        out << ">>" << std::endl;
        return false;
    }
    return readString(val, str, out, require);
}
bool mapDouble(Mapping *mp, const std::string &key, double &dbl, std::ostream &out = std::cerr, bool require = true)
{
    ValueNode *val = mp->find(key);
    if( ! val->isValid() ) {
        if (!require) return true;
        out << "<< key " << key << " not found:" << std::endl;
        printNode(out, mp);
        out << ">>" << std::endl;
        return false;
    }
    return readDouble(val, dbl, out, require);
}
bool mapVector(Mapping *mp, const std::string &key, std::vector<double> &vec,
               std::ostream &out = std::cerr, bool require = true)
{
    ValueNode *val = mp->find(key);
    if( ! val->isValid() ) {
        if (!require) return true;
        out << "<< key " << key << " not found:" << std::endl;
        printNode(out, mp);
        out << ">>" << std::endl;
        return false;
    }
    return readVector(val, vec, out, require);
}
};



class Settings::Impl
{
public:
    Settings *self;

    ::UnitConfig uc;

    Impl(Settings *self_);

    bool parseYaml(const std::string &filename);
    bool parseGeneralSettings(ValueNode *vn);
    bool parseConnectingConstraintSettings(ValueNode *vn);
    bool parsePartsSettings(ValueNode *vn);
    //
    bool parseConnectingConf(ValueNode *vn, ConnectingConfiguration &out);
    bool parseConnectingTypeMatch(ValueNode *vn, ConnectingTypeMatch &out);

    bool parseParts(ValueNode *vn, Parts &out);
    bool parseGeometry(ValueNode *vn, Geometry &geom);
    bool parseConnectingPoint(ValueNode *vn, ConnectingPoint &cpt);
    bool parseActuator(ValueNode *vn, Actuator &act);
    bool parseExtraInfo(ValueNode *vn, ExtraInfo &einfo);
    //
    void parseParts(YAMLReader &reader, std::vector<Parts> &results);
    void parseParts(ValueNode *val, std::vector<Parts> &results);
    //
    bool parseCoords(Mapping *map_, coordinates &cds);

    // reverse map (name -> index)
    std::map<std::string, long> reverseTypeNames; // listConnectingType
    std::map<std::string, long> reverseConfNames; // listConnectingConfiguration

    //config _path
};
////
Settings::Settings()
{
    impl = new Impl(this);
}
Settings::~Settings()
{
    delete impl;
}
Settings::Impl::Impl(Settings *self_)
{
    self = self_;
}
////
bool Settings::parseYaml(const std::string &filename)
{
    return impl->parseYaml(filename);
}
ConnectingTypeMatch *Settings::searchMatch(ConnectingTypeID _a, ConnectingTypeID _b)
{
    //brute force search
    for(int i = 0; i < listConnectingTypeMatch.size(); i++) {
        ConnectingTypeMatch &mt = listConnectingTypeMatch[i];
        if((mt.pair[0] == _a && mt.pair[1] == _b) ||
           (mt.pair[0] == _b && mt.pair[1] == _a)) {
            return &mt;
        }
    }
    return nullptr;
}
ConnectingTypeMatch *Settings::searchConnection
(ConnectingTypeID _a, ConnectingTypeID _b, ConnectingConfigurationID _tp)
{
    ConnectingTypeMatch *mt_ = searchMatch(_a, _b);
    if (!mt_) { // no match
        return nullptr;
    }
    std::vector<ConnectingConfigurationID> &ac_ = mt_->allowed_configuration;
    for(int i = 0; i < ac_.size(); i++) {
        if (ac_[i] == _tp) {
            return mt_;
        }
    }
    // configuration is not in searched match
    return nullptr;
}
ConnectingType *Settings::searchConnectingType(const std::string &_name)
{
    auto it = impl->reverseTypeNames.find(_name);
    if(it != impl->reverseTypeNames.end()) {
        long index = it->second;
        return &(listConnectingType[index]);
    }
    return nullptr;
}
ConnectingConfiguration *Settings::searchConnectingConfiguration(const std::string &_name)
{
    auto it = impl->reverseConfNames.find(_name);
    if(it != impl->reverseConfNames.end()) {
        long index = it->second;
        return &(listConnectingConfiguration[index]);
    }
    return nullptr;
}
bool Settings::validateParts(const Parts &pt)
{
    auto pt_ = mapParts.find(pt.type);
    if (pt_ != mapParts.end()) {
        ERROR_STREAM(" parseParts error: same type: " << pt.type);
        return false;
    }
    // check : There are connecting points or actuators with the same name
    {
        std::vector<const std::string *> vec;
        for(auto it = pt.connecting_points.cbegin();
            it != pt.connecting_points.cend(); it++) {
            vec.push_back(&(it->name));
        }
        for(auto it = pt.actuators.cbegin();
            it != pt.actuators.cend(); it++) {
            vec.push_back(&(it->name));
        }
        //
        int len = vec.size();
        for(int i = 0; i < len - 1; i++) {
            for(int j = i + 1; j < len; j++) {
                if ( *(vec[i]) == *(vec[j]) ) {
                    ERROR_STREAM(" parseParts error: in type: " << pt.type << " / there are connecting-points with the same name [" << *(vec[i]) << "]");
                    return false;
                }
            }
        }
    }
    return true;
}
bool Settings::parsePartsFromString(const std::string &settings, std::vector<Parts> &results)
{
    YAMLReader yaml_reader;
    if (! yaml_reader.parse(settings)) {
        ERROR_STREAM(" parts string Parsing error : " << settings);
        return false;
    }
    impl->parseParts(yaml_reader, results);
    return true;
}
bool Settings::parsePartsFromYaml(const std::string &filename, std::vector<Parts> &results)
{
    YAMLReader yaml_reader;
    if (! yaml_reader.load(filename)) {
        ERROR_STREAM(" parts file Loading error : " << filename);
        return false;
    }
    impl->parseParts(yaml_reader, results);
    return true;
}
bool Settings::parsePartsFromNode(ValueNode *val, std::vector<Parts> &results)
{
    impl->parseParts(val, results);
    return true;
}
bool Settings::insertParts(const Parts &pt)
{
    mapParts.insert(std::make_pair(pt.type, pt));
    return true;
}
bool Settings::insertPartsFromString(const std::string &settings)
{
    std::vector<Parts> lst;
    bool res;
    res = parsePartsFromString(settings, lst);
    if (res && lst.size() > 0) {
        for(auto it = lst.begin(); it != lst.end(); it++) {
            if (validateParts(*it)) {
                insertParts(*it);
            } else {
                res = false;
            }
        }
    }
    return res;
}
bool Settings::insertPartsFromYaml(const std::string &filename)
{
    std::vector<Parts> lst;
    bool res;
    res = parsePartsFromYaml(filename, lst);
    if (res && lst.size() > 0) {
        for(auto it = lst.begin(); it != lst.end(); it++) {
            if (validateParts(*it)) {
                insertParts(*it);
            } else {
                res = false;
            }
        }
    }
    return res;
}
bool Settings::insertPartsFromNode(ValueNode *val)
{
    std::vector<Parts> lst;
    bool res;
    res = parsePartsFromNode(val, lst);
    if (res && lst.size() > 0) {
        for(auto it = lst.begin(); it != lst.end(); it++) {
            if (validateParts(*it)) {
                insertParts(*it);
            } else {
                res = false;
            }
        }
    }
    return res;
}
//// [settings impl]
bool Settings::Impl::parseYaml(const std::string &filename)
{
    YAMLReader yaml_reader;
    if (! yaml_reader.load(filename)) {
        ERROR_STREAM(" File Loading error : " << filename);
        return false;
    }
    //[TODO] config_path
    bool ret = false;
    for(int i = 0; i < yaml_reader.numDocuments(); i++) {
        ValueNode *val = yaml_reader.document(i);
        if ( val->isMapping() ) {
            std::string key = "GeneralSettings";
            ValueNode *target = val->toMapping()->find(key);
            if(target->isValid()) {
                DEBUG_STREAM("--- target " << key << " found ---");
                ret = parseGeneralSettings(target);
                break;
            }
        }
    }
    if (!ret) {
        ERROR_STREAM(" failed parse settings");
        return false;
    }
    //
    ret = false;
    for(int i = 0; i < yaml_reader.numDocuments(); i++) {
        ValueNode *val = yaml_reader.document(i);
        if ( val->isMapping() ) {
            std::string key = "ConnectingConstraintSettings";
            ValueNode *target = val->toMapping()->find(key);
            if(target->isValid()) {
                DEBUG_STREAM("--- target " << key << " found ---");
                ret = parseConnectingConstraintSettings(target);
                break;
            }
        }
    }
    if (!ret) {
        ERROR_STREAM(" failed parse constraint");
        return false;
    }
    //
    ret = false;
    for(int i = 0; i < yaml_reader.numDocuments(); i++) {
        ValueNode *val = yaml_reader.document(i);
        if ( val->isMapping() ) {
            std::string key = "PartsSettings";
            ValueNode *target = val->toMapping()->find(key);
            if(target->isValid()) {
                DEBUG_STREAM("--- target " << key << " found ---");
                ret = parsePartsSettings(target);
                break;
            }
        }
    }
    if (!ret) {
        ERROR_STREAM(" failed parse parts");
        return false;
    }

    return true;
}
bool Settings::Impl::parseGeneralSettings(ValueNode *vn)
{
    if ( !vn->isMapping() ) {
        //
        return false;
    }

    ::UnitConfig::Unit _a_unit = ::UnitConfig::None;
    ::UnitConfig::Unit _l_unit = ::UnitConfig::None;
    ::UnitConfig::Unit _m_unit = ::UnitConfig::None;
    { // angle
    ValueNode *target = vn->toMapping()->find("angleUnit");
    if(target->isValid() && target->isString()) {
        std::string str = target->toString();
        DEBUG_STREAM("[config] angle: " << str);
        if (str == "deg" || str == "degree" || str == "DEG" || str == "DEGREE" ) {
            _a_unit = ::UnitConfig::degree;
        }
    } }

    { // length
    ValueNode *target = vn->toMapping()->find("lengthUnit");
    if(target->isValid() && target->isString()) {
        std::string str = target->toString();
        DEBUG_STREAM("[config] length: " << str);
        if (str == "mm" || str == "MM" || str == "millimeter" || str == "Millimeter" ) {
            _l_unit = ::UnitConfig::mm;
        }
    } }

    { // mass
    ValueNode *target = vn->toMapping()->find("massUnit");
    if(target->isValid() && target->isString()) {
        std::string str = target->toString();
        DEBUG_STREAM("[config] mass: " << str);
        if (str == "g" || str == "gram" ) {
            _m_unit = ::UnitConfig::g;
        }
    } }
    uc.setScale(_a_unit, _l_unit, _m_unit);
    return true;
}
bool Settings::Impl::parseConnectingConstraintSettings(ValueNode *vn)
{
    if ( !vn->isMapping() ) {
        // [todo]
        return false;
    }
    Mapping *val = vn->toMapping();
    {
        ValueNode *target = val->find("connecting-type-list");
        if(target->isValid() && target->isListing()) {
            Listing *lst = target->toListing();
            bool strlist = true;
            for(int i = 0; i < lst->size(); i++) {
                if (!lst->at(i)->isScalar()) {
                    strlist = false;
                    break;
                }
                std::string str = lst->at(i)->toString();
                ConnectingType tp_;
                tp_.name = str;
                tp_.index = i;
                self->listConnectingType.push_back(tp_);
                reverseTypeNames.insert(std::make_pair(str, i));
            }
            if (!strlist) {
                // [todo] invalid type
            }
        } else {
            // [todo] error ?
        }
    }
    {
        ValueNode *target = val->find("connecting-configuration-list");
        if(target->isValid() && target->isListing()) {
            Listing *lst = target->toListing();
            bool maplist = true;
            DEBUG_STREAM(" conf-list / size: " << lst->size());
            for(int i = 0; i < lst->size(); i++) {
                ConnectingConfiguration _conf;
                if (! parseConnectingConf(lst->at(i), _conf)) {
                    maplist = false;
                    continue;
                }
                _conf.index = i;
                self->listConnectingConfiguration.push_back(_conf);
                reverseConfNames.insert(std::make_pair(_conf.name, i));
            }
            if (!maplist) {
                ERROR_STREAM(" invalid connecting-configuration-list");
            }
        } else {
            ERROR_STREAM(" error connecting-configuration-list");
            return false;
        }
    }
    {
        ValueNode *target = val->find("connecting-type-match-list");
        if(target->isValid() && target->isListing()) {
            Listing *lst = target->toListing();
            bool maplist = true;
            DEBUG_STREAM("match-list / size: " << lst->size());
            for(int i = 0; i < lst->size(); i++) {
                ConnectingTypeMatch _match;
                if (! parseConnectingTypeMatch(lst->at(i), _match)) {
                    maplist = false;
                    break;
                }
                _match.index = i;
                self->listConnectingTypeMatch.push_back(_match);
            }
            if (!maplist) {
                ERROR_STREAM(" invalid connecting-type-match-list");
            }
        } else {
            ERROR_STREAM(" error connecting-type-match-list");
        }
    }
    return true;
}
void Settings::Impl::parseParts(YAMLReader &reader, std::vector<Parts> &results)
{
    for(int i = 0; i < reader.numDocuments(); i++) {
        ValueNode *val = reader.document(i);
        parseParts(val, results);
    }
}
void Settings::Impl::parseParts(ValueNode *val, std::vector<Parts> &results)
{
    if ( val->isMapping() ) {
        Parts pt;
        if(parseParts(val, pt)) {
            results.push_back(pt);
        }
    } else if ( val->isListing() ) {
        Listing *lst = val->toListing();
        for(int i = 0; i < lst->size(); i++) {
            Parts pt;
            if(parseParts(lst->at(i), pt)) {
                results.push_back(pt);
            }
        }
    }
}
bool Settings::Impl::parseCoords(Mapping *map_, coordinates &cds)
{
    bool all_res = true;
    {
    ValueNode *val = map_->find("rotation");
    if (val->isValid()) {
        std::vector<double> vec;
        if(readVector(val, vec, std::cerr)) {
            if (vec.size() >= 4) {
                AngleAxis _aa;
                uc.convRotationAngle(vec, _aa);
                cds.set(_aa);
            } else {
                ERROR_STREAM(" rotation size is required to be more than 4:");
                ::printNode(std::cerr, val);
                all_res = false;
            }
        } else {
            ERROR_STREAM(" rotation is required to be a list:");
            ::printNode(std::cerr, val);
            all_res = false;
        }
    } }
    {
    ValueNode *val = map_->find("translation");
    if (val->isValid()) {
        std::vector<double> vec;
        if(readVector(val, vec, std::cerr)) {
            if (vec.size() >= 3) {
                Vector3 _a_pos;
                uc.convPoint(vec, _a_pos);
                cds.set(_a_pos);
            } else {
                ERROR_STREAM(" translation size is required to be more than 3:");
                ::printNode(std::cerr, val);
                all_res = false;
            }
        } else {
            ERROR_STREAM(" translation is required to be a list:");
            ::printNode(std::cerr, val);
            all_res = false;
        }
    } }
    return all_res;
}
bool Settings::Impl::parseConnectingConf(ValueNode *vn, ConnectingConfiguration &out) {
    if ( !vn->isMapping() ) {
        // [todo] error message
        return false;
    }
    Mapping *val = vn->toMapping();
    ValueNode *ret;
    ret = val->find("name");
    if (ret->isValid() && ret->isScalar()) {
        out.name = ret->toString();
    } else {
        ERROR_STREAM(" not scalar");
        return false;
    }
    ret = val->find("description");
    if (ret->isValid() && ret->isScalar()) {
        out.description = ret->toString();
    }
    return parseCoords(val, out.coords);
}
bool Settings::Impl::parseConnectingTypeMatch(ValueNode *vn, ConnectingTypeMatch &out) {
    if ( !vn->isMapping() ) {
        // [todo] error message
        return false;
    }
    Mapping *val = vn->toMapping();
    ValueNode *ret = val->find("pair");
    if (ret->isValid() && ret->isListing()) {
        Listing *lst = ret->toListing();
        if (lst->size() < 2 || !lst->at(0)->isScalar() || !lst->at(1)->isScalar() ) {
            // [todo] error message
            return false;
        }
        std::string pair_a = lst->at(0)->toString();
        std::string pair_b = lst->at(1)->toString();
        auto it_a = reverseTypeNames.find(pair_a);
        if (it_a == reverseTypeNames.end()) { // not found
            // [todo]
            return false;
        }
        out.pair[0] = it_a->second;
        auto it_b = reverseTypeNames.find(pair_b);
        if (it_b == reverseTypeNames.end()) { // not found
            // [todo]
            return false;
        }
        out.pair[1] = it_b->second;
    } else {
        // [todo] error message
        return false;
    }
    ret = val->find("allowed-configuration");
    if (ret->isValid() && ret->isListing()) {
        Listing *lst = ret->toListing();
        for(int i = 0; i < lst->size(); i++) {
            if (lst->at(i)->isScalar()) {
                std::string str = lst->at(i)->toString();
                auto it = reverseConfNames.find(str);
                if (it == reverseConfNames.end()) { // not found
                    // [todo] error message
                    return false;
                }
                out.allowed_configuration.push_back(it->second);
            } else {
                // [todo] invalid but not critical
            }
        }
    } else {
        // [todo] error message
        return false;
    }
    return true;
}
bool Settings::Impl::parsePartsSettings(ValueNode *vn)
{
    if ( !vn->isListing() ) {
        // [todo]
        return false;
    }
    Listing *lst = vn->toListing();
    for(int i = 0; i < lst->size(); i++) {
        Parts pt;
        if(parseParts(lst->at(i), pt)) {
            if(self->validateParts(pt)) {
                self->insertParts(pt);
            } else {
                return false;
            }
#if 0
            // check : There are types with the same name.
            auto pt_ = self->mapParts.find(pt.type);
            if (pt_ != self->mapParts.end()) {
                ERROR_STREAM(" parseParts error: same type: " << pt.type);
                return false;
            }
            // check : There are connecting points or actuators with the same name
            {
                std::vector<std::string *> vec;
                for(auto it = pt.connecting_points.begin();
                    it != pt.connecting_points.end(); it++) {
                    vec.push_back(&(it->name));
                }
                for(auto it = pt.actuators.begin();
                    it != pt.actuators.end(); it++) {
                    vec.push_back(&(it->name));
                }
                //
                int len = vec.size();
                for(int i = 0; i < len - 1; i++) {
                    for(int j = i + 1; j < len; j++) {
                        if ( *(vec[i]) == *(vec[j]) ) {
                            ERROR_STREAM(" parseParts error: in type: " << pt.type << " / there are connecting-points with the same name [" << *(vec[i]) << "]");
                            return false;
                        }
                    }
                }
            }
            self->mapParts.insert(std::make_pair(pt.type, pt));
#endif
        } else {
            ERROR_STREAM(" parseParts error: ");
            ::printNode(std::cerr, lst->at(i));
            return false;
        }
    }
    return true;
}
bool Settings::Impl::parseParts(ValueNode *vn, Parts &out)
{
    if( !vn->isMapping() ) {
        // [todo]
        return false;
    }
    Mapping *mp = vn->toMapping();
    if(! ::mapString(mp, "type", out.type, std::cerr) ) {
        return false;
    }
    if(! ::mapString(mp, "class", out.class_name, std::cerr, false) ) {
        return false;
    }
    if(! ::mapString(mp, "description", out.description, std::cerr, false) ) {
        return false;
    }
    {   // <<visual
    ValueNode *val = mp->find("visual");
    if ( ! val->isValid() ) {
        // [todo]
    } else if ( ! val->isListing() ) {
        // [todo]
    } else {
        Listing *lst = val->toListing();
        for(int i = 0; i < lst->size(); i++) {
            Geometry geom;
            if(parseGeometry(lst->at(i), geom)) {
                out.visual.push_back(geom);
            }
        }
    } } // >>visual
    {   // <<collision
    ValueNode *val = mp->find("collision");
    if ( ! val->isValid() ) {
        // [todo]
    } else if ( ! val->isListing() ) {
        // [todo]
    } else {
        Listing *lst = val->toListing();
        for(int i = 0; i < lst->size(); i++) {
            Geometry geom;
            if(parseGeometry(lst->at(i), geom)) {
                out.collision.push_back(geom);
            }
        }
    } } // >>collision
    {   // <<mass-param
    out.hasMassParam = false;
    ValueNode *val = mp->find("mass-param");
    if (val->isValid() && val->isMapping()) {
        Mapping *mass_map = val->toMapping();
        double mass_;
        if(!mapDouble(mass_map, "mass", mass_, std::cerr)) {
            ERROR_STREAM("parameter mass can not be read");
            return false;
        }
        out.mass = uc.convMass(mass_);
        std::vector<double> com;
        if(!mapVector(mass_map, "center-of-mass", com, std::cerr)) {
            ERROR_STREAM("parameter center-of-mass can not be read");
            return false;
        } else {
            if (com.size() >= 3) {
                uc.convPoint(com, out.COM);
            } else {
                ERROR_STREAM(" size of center-of-mass is required to be more than 3:");
                return false;
            }
        }
        std::vector<double> it;
        if(!mapVector(mass_map, "inertia-tensor", it, std::cerr)) {
            ERROR_STREAM("parameter inertia-ternsor can not be read");
            return false;
        } else {
            if (it.size() >= 9) {
                uc.convInertiaTensor(it, out.inertia_tensor);
            } else {
                ERROR_STREAM(" size of inertia-tensor is required to be more than 9 (6):");
                return false;
            }
        }
        out.hasMassParam = true;
    } else if (!!val) {
        ERROR_STREAM(" mass-param required to be dictionary type");
        ::printNode(std::cerr, val);
    } } // mass-param

    {   // <<connecting-points
    ValueNode *val = mp->find("connecting-points");
    if ( ! val->isValid() ) {
        // [todo]
    } else if ( ! val->isListing() ) {
        // [todo]
    } else {
        Listing *lst = val->toListing();
        for(int i = 0; i < lst->size(); i++) {
            ConnectingPoint cpt;
            if(parseConnectingPoint(lst->at(i), cpt)) {
                out.connecting_points.push_back(cpt);
            }
        }
    } } // >>connecting-points
    {   // <<actuators
    ValueNode *val = mp->find("actuators");
    if ( ! val->isValid() ) {
        // [todo]
    } else if ( ! val->isListing() ) {
        // [todo]
    } else {
        Listing *lst = val->toListing();
        for(int i = 0; i < lst->size(); i++) {
            Actuator act;
            if(parseActuator(lst->at(i), act)) {
                out.actuators.push_back(act);
            }
        }
    } } // >>actuators
    {   // <<extra-data
    ValueNode *val = mp->find("extra-data");
    if ( ! val->isValid() ) {
        // [todo]
    } else if ( ! val->isListing() ) {
        // [todo]
    } else {
        Listing *lst = val->toListing();
        for(int i = 0; i < lst->size(); i++) {
            ExtraInfo einfo;
            if(parseExtraInfo(lst->at(i), einfo)) {
                out.extra_data.push_back(einfo);
            }
        }
    } } // >>extra-data
    return true;
}
bool Settings::Impl::parseGeometry(ValueNode *vn, Geometry &geom)
{
    if ( ! vn->isMapping() ) {
        // [todo]
        return false;
    }
    Mapping *mp = vn->toMapping();
    // coords
    parseCoords(mp, geom.coords);
    // scale
    geom.scale = 1.0;
    mapDouble(mp, "scale", geom.scale, std::cerr, false);
    // type
    std::string tp_; geom.type = Geometry::None;
    mapString(mp, "type", tp_, std::cerr, false);
    if (tp_ == "mesh") {
        geom.type = Geometry::Mesh;
    } else if (tp_ == "box") {
        geom.type = Geometry::Box;
    } else if (tp_ == "cylinder") {
        geom.type = Geometry::Cylinder;
    } else if (tp_ == "sphere") {
        geom.type = Geometry::Sphere;
    } else if (tp_ == "cone") {
        geom.type = Geometry::Cone;
    } else if (tp_ == "capsule") {
        geom.type = Geometry::Capsule;
    } else if (tp_ == "ellipsoid") {
        geom.type = Geometry::Ellipsoid;
    } else if (tp_.size() > 0) {
        ERROR_STREAM(" unknown geometry type: " << tp_);
        return false;
    }
    if (geom.type == Geometry::None) {
        ValueNode *val;
        std::vector<double> vec;
        if ( (val = mp->find("box"))->isValid() ) {
            if( readVector(val, vec, std::cerr) ) {
                if (vec.size() >= 3) {
                    geom.type = Geometry::Box;
                    std::vector<double> nvec;
                    uc.convLengthVector(vec, nvec);
                    geom.parameter = nvec;
                } else {
                    // [todo]
                }
            }
        } else if ( (val = mp->find("cylinder"))->isValid() ) {
            if( readVector(val, vec, std::cerr) ) {
                if (vec.size() >= 2) {
                    geom.type = Geometry::Cylinder;
                    std::vector<double> nvec;
                    uc.convLengthVector(vec, nvec);
                    geom.parameter = nvec;
                } else {
                    // [todo]
                }
            }
        } else if ( (val = mp->find("sphere"))->isValid() ) {
            if( readVector(val, vec, std::cerr) ) {
                if (vec.size() >= 1) {
                    geom.type = Geometry::Sphere;
                    std::vector<double> nvec;
                    uc.convLengthVector(vec, nvec);
                    geom.parameter = nvec;
                } else {
                    // [todo]
                }
            }
        } else if ( (val = mp->find("cone"))->isValid() ) {
            if( readVector(val, vec, std::cerr) ) {
                if (vec.size() >= 1) { // [todo] param length
                    geom.type = Geometry::Cone;
                    std::vector<double> nvec;
                    uc.convLengthVector(vec, nvec);
                    geom.parameter = nvec;
                } else {
                    // [todo]
                }
            }
        } else if ( (val = mp->find("capsule"))->isValid() ) {
            if( readVector(val, vec, std::cerr) ) {
                if (vec.size() >= 1) { // [todo] param length
                    geom.type = Geometry::Capsule;
                    std::vector<double> nvec;
                    uc.convLengthVector(vec, nvec);
                    geom.parameter = nvec;
                } else {
                    // [todo]
                }
            }
        } else if ( (val = mp->find("ellipsoid"))->isValid() ) {
            if( readVector(val, vec, std::cerr) ) {
                if (vec.size() >= 1) { // [todo] param length
                    geom.type = Geometry::Ellipsoid;
                    std::vector<double> nvec;
                    uc.convLengthVector(vec, nvec);
                    geom.parameter = nvec;
                } else {
                    // [todo]
                }
            }
        } else if ( (val = mp->find("dummy"))->isValid() ) {
            geom.type = Geometry::Dummy;
        }
        //
        if (geom.type == Geometry::None) {
            ERROR_STREAM(" Could not found geometry type!");
            return false;
        }
    } else if (geom.type == Geometry::Mesh) {
        if(! mapString(mp, "url", geom.url, std::cerr, true)) {
            ERROR_STREAM(" mesh type geometry requires url:");
            return false;
        }
    } else {
        std::vector<double> vec;
        if(! mapVector(mp, "parameter", vec, std::cerr, true)) {
            ERROR_STREAM(" non mesh type geometry requires parameter:");
            return false;
        }
        std::vector<double> nvec;
        uc.convLengthVector(vec, nvec);
        geom.parameter = nvec;
    }
    {
        geom.color[0] = -1.0f; // for checking color is changed
        geom.color[1] = 0.0f;
        geom.color[2] = 0.0f;
        std::vector<double> vec_;
        if ( mapVector(mp, "color", vec_, std::cerr, false) ) {
            for(int i = 0; i < vec_.size() && i < 3; i++) {
                geom.color[i] = vec_[i];
            }
        }
    }
    return true;
}
bool Settings::Impl::parseConnectingPoint(ValueNode *vn, ConnectingPoint &cpt)
{
    if ( ! vn->isMapping() ) {
        return false;
    }
    Mapping *mp = vn->toMapping();
    if (!mapString(mp, "name", cpt.name, std::cerr)) {
        return false;
    }
    ValueNode *val = mp->find("types");
    if ( ! val->isValid() ) {
        return false;
    }
    if ( ! val->isListing() ) {
        return false;
    }
    Listing *lst = val->toListing();
    for(int i = 0; i < lst->size(); i++) {
        if(lst->at(i)->isScalar()) {
            std::string key = lst->at(i)->toString();
            auto it = reverseTypeNames.find(key);
            if (it == reverseTypeNames.end()) {
                // [todo] key not found (not fatal)
                continue;
            }
            cpt.type_list.push_back(it->second);
        } else {
            // [todo] error (not fatal)
        }
    }
    if(cpt.type_list.size() < 1) {
        // [todo] no types
        return false;
    }
    parseCoords(mp, cpt.coords);

    return true;
}
bool Settings::Impl::parseActuator(ValueNode *vn, Actuator &act)
{
    if (! vn->isMapping() ) {
        return false;
    }
    Mapping *mp = vn->toMapping();
    std::string act_type;
    if (!mapString(mp, "actuator-type", act_type, std::cerr)) {
        return false;
    }
    ConnectingPoint::PartsType tp_;
    if (act_type == "rotational") {
        tp_ =  ConnectingPoint::Rotational;
    } else if (act_type == "linear") {
        tp_ =  ConnectingPoint::Linear;
    } else if (act_type ==  "fixed") {
        tp_ =  ConnectingPoint::Fixed;
    } else {
        ERROR_STREAM(" unknown actuator-type:" << act_type);
        return false;
    }
    act = Actuator(tp_);

    ValueNode *val = mp->find("axis");
    if ( ! val->isValid() ) {
        // [todo] no axis
        act.axis = Vector3(0, 0, 1);
    } else if ( val->isScalar() ) {
        std::string ax = val->toString();
        if (ax == "x") {
            act.axis = Vector3(1, 0, 0);
        } else if (ax == "y") {
            act.axis = Vector3(0, 1, 0);
        } else if (ax == "z") {
            act.axis = Vector3(0, 0, 1);
        } else if (ax == "-x") {
            act.axis = Vector3(-1, 0, 0);
        } else if (ax == "-y") {
            act.axis = Vector3(0, -1, 0);
        } else if (ax == "-z") {
            act.axis = Vector3(0, 0, -1);
        } else {
            ERROR_STREAM(" invalid axis[string] : " << ax);
            return false;
        }
    } else if ( val->isListing() ) {
        std::vector<double> ax;
        readVector(val, ax, std::cerr);
        if (ax.size() >= 3) {
            Vector3 v(ax[0], ax[1], ax[2]);
            v.normalize();
            act.axis = v;
        } else {
            // [todo] size error
        }
    } else {
        ERROR_STREAM(" invalid axis ");
        return false;
    }
    std::vector<double> qlim;
    mapVector(mp, "limit", qlim, std::cerr, false);
    if (qlim.size() >= 2) {
        if (tp_ ==  ConnectingPoint::Rotational) {
            act.limit[0] = uc.convAngle(qlim[0]);
            act.limit[1] = uc.convAngle(qlim[1]);
        } else {
            act.limit[0] = qlim[0];
            act.limit[1] = qlim[1];
        }
    }
    std::vector<double> vlim;
    mapVector(mp, "vlimit", vlim, std::cerr, false);
    if (vlim.size() >= 2) {
        if (tp_ ==  ConnectingPoint::Rotational) {
            act.vlimit[0] = uc.convAngle(vlim[0]);
            act.vlimit[1] = uc.convAngle(vlim[1]);
        } else {
            act.vlimit[0] = vlim[0];
            act.vlimit[1] = vlim[1];
        }
    }
    std::vector<double> tlim;
    mapVector(mp, "tqlimit", tlim, std::cerr, false);
    if (tlim.size() >= 2) {
        act.tqlimit[0] = tlim[0];
        act.tqlimit[1] = tlim[1];
    }
    bool ret;
    ret = parseConnectingPoint(vn, *static_cast<ConnectingPoint *>(&act) );
    if (!ret) {
        // [todo] fatal
        return false;
    }

    return true;
}
bool Settings::Impl::parseExtraInfo(ValueNode *vn, ExtraInfo &einfo)
{
    if (! vn->isMapping() ) {
        return false;
    }
    Mapping *mp = vn->toMapping();
    if (!mapString(mp, "name", einfo.name, std::cerr)) {
        return false;
    }
    if(!mapString(mp, "description", einfo.description, std::cerr, false) ) {
        return false;
    }
    std::string _sen_type;
    if (!mapString(mp, "type", _sen_type, std::cerr)) {
        return false;
    }
    std::transform(_sen_type.cbegin(), _sen_type.cend(), _sen_type.begin(), ::toupper);
    if (_sen_type == "IMU") {
        einfo.type = ExtraInfo::IMU;
    } else if (_sen_type == "TOUCH") {
        einfo.type = ExtraInfo::Touch;
    } else if (_sen_type == "FORCE") {
        einfo.type = ExtraInfo::Force;
    } else if (_sen_type == "FORCETORQUE") {
        einfo.type = ExtraInfo::Force;
    } else if (_sen_type == "COLOR") {
        einfo.type = ExtraInfo::Color;
    } else if (_sen_type == "DISTANCE") {
        einfo.type = ExtraInfo::Distance;
    } else if (_sen_type == "POSITION") {
        einfo.type = ExtraInfo::Position;
    } else if (_sen_type == "CAMERA") {
        einfo.type = ExtraInfo::Camera;
    } else if (_sen_type == "DEPTH") {
        einfo.type = ExtraInfo::Depth;
    } else if (_sen_type == "RGBD") {
        einfo.type = ExtraInfo::RGBD;
    } else if (_sen_type == "RAY") {
        einfo.type = ExtraInfo::Ray;
    } else {
        ERROR_STREAM("unknown sensor type: " << _sen_type);
        return false;
    }
    {   // coords
        bool res = parseCoords(mp, einfo.coords);
    }
    {   // parameters
        bool res = mapVector(mp, "parameters", einfo.parameters, std::cerr, false);
    }
    {   // device_mapping
        ValueNode *vn_ = mp->find("device-mapping");
        if(vn_->isValid() && vn_->isMapping()) {
            YAMLWriter yw_;
            yw_.setMessageSink(std::cerr);
            yw_.setDoubleFormat("%12.12f");
            std::ostringstream oss_;
            yw_.setOutput(oss_);
            MappingPtr mp_ = vn_->toMapping();
            yw_.putNode(mp_);
            yw_.flush();
            einfo.device_mapping = oss_.str();
        }
    }
    return true;
}
static bool parse(Mapping *map_, coordinates &cds, bool check = true)
{
    if (check) {
        bool valid = false;
        ValueNode *val = map_->find("rotation");
        if(val->isValid() && val->isListing()) valid = true;
        if(!valid) {
            ValueNode *valt = map_->find("translation");
            if(valt->isValid() && valt->isListing()) valid = true;
        }
        if(!valid) {
            DEBUG_STREAM(" invalid coords in yaml");
        }
    }
    bool all_res = true;
    ValueNode *valr = map_->find("rotation");
    if (valr->isValid()) {
        std::vector<double> vec;
        if(readVector(valr, vec, std::cerr)) {
            if (vec.size() >= 4) {
                Vector3 ax_(vec[0], vec[1], vec[2]);
                AngleAxis aa_(vec[3], ax_);
                cds.set(aa_);
            } else {
                all_res = false;
            }
        } else {
            all_res = false;
        }
    }
    ValueNode *valt = map_->find("translation");
    if (valt->isValid()) {
        std::vector<double> vec;
        if(readVector(valt, vec, std::cerr)) {
            if (vec.size() >= 3) {
                Vector3 _a_pos(vec[0], vec[1], vec[2]);
                cds.set(_a_pos);
            } else {
                all_res = false;
            }
        } else {
            all_res = false;
        }
    }
    return all_res;
}
static bool parse(ValueNode *_vn, AttachHistoryItem& hist)
{
    if(! _vn->isMapping()) return false;
    Mapping *mp_ = _vn->toMapping();
    mapString(mp_, "parts-name", hist.parts_name, std::cerr, false);
    mapString(mp_, "parts-type", hist.parts_type, std::cerr, false);
    mapString(mp_, "parts-point", hist.parts_point, std::cerr, false);
    mapString(mp_, "parts-point-url", hist.parts_point_url, std::cerr, false);
    mapString(mp_, "parent", hist.parent, std::cerr, false);
    mapString(mp_, "parent-point", hist.parent_point, std::cerr, false);
    mapString(mp_, "parent-point-url", hist.parent_point_url, std::cerr, false);
    mapString(mp_, "config-name", hist.config_name, std::cerr, false);
    {
        ValueNode *vn = mp_->find("connecting-offset");
        if(vn->isValid() && vn->isMapping()) {
            Mapping *m_cds_ = vn->toMapping();
            parse(m_cds_, hist.connecting_offset);
        }
    }
    ValueNode *vn = mp_->find("initial-parts");
    if(vn->isValid()) hist.initial_parts = true;
    return true;
}
static bool parse(ValueNode *_vn, AssembleConfig& config)
{
    if(! _vn->isMapping()) return false;
    Mapping *mp_ = _vn->toMapping();
    mapString(mp_, "robot-name", config.robot_name, std::cerr, false);
    ValueNode *vi = mp_->find("initial-coords");
    if(vi->isValid() && vi->isMapping()) {
        parse(vi->toMapping(), config.initial_coords);
    }
    ValueNode *an = mp_->find("actuator-name");
    if(an->isValid() && an->isMapping()) {
        Mapping *am = an->toMapping();
        for(auto it = am->begin(); it != am->end(); it++) {
            if(it->second->isScalar()) {
                config.actuator_name.insert(
                    std::pair<std::string, std::string>(it->first, it->second->toString()));
            }
        }
    }
    ValueNode *ax = mp_->find("actuator-axis");
    if(ax->isValid() && ax->isMapping()) {
        Mapping *am = ax->toMapping();
        for(auto it = am->begin(); it != am->end(); it++) {
            if(it->second->isScalar()) {
                config.actuator_axis_name.insert(
                    std::pair<std::string, std::string>(it->first, it->second->toString()));
            } else if (it->second->isListing()) {
                // [TODO]
            }
        }
    }
    return true;
}
static bool parseYaml(YAMLReader &yaml_reader, AttachHistory &history, AssembleConfig  &config, bool parse_config)
{
    bool ret = false;
    history.clear();
    for(int i = 0; i < yaml_reader.numDocuments(); i++) {
        ValueNode *val = yaml_reader.document(i);
        if ( !val->isMapping() ) continue;
        std::string key = "history";
        ValueNode *target = val->toMapping()->find(key);
        if( ! target->isValid() ) continue;
        DEBUG_STREAM("--- target " << key << " found ---");
        if( ! target->isListing() ) continue;
        Listing *lst = target->toListing();
        for(int i = 0; i < lst->size(); i++) {
            AttachHistoryItem hist;
            DEBUG_STREAM(" hist : " << i);
            if ( parse(lst->at(i), hist) ) {
                history.push_back(hist);
            }
        }
    }
    if ( history.size() < 1 ) {
        ERROR_STREAM("failed to parse roboasm");
        return false;
    }
    if(!parse_config) return true;
    for(int i = 0; i < yaml_reader.numDocuments(); i++) {
        ValueNode *val = yaml_reader.document(i);
        if ( !val->isMapping() ) continue;
        std::string key = "assemble-config";
        ValueNode *target = val->toMapping()->find(key);
        if( ! target->isValid() ) continue;
        DEBUG_STREAM("--- target " << key << " found ---");
        if( ! target->isMapping() ) continue;
        parse(target, config);
    }
    return true;
}
bool RoboasmFile::parseRoboasm(const std::string &_filename, bool parse_config)
{
    YAMLReader yaml_reader;
    if (! yaml_reader.load(_filename)) {
        ERROR_STREAM(" roboasm file Loading error : " << _filename);
        return false;
    }
    return parseYaml(yaml_reader, this->history, this->config, parse_config);
}
bool RoboasmFile::parseRoboasmFromString(const std::string &yml_string, bool parse_config)
{
    YAMLReader yaml_reader;
    if (! yaml_reader.parse(yml_string)) {
        ERROR_STREAM(" roboasm yaml parsing error : " << yml_string);
        return false;
    }
    return parseYaml(yaml_reader, this->history, this->config, parse_config);
}
static inline void dumpCoords(YAMLWriter &yaml_writer, const coordinates &cds, double eps = 1e-12)
{
    const Vector3 &v = cds.pos;
    yaml_writer.startFlowStyleMapping();
    if(v.norm() > eps) {
        yaml_writer.putKey("translation");
        yaml_writer.startFlowStyleListing();
        if(std::fabs(v(0)) <= eps) {
            yaml_writer.putScalar(0);
        } else {
            yaml_writer.putScalar(v(0));
        }
        if(std::fabs(v(1)) <= eps) {
            yaml_writer.putScalar(0);
        } else {
            yaml_writer.putScalar(v(1));
        }
        if(std::fabs(v(2)) <= eps) {
            yaml_writer.putScalar(0);
        } else {
            yaml_writer.putScalar(v(2));
        }
        yaml_writer.endListing();
    }
    AngleAxis a(cds.rot);
    Vector3 &av = a.axis();
    if(a.angle() > eps) {
        yaml_writer.putKey("rotation");
        yaml_writer.startFlowStyleListing();
        if(std::fabs(av(0)) <= eps) {
            yaml_writer.putScalar(0);
        } else {
            yaml_writer.putScalar(av(0));
        }
        if(std::fabs(av(1)) <= eps) {
            yaml_writer.putScalar(0);
        } else {
            yaml_writer.putScalar(av(1));
        }
        if(std::fabs(av(2)) <= eps) {
            yaml_writer.putScalar(0);
        } else {
            yaml_writer.putScalar(av(2));
        }
        yaml_writer.putScalar(a.angle());
        yaml_writer.endListing();
    }
    yaml_writer.endMapping();
}
static bool dumpYaml(YAMLWriter &yaml_writer, AttachHistory &history, AssembleConfig  &config)
{
    //
    yaml_writer.startDocument();
    yaml_writer.startMapping(); // main
    yaml_writer.putKey("history");
    yaml_writer.startListing();
    for(auto it = history.begin(); it != history.end(); it++) {
        yaml_writer.startMapping();
        yaml_writer.putKeyValue("parts-name", (*it).parts_name);
        yaml_writer.putKeyValue("parts-type", (*it).parts_type);
        if(!(*it).initial_parts) {
            if((*it).parts_point.size() > 0)
                yaml_writer.putKeyValue("parts-point", (*it).parts_point);
            if((*it).parts_point.size() == 0 && (*it).parts_point_url.size() > 0)
                yaml_writer.putKeyValue("parts-point-url", (*it).parts_point_url);
            if((*it).parent.size() > 0)
                yaml_writer.putKeyValue("parent", (*it).parent);
            if((*it).parent_point.size() > 0)
                yaml_writer.putKeyValue("parent-point", (*it).parent_point);
            if( ((*it).parent.size() == 0 || (*it).parent_point.size() == 0) &&
                (*it).parent_point_url.size() > 0 ) {
                yaml_writer.putKeyValue("parent-point-url", (*it).parent_point_url);
            }
            if((*it).connecting_offset.isInitial()) {
                    // no config
            } else {
                if(!(*it).connecting_offset.isInitial(1e-12)) {
                    yaml_writer.putKey("connecting-offset");
                    dumpCoords(yaml_writer, (*it).connecting_offset);
                }
            }
        } else {
            yaml_writer.putKeyValue("initial-parts", true);
        }
        yaml_writer.endMapping();
    }
    yaml_writer.endListing();
    yaml_writer.endMapping(); // main

    if(config.isValid()) {
        yaml_writer.startMapping(); // main
        yaml_writer.putKey("assemble-config");
        yaml_writer.startMapping(); // assemble-config
        if(config.robot_name.size() > 0) {
            yaml_writer.putKeyValue("robot-name", config.robot_name);
        }
        if( !config.initial_coords.isInitial() ) {
            yaml_writer.putKey("initial-coords");
            dumpCoords(yaml_writer, config.initial_coords);
        }
        if(config.actuator_name.size() > 0) {
            yaml_writer.putKey("actuator-name");
            yaml_writer.startMapping(); // actuator-name
            for(auto it = config.actuator_name.begin();
                it != config.actuator_name.end(); it++) {
                yaml_writer.putKeyValue(it->first, it->second);
            }
            yaml_writer.endMapping(); // actuator-name
        }
        if(config.actuator_axis_name.size() > 0 || config.actuator_axis_vector.size() > 0) {
            yaml_writer.putKey("actuator-axis");
            yaml_writer.startMapping(); // actuator-axis
            for(auto it = config.actuator_axis_name.begin();
                it != config.actuator_axis_name.end(); it++) {
                yaml_writer.putKeyValue(it->first, it->second);
            }
            for(auto it = config.actuator_axis_vector.begin();
                it != config.actuator_axis_vector.end(); it++) {
                // [TODO]
            }
            yaml_writer.endMapping(); // actuator-axis
        }
        yaml_writer.endMapping(); // assemble-config
        yaml_writer.endMapping(); // main
    }

    yaml_writer.flush();
    yaml_writer.closeFile();

    return true;
}
bool RoboasmFile::dumpRoboasm(const std::string &_filename)
{
    YAMLWriter yaml_writer;
    yaml_writer.setMessageSink(std::cerr);
    yaml_writer.setDoubleFormat("%12.12f");
    if(!yaml_writer.openFile(_filename)) {
        return false;
    }
    return dumpYaml(yaml_writer, this->history, this->config);
}
#include <sstream>
bool RoboasmFile::dumpRoboasmToString(std::string &result_yml)
{
    YAMLWriter yaml_writer;
    yaml_writer.setMessageSink(std::cerr);
    yaml_writer.setDoubleFormat("%12.12f");
    std::ostringstream oss;
    yaml_writer.setOutput(oss);
    bool res = dumpYaml(yaml_writer, this->history, this->config);
    if (!res) {
        return false;
    }
    result_yml = oss.str();
    return true;
}
