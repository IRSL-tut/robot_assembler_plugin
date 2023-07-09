#include "RobotAssemblerBody.h"
#include "RobotAssemblerInfo.h"
// shape
#include <cnoid/SceneLoader>
#include <cnoid/MeshGenerator>
#include <cnoid/CloneMap>

// Devices
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <cnoid/AccelerationSensor>
#include <cnoid/RateGyroSensor>
#include <cnoid/ForceSensor>
#include <cnoid/Light>
#include <cnoid/PointLight>
#include <cnoid/SpotLight>

#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>

//#define IRSL_DEBUG
#include "irsl_debug.h"

namespace filesystem = cnoid::stdx::filesystem;

namespace cnoid {
namespace robot_assembler {

const Vector3f default_body_color(0.0f, 0.0f, 0.8f);
const Vector3f default_mesh_color(0.95f, 0.95f, 0.95f);

static inline void addMaterial(SgNode *_nd, const Vector3f &_color, float _intensity = 0.7)
{
    SgShape *_shape = dynamic_cast<SgShape *>(_nd);
    if(!_shape) return;
    SgMaterialPtr mat_(new SgMaterial());
    //mat_->setName("material");
    //Vector3f color(0.1f, 0.1f, 0.7f);
    mat_->setDiffuseColor(_color);
    mat_->setAmbientIntensity(_intensity);
    mat_->setEmissiveColor(Vector3f(0.0f, 0.0f, 0.0f));
    _shape->setMaterial(mat_);
}
static inline SgPosTransformPtr addShape(const std::string &_name, const SgMeshPtr mesh, const Vector3f &_color, Geometry &geom)
{
    SgShapePtr shape(new SgShape());
    shape->setMesh(mesh);
    shape->setName(_name);
    if(!_color.isZero()) {
        addMaterial(shape, _color);
    } else if (geom.color[0] >= 0.0) { // checking color is changed
        addMaterial(shape, geom.color);
    } else {
        addMaterial(shape, default_body_color);
    }
    SgPosTransformPtr trs(new SgPosTransform);
    geom.coords.toPosition(trs->position());
    trs->setName("geom_postrans");
    trs->addChild(shape);
    return trs;
}
void createSceneFromGeometry(SgGroup *sg_main, std::vector<Geometry> &geom_list, const Vector3f &_color) {
    createSceneFromGeometry(sg_main, geom_list, std::string(), _color);
}
void createSceneFromGeometry(SgGroup *sg_main, std::vector<Geometry> &geom_list,
                             const std::string &_proj_dir, const Vector3f &_color)
{
    DEBUG_PRINT();
    if (geom_list.size() <= 0) {
        //
        return;
    }
    const std::string &name_ = sg_main->name();
    for(int i = 0; i < geom_list.size(); i++) {
        Geometry &geom = geom_list[i];
        if (geom.type == Geometry::Mesh) {
            SceneLoader sceneLoader;
            sceneLoader.setMessageSink(std::cerr);
            std::string geom_file_path = geom.url;
            filesystem::path path_(fromUTF8(geom.url));
            if(_proj_dir.size() > 0 && path_.is_relative()) {
                geom_file_path = _proj_dir;
                geom_file_path = geom_file_path + "/" + geom.url;
            }
            DEBUG_STREAM(" mesh load: " << geom.url << " / " << geom_file_path);
            SgNodePtr shape = sceneLoader.load(geom_file_path);
            if (!!shape) {
                DEBUG_STREAM(" mesh loaded!");
                shape->setName(name_ + "/geom");
                shape->setUri(geom.url, geom_file_path);
                if(!_color.isZero()) {
                    addMaterial(shape, _color);
                } else if (geom.color[0] >= 0.0) { // checking color is changed
                    addMaterial(shape, geom.color);
                } else { // default color
                    addMaterial(shape, default_mesh_color);
                }
                Position p; geom.coords.toPosition(p);
                SgPosTransformPtr trs(new SgPosTransform(p));
                trs->setName(name_ + "/geom_postrans");
#define SCALE_EPS 0.005
                if ( geom.scale != 0.0 && ((geom.scale < 1.0 - SCALE_EPS) || (geom.scale > 1.0 + SCALE_EPS)) ) {
                    SgScaleTransformPtr scl_(new SgScaleTransform(geom.scale));
                    scl_->addChild(shape);
                    trs->addChild(scl_);
                    sg_main->addChild(trs);
                } else {
                    trs->addChild(shape);
                    sg_main->addChild(trs);
                }
            }
        } else if (geom.type == Geometry::Box) {
            MeshGenerator mg;
            SgMeshPtr mesh = mg.generateBox(Vector3(geom.parameter[0], geom.parameter[1], geom.parameter[2]));
            //SgMesh* generateBox(const Vector3& size, int options = NoOption);
            SgPosTransformPtr trs = addShape(name_ + "/box", mesh, _color, geom);
            sg_main->addChild(trs);
        } else if (geom.type == Geometry::Cylinder) {
            MeshGenerator mg;
            SgMeshPtr mesh = mg.generateCylinder(geom.parameter[0], geom.parameter[1]);
            //SgMesh* generateCylinder(double radius, double height, int options = NoOption);
            SgPosTransformPtr trs = addShape(name_ + "/cylinder", mesh, _color, geom);
            sg_main->addChild(trs);
        } else if (geom.type == Geometry::Sphere) {
            MeshGenerator mg;
            SgMeshPtr mesh = mg.generateSphere(geom.parameter[0]);
            //SgMesh* generateSphere(double radius, int options = NoOption);
            SgPosTransformPtr trs = addShape(name_ + "/sphere", mesh, _color, geom);
            sg_main->addChild(trs);
        } else if (geom.type == Geometry::Cone) {
            MeshGenerator mg;
            SgMeshPtr mesh = mg.generateCone(geom.parameter[0], geom.parameter[1]);
            //SgMesh* generateCone(double radius, double height, int options = NoOption);
            SgPosTransformPtr trs = addShape(name_ + "/cone", mesh, _color, geom);
            sg_main->addChild(trs);
        } else if (geom.type == Geometry::Capsule) {
            MeshGenerator mg;
            SgMeshPtr mesh = mg.generateCapsule(geom.parameter[0], geom.parameter[1]);
            //SgMesh* generateCapsule(double radius, double height);
            SgPosTransformPtr trs = addShape(name_ + "/capsule", mesh, _color, geom);
            sg_main->addChild(trs);
        }
    }
}
static cnoid::Device *createDevice(ExtraInfo &_info, int dev_no, coordinates &link_origin_to_self_)
{
    cnoid::Device *ret = nullptr;
    MappingPtr mp_ = nullptr;
    if (_info.device_mapping.size() > 0) {
        YAMLReader rd_;
        bool res_ = rd_.parse(_info.device_mapping);
        if(res_ && rd_.numDocuments() > 0) {
            ValueNode *vn_ = rd_.document();
            if( vn_->isValid() && vn_->isMapping() ) {
                mp_ = vn_->toMapping();
            }
        }
    }

    switch(_info.type) {
    case ExtraInfo::IMU:
        break;
    case ExtraInfo::Acceleration:
    {
        cnoid::AccelerationSensor *dev_ = new AccelerationSensor();
        if(!!mp_ && mp_->isValid()) {
            dev_->readSpecifications(mp_);
        }
        ret = dev_;
    }
    break;
    case ExtraInfo::RateGyro:
    {
        cnoid::RateGyroSensor *dev_ = new RateGyroSensor();
        if(!!mp_ && mp_->isValid()) {
            dev_->readSpecifications(mp_);
        }
        ret = dev_;
    }
    break;
    case ExtraInfo::Touch:
        ERROR_STREAM(" TouchSensor is not implemented yet : " << _info.name);
        break;
    case ExtraInfo::Force:
    {
        cnoid::ForceSensor *dev_ = new ForceSensor();
        if(!!mp_ && mp_->isValid()) {
            dev_->readSpecifications(mp_);
        }
        ret = dev_;
    }
    case ExtraInfo::Color:
    case ExtraInfo::Camera:
    {
        cnoid::Camera *dev_ = new Camera();
        if(!!mp_ && mp_->isValid()) {
            dev_->readSpecifications(mp_);
        }
        ret = dev_;
    }
    break;
    case ExtraInfo::Distance:
    case ExtraInfo::Depth:
    case ExtraInfo::RGBD:
    {
        cnoid::RangeCamera *dev_ = new RangeCamera();
        if(!!mp_ && mp_->isValid()) {
            dev_->readSpecifications(mp_);
        }
        ret = dev_;
    }
    break;
    case ExtraInfo::Ray:
    {
        cnoid::RangeSensor *dev_ = new RangeSensor();
        if(!!mp_ && mp_->isValid()) {
            dev_->readSpecifications(mp_);
        }
        ret = dev_;
    }
    break;
    case ExtraInfo::Position:
        ERROR_STREAM(" PositionSensor is not implemented yet : " << _info.name);
        break;
    case ExtraInfo::Light:
    case ExtraInfo::PointLight:
    {
        cnoid::PointLight *dev_ = new PointLight();
        if(!!mp_ && mp_->isValid()) {
            dev_->readSpecifications(mp_);
        }
        ret = dev_;
    }
    case ExtraInfo::SpotLight:
    {
        cnoid::SpotLight *dev_ = new SpotLight();
        if(!!mp_ && mp_->isValid()) {
            dev_->readSpecifications(mp_);
        }
        ret = dev_;
    }
    case ExtraInfo::DegitalIO:
        ERROR_STREAM(" DegitalIO is not implemented yet : " << _info.name);
        break;
    case ExtraInfo::AnalogIO:
        ERROR_STREAM(" AnalogIO is not implemented yet : " << _info.name);
        break;
    default:
        ERROR_STREAM(" invalid device-type [" << _info.name << "] " << _info.type);
        break;
    }

    if (!ret) return ret;

    std::ostringstream oss_;
    oss_ << _info.name << dev_no;
    ret->setName(oss_.str());

    DEBUG_STREAM(" device local: " << _info.coords);
    coordinates c_pos(link_origin_to_self_);
    c_pos.transform(_info.coords);
    DEBUG_STREAM(" device local(new): " << c_pos);
    Isometry3 pos;
    c_pos.toPosition(pos);
    ret->setLocalPosition(pos);

    return ret;
}
RoboasmBodyCreator::RoboasmBodyCreator(const std::string &_proj_dir) : body(nullptr), joint_counter(0), device_counter(0), merge_fixed_joint(false)
{
    project_directory = _proj_dir;
}
Link *RoboasmBodyCreator::createLink(RoboasmPartsPtr _pt, bool _is_root, DevLinkList &_dev_list)
{
    Link *lk = new Link();
    coordinates link_origin_to_self_;
    cnoidRAInfo cinfo(info);

    if (_is_root) { // for root link
        std::string nm_("Root");
        cinfo.getPartsName(_pt->name(), nm_);
        lk->setName(nm_);
        //lk->setJointName();
        lk->setJointType(Link::FreeJoint);
        lk->setJointId(-1);
        // initial-offset : RoboasmRobot => RoboasmParts(root-parts)
        Position p;
        _pt->worldcoords().toPosition(p);
        lk->setOffsetPosition(p);
        DEBUG_STREAM(" coords(root) : " << _pt->worldcoords());
        //
        map_link_cnoid_roboasm.insert(std::pair<std::string, std::string>(nm_, _pt->name()));
    } else { // usual (has parent) link
        RoboasmPartsPtr p_pt_;
        RoboasmConnectingPointPtr p_cp_;
        RoboasmConnectingPointPtr s_cp_;
        if(!_pt->parentParts(p_pt_, p_cp_, s_cp_)) {
            ERROR_STREAM(" miss fetch : " << _pt->name());
            delete lk;
            return nullptr;
        }
        s_cp_->worldcoords().transformation(link_origin_to_self_,
                                            _pt->worldcoords());
        if(!p_pt_->parent()) {
            ERROR_STREAM(" invalid parts : " << p_pt_->name());
            delete lk;
            return nullptr;
        }
        coordinates p_origin = p_pt_->parent()->worldcoords();
        if(p_pt_->parent()->isRobot()) {
            p_origin = p_pt_->worldcoords();
        }
        coordinates offset_cds;
        p_origin.transformation(offset_cds, s_cp_->worldcoords());
        Position p; offset_cds.toPosition(p);
        lk->setOffsetPosition(p);
        DEBUG_STREAM(" link : " << _pt->name());
        DEBUG_STREAM(" parent : " << p_pt_->name());
        DEBUG_STREAM(" coords(parent) : " << p_origin);
        DEBUG_STREAM(" coords(self) : " << s_cp_->worldcoords());
        DEBUG_STREAM(" offset : " << offset_cds);
        DEBUG_STREAM(" link_origin_to_self : " << link_origin_to_self_);

        RoboasmConnectingPointPtr act_;
        if (p_cp_->isActuator()) {
            act_ = p_cp_;
        } else if (s_cp_->isActuator()) {
            act_ = s_cp_;
        }
        if( !!act_ ) {
            // link-name
            std::ostringstream ss_;
            ss_ << "LINK_" << joint_counter;
            std::string lnm_ = ss_.str();
            cinfo.getPartsName(_pt->name(), lnm_);
            lk->setName(lnm_);
            // joint-name
            std::string jnm_ = lnm_;
            if(cinfo.getActuatorName(act_->name(), jnm_)) {
                lk->setJointName(jnm_);
            }
            DEBUG_STREAM("  _pt->name() : " << _pt->name());
            DEBUG_STREAM(" act_->name() : " << act_->name());
            DEBUG_STREAM("         lnm_ : " << lnm_);
            DEBUG_STREAM("         jnm_ : " << jnm_);
            lk->setJointId(joint_counter++); // [todo] overwrite id by info
            map_link_cnoid_roboasm.insert(std::pair<std::string, std::string>(lnm_, _pt->name()));
            //map_joint_cnoid_roboasm.insert(std::pair<std::string, std::string>(jnm_, _pt->name()));
            Actuator *ainfo_ = dynamic_cast<Actuator*>(act_->info);
            if(!!ainfo_) {
                if (act_ == s_cp_) {
                    lk->setJointAxis(- ainfo_->axis); // [todo] update direction by info
                } else {
                    lk->setJointAxis(ainfo_->axis); // [todo] update direction by info
                }
                switch (ainfo_->getType()) {
                case ConnectingPoint::Rotational:
                    lk->setJointType(Link::RevoluteJoint);
                    break;
                case ConnectingPoint::Linear:
                    lk->setJointType(Link::PrismaticJoint);
                    break;
                case ConnectingPoint::Fixed: // ??
                    lk->setJointType(Link::RevoluteJoint);
                    lk->setJointRange(0, 0);
                    break;
                case ConnectingPoint::Free: // ??
                    lk->setJointType(Link::FreeJoint);
                    break;
                }
                DEBUG_STREAM(" limit : " << ainfo_->limit[0] << " / " << ainfo_->limit[1]);
                double lim_min_, lim_max_;
                if(cinfo.getActuatorLimit(act_->name(), "limit", lim_min_, lim_max_)) {
                    lk->setJointRange(lim_min_, lim_max_);
                } else {
                    lk->setJointRange(ainfo_->limit[0], ainfo_->limit[1]);
                }
                if(cinfo.getActuatorLimit(act_->name(), "vlimit", lim_min_, lim_max_)) {
                    lk->setJointVelocityRange(lim_min_, lim_max_);
                } else {
                    lk->setJointVelocityRange(ainfo_->vlimit[0], ainfo_->vlimit[1]);
                }
                if(cinfo.getActuatorLimit(act_->name(), "tqlimit", lim_min_, lim_max_)) {
                    lk->setJointEffortRange(lim_min_, lim_max_);
                } else {
                    lk->setJointEffortRange(ainfo_->tqlimit[0], ainfo_->tqlimit[1]);
                }
                double init_ang;
                if (cinfo.getActuatorValue(act_->name(), "initial-angle",  init_ang)) {
                    DEBUG_STREAM(" " << act_->name() << ", initial-angle <= " << init_ang);
                    lk->setInitialJointAngle(init_ang);
                }
            } else {
                // no ainfo
            }
        } else { // not actuator(fixed connect)
            std::string nm_ = _pt->name();
            cinfo.getPartsName(_pt->name(), nm_);
            lk->setName(nm_);
            lk->setJointType(Link::FixedJoint);
            lk->setJointId(-1);
            map_link_cnoid_roboasm.insert(std::pair<std::string, std::string>(nm_, _pt->name()));
        }
    } // if (_is_root) {

    // Mass Parameter
    if (!!_pt->info && _pt->info->hasMassParam) {
        Vector3 com(_pt->info->COM);
        link_origin_to_self_.transform_vector(com);
        lk->setCenterOfMass(com);
        lk->setMass(_pt->info->mass);
        Matrix3 rr(link_origin_to_self_.rot);
        Matrix3 in_(rr * _pt->info->inertia_tensor * rr.transpose());
        lk->setInertia(in_);
    }
#if 0
    // Debug for shape
    MeshGenerator mg;
    // com / next point
    { // red parts(origin)
        SgMeshPtr mesh = mg.generateBox(Vector3(0.004, 0.004, 0.004));
        SgShapePtr shape(new SgShape());
        shape->setMesh(mesh);
        SgMaterialPtr material(new SgMaterial());
        material->setDiffuseColor(Vector3f(1.0f, 0.0f, 0.0f));
        material->setEmissiveColor(Vector3f(0.0f, 0.0f, 0.0f));
        material->setAmbientIntensity(0.7f);
        shape->setMaterial(material);
        Position p; link_origin_to_self_.toPosition(p);
        SgPosTransformPtr trs(new SgPosTransform(p));
        trs->addChild(shape);
        lk->addShapeNode(trs); //
    }
    if(!_is_root)
    { // green self_cp
        SgMeshPtr mesh = mg.generateBox(Vector3(0.004, 0.004, 0.004));
        SgShapePtr shape(new SgShape());
        shape->setMesh(mesh);
        SgMaterialPtr material(new SgMaterial());
        material->setDiffuseColor(Vector3f(0.0f, 1.0f, 0.0f));
        material->setEmissiveColor(Vector3f(0.0f, 0.0f, 0.0f));
        material->setAmbientIntensity(0.7f);
        shape->setMaterial(material);
        // link_origin
        SgPosTransformPtr trs(new SgPosTransform());
        trs->addChild(shape);
        lk->addShapeNode(trs); //
    }
#endif
    // Geometry
    if(!!_pt->info) {
        SgPosTransformPtr trs_vis;
        SgPosTransformPtr trs_col;
        // [TODO] update color by info
        Vector3f col_;
        if(_pt->info->visual.size() > 0) {
            trs_vis = (new SgPosTransform());
            trs_vis->setName(_pt->name() + "/visual");
            createSceneFromGeometry(trs_vis, _pt->info->visual, project_directory, _pt->color);
        }
        if(_pt->info->collision.size() > 0) {
            trs_col = (new SgPosTransform());
            trs_col->setName(_pt->name() + "/collision");
            createSceneFromGeometry(trs_col, _pt->info->collision, project_directory, _pt->color);
        }
        if(!!trs_vis) {
            Position p; link_origin_to_self_.toPosition(p);
            trs_vis->position() = p;
        }
        if(!!trs_col) {
            Position p; link_origin_to_self_.toPosition(p);
            trs_col->position() = p;
        }
        if (!!trs_vis) {
            lk->addVisualShapeNode(trs_vis);
        } else if (!!trs_col) {
            lk->addVisualShapeNode(trs_col);
        }
        if (!!trs_col) {
            lk->addCollisionShapeNode(trs_col);
        } else if (!!trs_vis) {
            lk->addCollisionShapeNode(trs_vis);
        }
    }
    // Device
    if(!!_pt->info && _pt->info->extra_data.size() > 0) {
        for(auto it = _pt->info->extra_data.begin();
            it != _pt->info->extra_data.end(); it++) {
            cnoid::Device *dev = createDevice(*it, device_counter++, link_origin_to_self_);
            if(!!dev) {
                // device name
                std::string devname_;
                if (cinfo.getDeviceName(_pt->name(), (*it).name, devname_)) {
                    dev->setName(devname_);
                }
                _dev_list.push_back(std::make_pair(dev, lk));
            }
        }
    }
    return lk;
}
bool RoboasmBodyCreator::appendChildLink(BodyPtr _bd, Link *_lk, RoboasmPartsPtr _pt, DevLinkList &_dev_list)
{
    partsPtrList plst;
    _pt->childParts(plst);
    for (auto it = plst.begin(); it != plst.end(); it++) {
        Link *clk = createLink(*it, false, _dev_list);
        if(!!clk) {
            _lk->appendChild(clk);
            _bd->updateLinkTree();
            appendChildLink(_bd, clk, (*it), _dev_list);
        }
    }
    return true;
}
BodyPtr RoboasmBodyCreator::_createBody(RoboasmRobotPtr _rb, const std::string &_name)
{
    _rb->updateDescendants();
    RoboasmPartsPtr root_ = _rb->rootParts();
    if(!root_) {
        ERROR_STREAM(" root not found");
        return nullptr;
    }

    body = new Body();
    if(_name.size() > 0) {
        body->setName(_name);
        body->setModelName(_rb->name());
    } else {
        body->setName(_rb->name());
        body->setModelName(_rb->name());
    }
    DevLinkList dev_list;
    Link *lk = createLink(root_, true, dev_list);
    body->setRootLink(lk);
    body->updateLinkTree();

    appendChildLink(body, lk, root_, dev_list);

    // devices
    if (dev_list.size() > 0) {
        for(auto it = dev_list.begin(); it != dev_list.end(); it++) {
            Device *dev = it->first;
            Link *lk = it->second;
            if (!body->addDevice(dev, lk)) {
                ERROR_STREAM("failed : add device");
            } else {
                DEBUG_STREAM("!!!!!!!!!!!! add device !!!!!!!!!!!!!");
            }
        }
    }
    return body;
}
BodyPtr RoboasmBodyCreator::createBody(RoboasmRobotPtr _rb, MappingPtr _info, const std::string &_name, bool reset_angle)
{
    info = _info;

    body = new Body();
    currentRobot = _rb;

    // set robot-name
    std::string local_name;
    if(_name.size() > 0) {
        local_name = _name;
    } else if (name.size() > 0) {
        local_name = name;
    } else if (!!info) {
        cnoidRAInfo cinfo(info);
        std::string nm_;
        cinfo.getRobotName(nm_);
        if(nm_.size() > 0) {
            local_name = nm_;
        }
    } else {
        local_name = _rb->name();
    }

    if (reset_angle) {
        // reset angle to default
        connectingPointPtrList a_act;
        _rb->inactiveActuators(a_act);

        for(auto it = a_act.begin(); it != a_act.end(); it++) {
            DEBUG_STREAM(" reset: " << (*it)->name());
            (*it)->resetJointAngle();
        }
    }

    //// Create Body
    _rb->updateDescendants();
    _createBody(_rb, local_name);

    if (reset_angle) {
        // revert angle
        cnoidRAInfo cinfo(info);
        connectingPointPtrList a_act;
        _rb->inactiveActuators(a_act);
        for(auto it = a_act.begin(); it != a_act.end(); it++) {
            double ang;
            if (cinfo.getActuatorValue((*it)->name(), "current-angle", ang)) {
                DEBUG_STREAM(" set: " << (*it)->name() << " <= " << ang);
                (*it)->applyJointAngle(ang);
            }
        }
    }

    if (merge_fixed_joint) {
        mergeFixedJoint(body);
    }

    return body;
}
static bool mergeLink(Link *plink, Link *clink)
{
    if(!plink) {
        std::cerr << "plink does not exist" << std::endl;
        return false;
    }
    if(!clink) {
        std::cerr << "clink does not exist" << std::endl;
        return false;
    }
    std::cerr << "p:" << plink->name() << " / " << "c:" << clink->name() << std::endl;
    LinkPtr link_protect(clink);
    std::vector<LinkPtr> all_child;
    {
        Link *cur = clink->child();
        while(!!cur) {
            all_child.push_back(cur);
            cur = cur->sibling();
        }
    }
    //std::cout << "child : " << all_child.size() << std::endl;
    // remove child
    if(!plink->removeChild(clink)) {
        std::cerr << "remove failed : " << clink->name() << std::endl;
        return false;
    }
    // update mass paramter of plink
    {
        double new_mass = plink->mass() + clink->mass();
        coordinates cds_Tb(clink->Tb());
        Vector3 p_c_c = clink->c();
        cds_Tb.transform_vector(p_c_c);
        Vector3 new_c = ((clink->mass() * p_c_c) + (plink->mass() * plink->c()))/new_mass;

        Matrix3 pIc = cds_Tb.rot * clink->I() * cds_Tb.rot.transpose();
        Matrix3 h_c = hat(new_c - p_c_c);
        Matrix3 h_p = hat(new_c - plink->c());

        Matrix3 newI = (pIc - clink->mass() * (h_c * h_c)) +
                       (plink->I() - plink->mass() * (h_p * h_p));
        plink->setInertia(newI);
        plink->setMass(new_mass);
        plink->setCenterOfMass(new_c);
    }
    const Position cTb = clink->Tb();
    {// update visual shape of plink
        SgGroup *vsp = clink->visualShape();
        if(!!vsp) {
            CloneMap cmp;
            vsp = new SgGroup(*vsp, &cmp);
        }
        SgPosTransform *posT = new SgPosTransform();
        posT->addChild(vsp);
        posT->setPosition(cTb);
        plink->addVisualShapeNode(posT);
    }
    {// update collision shape of plink
        SgGroup *csp = clink->collisionShape();
        if(!!csp) {
            CloneMap cmp;
            csp = new SgGroup(*csp, &cmp);
        }
        SgPosTransform *posT = new SgPosTransform();
        posT->addChild(csp);
        posT->setPosition(cTb);
        plink->addCollisionShapeNode(posT);
    }
    // append children of clink
    for(int i = 0; i < all_child.size(); i++) {
        // update offset of children of clink
        Position newTb = cTb * all_child[i]->Tb();
        all_child[i]->setOffsetPosition(newTb);
        // append children of clink to plink
        plink->appendChild(all_child[i]);
    }
    // [TODO] merge Devices
    return true;
}
bool RoboasmBodyCreator::mergeFixedJoint(BodyPtr _bd)
{
    std::cerr << "merge" << std::endl;
    Link *lk = _bd->rootLink();
    bool do_loop = true;
    while(do_loop) {
        Link *clink = nullptr;
        Link *cur = lk->child();
        while(!!cur) {
            std::cerr << "nm: " << cur->name() << " : " << cur->isFixedJoint() << std::endl;
            if(cur->isFixedJoint()) {
                clink = cur;
                break;
            }
            if(!!cur->sibling()) {
                cur = cur->sibling();
            } else {
                if(!!cur->parent()->child()) {
                    cur = cur->parent()->child()->child();
                } else {
                    cur = nullptr;
                }
            }
        }
        if(!clink) {
            break;
        }
        std::cout << "clink : " << clink->name() << std::endl;
        if(!mergeLink(clink->parent(), clink)) {
            return false;
        }
        _bd->updateLinkTree();
    }
    return true;
}

} }
