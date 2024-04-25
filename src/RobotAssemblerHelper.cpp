#include "RobotAssemblerHelper.h"
#include "RobotAssemblerBody.h"
// shape
#include <cnoid/SceneLoader>
#include <cnoid/MeshGenerator>
// SgBoundingBox
#include <cnoid/SceneEffects>
#include <iostream>

//#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;
using namespace cnoid::robot_assembler;

namespace
{
static const Vector3f color_default(0.3f, 0.3f, 0.6f);
static const Vector3f color_good0(0.33f, 1.0f, 0.0f);
//static const Vector3f color_good1(0.0f, 1.0f, 0.33f);//
static const Vector3f color_good1(0.0f, 0.5f, 0.166f);
static const Vector3f color_bad0(1.0f, 0.0f, 0.0f);
//static const Vector3f color_bad1(1.0f, 0.0f, 0.0f);
static const Vector3f color_bad1(0.5f, 0.0f, 0.166f);
static const Vector3f color_can_connect0(0.0f, 1.0f, 1.0f);
static const Vector3f color_can_connect1(0.0f, 1.0f, 1.0f);
static const Vector3f color_selected(0.5f, 0.0f, 0.5f);
}

#define SCP_LENGTH_LONG  0.015
#define SCP_LENGTH_SHORT 0.006
#define SCP_WIDTH 0.003
//    Vector3(SCP_LENGTH_SHORT, SCP_WIDTH, SCP_WIDTH)
static inline SgPosTransformPtr genBox(const std::string &_nm, const Vector3 &_size, const Vector3 &_trans, SgMaterialPtr &_mat)
{
    MeshGenerator mg;

    SgShapePtr shape(new SgShape());
    shape->setName(_nm);
    SgMeshPtr mesh = mg.generateBox(_size);
    shape->setMesh(mesh);
    shape->setMaterial(_mat);
    SgPosTransformPtr pt = new SgPosTransform();
    pt->position().translation() = _trans;
    pt->addChild(shape);
    return pt;
}
static inline SgPosTransformPtr genCylinder(const std::string &_nm, double _radius, double _height, const Isometry3 &_position, SgMaterialPtr &_mat)
{
    MeshGenerator mg;
    SgShapePtr shape(new SgShape());
    shape->setName(_nm);
    SgMeshPtr mesh = mg.generateCylinder(_radius, _height);
    shape->setMesh(mesh);
    shape->setMaterial(_mat);
    SgPosTransformPtr pt = new SgPosTransform();
    pt->position() = _position;
    pt->addChild(shape);
    return pt;
}
static void createShapeConnectingPoint(SgPosTransform *_root, SgMaterialPtr &_res_material,
                                       SgSwitchableGroupPtr &_res_switch, SgScaleTransformPtr &_res_scl,
                                       Vector3 _axis = Vector3::Zero())
{
    // create shape
    const std::string &name_ = _root->name();
    // material ???
    SgMaterialPtr material(new SgMaterial());
    material->setName(name_ + "/common_material");

    material->setDiffuseColor(color_default);
    material->setEmissiveColor(Vector3f(0.0f, 0.0f, 0.0f));
    material->setSpecularColor(Vector3f(0.0f, 0.0f, 0.0f));
    material->setAmbientIntensity(0.7f);

    SgSwitchableGroupPtr sw_g(new SgSwitchableGroup());
    sw_g->setName(name_ + "/switch");
    {// x-axis
        SgPosTransformPtr pt = genBox(name_ + "/x",
                                      Vector3(SCP_LENGTH_SHORT, SCP_WIDTH, SCP_WIDTH),
                                      Vector3(SCP_LENGTH_SHORT/2 + SCP_WIDTH/2, 0, 0),
                                      material);
        sw_g->addChild(pt);
    }
    {// y-axis
        SgPosTransformPtr pt = genBox(name_ + "/y",
                                      Vector3(SCP_WIDTH, SCP_LENGTH_SHORT, SCP_WIDTH),
                                      Vector3(0, SCP_LENGTH_SHORT/2 + SCP_WIDTH/2, 0),
                                      material);
        sw_g->addChild(pt);
    }
    {// z-axis
        SgPosTransformPtr pt = genBox(name_ + "/z",
                                      Vector3(SCP_WIDTH, SCP_WIDTH, SCP_LENGTH_LONG),
                                      Vector3(0, 0, SCP_LENGTH_LONG/2 - SCP_WIDTH/2),
                                      material);
        sw_g->addChild(pt);
    }
    if (!_axis.isZero()) { // valid axis
        //// [todo] update shape
        Isometry3 pos_;
        pos_.setIdentity();
        pos_.translation() = SCP_LENGTH_SHORT * _axis;
        SgPosTransformPtr pt = genCylinder(name_ + "/axis",
                                           SCP_WIDTH, SCP_WIDTH,
                                           pos_,
                                           material);
        sw_g->addChild(pt);
    }
    _res_scl = new SgScaleTransform(0.5);
    _res_scl->addChild(sw_g);
    _root->addChild(_res_scl);

    _res_material = material;
    _res_switch = sw_g;
}
static SgMaterial *searchMaterial(SgNode *_node)
{
    if (!_node) return nullptr;
    int id_ = SgNode::findClassId<SgShape>();
    for(int i = 0; i < _node->numChildObjects(); i++) {
        SgObject *obj = _node->childObject(i);
        if(obj->isNode()) {
            SgNode *nd_ = obj->toNode();
            if(!nd_) continue;
            if (id_ == nd_->classId()) {
                SgShape *sp_ = dynamic_cast<SgShape *>(nd_);
                return sp_->material();
            }
            SgMaterial *res = searchMaterial(nd_);
            if (!!res) {
                return res;
            }
        }
    }
    return nullptr;
}

//
// RASceneConnectingPoint
//
RASceneConnectingPoint::RASceneConnectingPoint(RoboasmConnectingPointPtr _c)
    : SgPosTransform(), self(_c), current_state(DEFAULT)
{
    setName("CP:" + self->name());
    SgMaterialPtr mat_;
    SgSwitchableGroupPtr sw_;
    SgScaleTransformPtr scl_;
    if (self->isActuator()) {
        DEBUG_STREAM(" actuator : " << self->name());
        Actuator *ainfo_ = dynamic_cast<Actuator *>(self->info);
        createShapeConnectingPoint(this, mat_, sw_, scl_, ainfo_->axis);
    } else {
        createShapeConnectingPoint(this, mat_, sw_, scl_);
    }

    coordinates *cds = static_cast<coordinates *>(self.get());
    cds->toPosition(this->position());

    material = mat_;
    switch_node = sw_;
    scale_node = scl_;
}
RASceneConnectingPoint::~RASceneConnectingPoint()
{
    DEBUG_STREAM(self->name());
}
void RASceneConnectingPoint::changeState(RASceneConnectingPoint::Clicked _clk)
{
    if (current_state == _clk) return;
    current_state = _clk;
    switch(_clk) {
    case DEFAULT:
    {
        material->setDiffuseColor(color_default);
    }
    break;
    case SELECT_GOOD0:
    {
        material->setDiffuseColor(color_good0);
    }
    break;
    case SELECT_GOOD1:
    {
        material->setDiffuseColor(color_good1);
    }
    break;
    case SELECT_BAD0:
    {
        material->setDiffuseColor(color_bad0);
    }
    break;
    case SELECT_BAD1:
    {
        material->setDiffuseColor(color_bad1);
    }
    break;
    case CAN_CONNECT0:
    {
        material->setDiffuseColor(color_can_connect0);
    }
    break;
    case CAN_CONNECT1:
    {
        material->setDiffuseColor(color_can_connect1);
    }
    break;
    }
}
//
// RASceneParts
//
RASceneParts::RASceneParts(RoboasmPartsPtr _p, const std::string &_proj_dir)
    : SgPosTransform(), self(_p), partsScene(nullptr)
{
    setName("PT:" + self->name());
    //  ON: this -> effect-> parts
    // OFF: tihs -> parts
    partsScene = new SgGroup();
    createSceneFromGeometry(partsScene, self->info->visual, _proj_dir, _p->color);
    this->material = searchMaterial(partsScene);
    bbEffect = new SgBoundingBox();
    bbEffect->setColor(Vector3f(1.0f, 1.0f, 0.0f));
    bbEffect->setLineWidth(-1.0f);
    this->addChild(partsScene); // parts->connectingpoint (SgPosTrans)

    //partsScene = node;
    _p->worldcoords().toPosition(this->position());

    coordsPtrList lst;
    _p->directDescendants(lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        RoboasmConnectingPointPtr ptr = dynamic_pointer_cast<RoboasmConnectingPoint>(*it);
        if(!!ptr) {
            RASceneConnectingPoint *cp = new RASceneConnectingPoint(ptr);
            partsScene->addChild(cp);
            spoint_list.push_back(cp);
        }
    }
    // for making parts already connected
    if(_p->hasParent() && _p->parent()->isConnectingPoint()) {
        RoboasmCoords *ra_p = _p->parent();
        if (ra_p->hasParent()) {
            RoboasmCoordsPtr cds_ptr = ra_p->parent()->isDirectDescendant(ra_p);
            RoboasmConnectingPointPtr ptr = dynamic_pointer_cast<RoboasmConnectingPoint>(cds_ptr);
            if(!!ptr) {
                RASceneConnectingPoint *cp = new RASceneConnectingPoint(ptr);
                // invert position
                coordinates newtrans;
                _p->worldcoords().transformation(newtrans, ptr->worldcoords());
                newtrans.toPosition(cp->position());
                // add color
                Vector3f col(1.0f, 1.0f, 0);
                cp->material->setDiffuseColor(col);
                partsScene->addChild(cp);
                spoint_list.push_back(cp);
            }
        }
    }
}
//RASceneParts::RASceneParts() { std::cerr << "BAD usage of RASceneParts" << std::endl; }
RASceneParts::~RASceneParts()
{
    DEBUG_STREAM(self->name());
}
void RASceneParts::drawBoundingBox(bool _on)
{
    if(_on) {
        DEBUG_STREAM(" on " << this->name());
        this->clearChildren();
        bbEffect->clearChildren();
        bbEffect->addChild(partsScene);
        this->addChild(bbEffect);
    } else { // off
        DEBUG_STREAM(" off " << this->name());
        this->clearChildren();
        bbEffect->clearChildren();
        this->addChild(partsScene);
    }
}
bool RASceneParts::updateColor(const Vector3f &_color)
{
    if(!!material) {
        DEBUG_STREAM("update col1 " << this->name() << " / [" << _color[0] << " " << _color[1] << " " << _color[2] << "]");
        material->setDiffuseColor(_color);
        return true;
    }
    return false;
}
void RASceneParts::updateCoords()
{
    coordinates robot_to_parts;
    robot_ptr->robot()->worldcoords().transformation(robot_to_parts, self->worldcoords());
    robot_to_parts.toPosition(this->position());
    // update coords of connecting-points
    for(auto it = spoint_list.begin(); it != spoint_list.end(); it++) {
        (*it)->updateCoords();
    }
}
//
// RASceneRobot
//
RASceneRobot::RASceneRobot(RoboasmRobotPtr _r)
    : SgPosTransform(), self(_r)
{
}
RASceneRobot::RASceneRobot(RoboasmRobotPtr _r, const std::string &_project_dir)
    : RASceneRobot(_r)
{
    initializeRobotStructure(_project_dir);
}
RASceneRobot::~RASceneRobot()
{
    DEBUG_STREAM(self->name());
}
void RASceneRobot::initializeRobotStructure(const std::string &_project_dir)
{
    // store origignal coords
    coordinates org_coords_ = *self;
    self->newcoords();
    self->updateDescendants();
    //
    //manager = _ma;
    setName("RB:" + self->name());
    partsPtrList lst;
    self->allParts(lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        RASceneParts *pt;
        pt = new RASceneParts(*it, _project_dir);
        pt->robot_ptr = this;
        sparts_set.insert(pt);
        this->addChild(pt); // robot->parts(SgPosTrans)
        for(int i = 0; i < pt->spoint_list.size(); i++) {
            pt->spoint_list[i]->robot_ptr = this;
            spoint_set.insert(pt->spoint_list[i]);
        }
    }
    if(lst.size() == 1) { // make robot with single parts
        AttachHistoryItem itm;
        itm.initial_parts = true;
        itm.parts_name = lst[0]->name();
        itm.parts_type = lst[0]->info->type;
        history.push_back(itm);
    } else if (lst.size() > 1) { // assembled robot
        lst[0]->dumpConnectionFromParent(history);
    } else {
        //
    }
    // restore original coords
    self->newcoords(org_coords_);
    self->updateDescendants();
    org_coords_.toPosition(this->position());
}
RASceneParts *RASceneRobot::searchParts(RoboasmPartsPtr _pt)
{
    RASceneParts *ret = nullptr;
    for(auto it = sparts_set.begin(); it != sparts_set.end(); it++) {
        if((*it)->self == _pt) {
            ret = *it;
            break;
        }
    }
    return ret;
}
RASceneConnectingPoint *RASceneRobot::searchConnectingPoint(RoboasmConnectingPointPtr _pt)
{
    RASceneConnectingPoint *ret = nullptr;
    for(auto it = spoint_set.begin(); it != spoint_set.end(); it++) {
        if((*it)->self == _pt) {
            ret = *it;
            break;
        }
    }
    return ret;
}
bool RASceneRobot::mergeRobot(RASceneRobot *_rb) {
    coordinates base_coords = self->worldcoords();
    coordinates trans;
    _rb->clearChildren();//
    for(auto pt_it = _rb->sparts_set.begin(); pt_it != _rb->sparts_set.end(); pt_it++) {
        this->addChild(*pt_it);
        base_coords.transformation(trans, (*pt_it)->parts()->worldcoords());
        trans.toPosition((*pt_it)->position());
        sparts_set.insert(*pt_it);
        (*pt_it)->robot_ptr = this;
        for(auto pit = (*pt_it)->spoint_list.begin(); pit != (*pt_it)->spoint_list.end(); pit++) {
            (*pit)->robot_ptr = this;
            spoint_set.insert(*pit);
        }
    }
    return true;
}
