#include "RobotAssemblerHelper.h"
#include "RobotAssemblerBody.h"
#include "AssemblerManager.h"
// shape
#include <cnoid/SceneLoader>
#include <cnoid/MeshGenerator>
// context menu
#include <cnoid/MenuManager>
// SgBoundingBox
#include <cnoid/SceneEffects>
#include <iostream>

//#define IRSL_DEBUG
#include "irsl_debug.h"

namespace cnoid {
namespace robot_assembler {

static const Vector3f color_default(0.3f, 0.3f, 0.6f);
static const Vector3f color_good0(0.0f, 1.0f, 0.0f);
static const Vector3f color_good1(0.0f, 1.0f, 0.0f);
static const Vector3f color_bad0(1.0f, 0.0f, 0.0f);
static const Vector3f color_bad1(1.0f, 0.0f, 0.0f);
static const Vector3f color_can_connect0(0.0f, 1.0f, 1.0f);
static const Vector3f color_can_connect1(0.0f, 1.0f, 1.0f);
static const Vector3f color_selected(0.5f, 0.0f, 0.5f);

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
    Position p;
    cds->toPosition(p);
    position() = p;

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
    this->addChild(partsScene);

    //partsScene = node;
    Position p; _p->worldcoords().toPosition(p);
    position() = p;
    coordsPtrList lst;
    _p->directDescendants(lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        RoboasmConnectingPointPtr ptr = dynamic_pointer_cast<RoboasmConnectingPoint>(*it);
        if(!!ptr) {
            RASceneConnectingPoint *cp = new RASceneConnectingPoint(ptr);
            //this->addChild(cp);
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
                //this->addChild(cp);
                partsScene->addChild(cp);
                spoint_list.push_back(cp);
            }
        }
    }
}
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
bool RASceneParts::updateColor(Vector3f &_color)
{
    if(!!material) {
        DEBUG_STREAM("update col1 " << this->name() << " / [" << _color[0] << " " << _color[1] << " " << _color[2] << "]");
        material->setDiffuseColor(_color);
        return true;
    }
    return false;
}
RASceneRobot::RASceneRobot(RoboasmRobotPtr _r, AssemblerManager *_ma)
    : SgPosTransform(), self(_r)
{
    // store origignal coords
    coordinates org_coords_ = *_r;
    _r->newcoords();
    _r->updateDescendants();
    //
    manager = _ma;
    setName("RB:" + self->name());
    partsPtrList lst;
    self->allParts(lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        RASceneParts *pt;
        if (!!manager) {
            pt = new RASceneParts(*it, manager->project_directory());
        } else {
            pt = new RASceneParts(*it);
        }
        pt->robot_ptr = this;
        sparts_set.insert(pt);
        this->addChild(pt);
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
    _r->newcoords(org_coords_);
    _r->updateDescendants();
    Position p; org_coords_.toPosition(p);
    setPosition(p);
}
RASceneRobot::~RASceneRobot()
{
    DEBUG_STREAM(self->name());
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
    Position p;
    _rb->clearChildren();//
    for(auto pt_it = _rb->sparts_set.begin(); pt_it != _rb->sparts_set.end(); pt_it++) {
        this->addChild(*pt_it);
        base_coords.transformation(trans, (*pt_it)->parts()->worldcoords());
        trans.toPosition(p);
        (*pt_it)->position() = p;
        sparts_set.insert(*pt_it);
        (*pt_it)->robot_ptr = this;
        for(auto pit = (*pt_it)->spoint_list.begin(); pit != (*pt_it)->spoint_list.end(); pit++) {
            (*pit)->robot_ptr = this;
            spoint_set.insert(*pit);
        }
    }
    return true;
}
//// overrides : SceneWidgetEventHandler
void RASceneRobot::onSceneModeChanged(SceneWidgetEvent* event)
{
    // SgNode should have Operable
    DEBUG_PRINT();
}
bool RASceneRobot::onButtonPressEvent(SceneWidgetEvent* event)
{
    SceneWidgetEvent::EventType tp = event->type();
    DEBUG_STREAM(" Type: " << tp);

    SgNodePath enp = event->nodePath();
#if 0
    DEBUG_STREAM(" event->nodePath() : " << enp.size());
    for (int i = 0 ; i < enp.size(); i++) {
        SgNode *ptr = enp[i];
        DEBUG_STREAM(" ---");
        DEBUG_STREAM(" " << static_cast<void *> (ptr));
        DEBUG_STREAM(" name: " << ptr->name());
        DEBUG_STREAM(" class: " << ptr->className());
        DEBUG_STREAM(" attr: " << ptr->attributes());
        if (ptr->hasUri()) {
            DEBUG_STREAM( " uri: " << ptr->uri());
        }
        if (ptr->hasAbsoluteUri()) {
            DEBUG_STREAM( " abs_uri: " << ptr->absoluteUri());
        }
        if (ptr->hasParents()) {
            int j = 0;
            for(auto it = ptr->parentBegin(); it != ptr->parentEnd(); it++, j++) {
                DEBUG_STREAM(" p" << j << " : " << static_cast<void *>(*it));
            }
        }
    }
#endif
    RASceneParts *pt_ = nullptr;
    RASceneConnectingPoint *cp_ = nullptr;
    for (int i = 0 ; i < enp.size(); i++) {
        SgNode *ptr = enp[i];
        if(!pt_) pt_ = dynamic_cast<RASceneParts *>(ptr);
        if(!cp_) cp_ = dynamic_cast<RASceneConnectingPoint *>(ptr);
        if(!!pt_ && !!cp_) break;
    }
    lastClickedParts = nullptr;
    lastClickedPoint = nullptr;
    if(!!cp_) {
        DEBUG_STREAM( " PointClicked : " << cp_->name());
        lastClickedPoint = cp_;
    } else if (!!pt_) {
        DEBUG_STREAM( " PartsClicked : " << pt_->name());
        lastClickedParts = pt_;
    } else {
        DEBUG_STREAM( " ---unknown state---");
    }

    // check clicked button
    switch(tp) {
    case SceneWidgetEvent::ButtonPress:
    {
        int bt = event->button();
        switch(bt) {
        case Qt::LeftButton:
            DEBUG_STREAM(" Left");
            break;
        case Qt::RightButton:
            DEBUG_STREAM(" Right");
            return false;
            break;
        case Qt::MiddleButton:
            DEBUG_STREAM(" Middle");
            return false;
            break;
        default:
            DEBUG_STREAM("invalid button / Type: " << tp << ", Button: " << bt);
            break;
        }
    }
    break;
    default:
        DEBUG_STREAM("invalid type / Type: " << tp << ", Button: " << event->button());
        break;
    }

    if(!!lastClickedPoint) {
        manager->pointClicked(cp_);
    } else if (!!lastClickedParts) {
        manager->partsClicked(pt_);
    }
    return false;
    //if return true, handling events after this function may not occurred
}
bool RASceneRobot::onDoubleClickEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    // disable default behavior / default: double click -> toggle edit-mode
    return true;
}
#if 0
bool RASceneRobot::onButtonReleaseEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    return false;
}
bool RASceneRobot::onPointerMoveEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    return false;
}
void RASceneRobot::onPointerLeaveEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
}
bool RASceneRobot::onKeyPressEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    return false;
}
bool RASceneRobot::onKeyReleaseEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    return false;
}
bool RASceneRobot::onScrollEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    return false;
}
#endif
void RASceneRobot::onFocusChanged(SceneWidgetEvent* event, bool on)
{
    // may call when mode was changed
    DEBUG_PRINT();
}
bool RASceneRobot::onContextMenuRequest(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    if (!manager || !(manager->isAssembler())) {
        return false;
    }
    auto menu = event->contextMenu();

    menu->addSeparator();
    menu->addItem("Move")->sigTriggered().connect(
        [this](){ } );

    if (!!lastClickedParts) {
        menu->addSeparator();

        std::string label0_ = "Save history : " + this->name();
        menu->addItem(label0_)->sigTriggered().connect(
            [this](){ manager->save_history(this); });
        std::string label1_ = "Save model : " + this->name();
        menu->addItem(label1_)->sigTriggered().connect(
            [this](){ manager->save_model(this); });
        //
        menu->addSeparator();
        std::string label2_ = "Delete Robot: " + this->name();
        menu->addItem(label2_)->sigTriggered().connect(
            [this](){ manager->deleteRobot(this); });
        //
        menu->addSeparator();
        std::string label3_ = "Select Parts : " + lastClickedParts->name();
        menu->addItem(label3_);
        std::string label4_ = "Detach Parts : " + lastClickedParts->name();
        menu->addItem(label4_)->sigTriggered().connect(
            [this](){ manager->detachSceneRobot(lastClickedParts); });
    }
    if (!!lastClickedPoint) {
        menu->addSeparator();
        std::string label_ = "Select Point : ";
        label_ += lastClickedPoint->name();
        menu->addItem(label_);
    }
    //menu->addSeparator();
    //menu->addItem("Attach")->sigTriggered().connect(
    //[this](){ manager->com_attach(); } );

    return false; //??
}

} }
