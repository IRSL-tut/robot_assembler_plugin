#include "RobotAssemblerHelper.h"
#include "RobotAssemblerBody.h"
#include "AssemblerManager.h"
// shape
#include <cnoid/SceneLoader>
#include <cnoid/MeshGenerator>
// context menu
#include <cnoid/MenuManager>

#include <iostream>

#define IRSL_DEBUG
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

static void createShapeConnectingPoint(SgPosTransform *_root, SgMaterialPtr &_res_material,
                                       SgSwitchableGroupPtr &_res_switch)
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
#define SCP_LENGTH_LONG  0.014
#define SCP_LENGTH_SHORT 0.007
#define SCP_WIDTH 0.004
    SgSwitchableGroupPtr sw_g(new SgSwitchableGroup());
    sw_g->setName(name_ + "/switch");
    MeshGenerator mg;
    {
        SgShapePtr shape(new SgShape());
        shape->setName(name_ + "/x");
        SgMeshPtr mesh = mg.generateBox(Vector3(SCP_LENGTH_SHORT, SCP_WIDTH, SCP_WIDTH));
        shape->setMesh(mesh);
        shape->setMaterial(material);
        SgPosTransformPtr pt = new SgPosTransform();
        pt->position().translation() = Vector3(SCP_LENGTH_SHORT/2 + SCP_WIDTH/2, 0, 0);
        pt->addChild(shape);
        sw_g->addChild(pt);
    }
    {
        SgShapePtr shape(new SgShape());
        shape->setName(name_ + "/y");
        SgMeshPtr mesh = mg.generateBox(Vector3(SCP_WIDTH, SCP_LENGTH_SHORT, SCP_WIDTH));
        shape->setMesh(mesh);
        shape->setMaterial(material);
        SgPosTransformPtr pt = new SgPosTransform();
        pt->position().translation() = Vector3(0, SCP_LENGTH_SHORT/2 + SCP_WIDTH/2, 0);
        pt->addChild(shape);
        sw_g->addChild(pt);
    }
    {
        SgShapePtr shape(new SgShape());
        shape->setName(name_ + "/z");
        SgMeshPtr mesh = mg.generateBox(Vector3(SCP_WIDTH, SCP_WIDTH, SCP_LENGTH_LONG));
        shape->setMesh(mesh);
        shape->setMaterial(material);
        SgPosTransformPtr pt = new SgPosTransform();
        pt->position().translation() = Vector3(0, 0, SCP_LENGTH_LONG/2 - SCP_WIDTH/2);
        pt->addChild(shape);
        sw_g->addChild(pt);
    }
    _root->addChild(sw_g);

    _res_material = material;
    _res_switch = sw_g;
}

RASceneConnectingPoint::RASceneConnectingPoint(RoboasmConnectingPointPtr _c)
    : SgPosTransform(), self(_c), current_state(DEFAULT)
{
    setName("CP:" + self->name());
    SgMaterialPtr mat_;//todo
    SgSwitchableGroupPtr sw_;//todo
    createShapeConnectingPoint(this, mat_, sw_);

    coordinates *cds = static_cast<coordinates *>(self.get());
    Position p;
    cds->toPosition(p);
    position() = p;

    material = mat_;
    switch_node = sw_;
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
    createSceneFromGeometry(this, self->info->visual, _proj_dir);
    //partsScene = node;

    coordsPtrList lst;
    _p->directDescendants(lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        RoboasmConnectingPointPtr ptr = dynamic_pointer_cast<RoboasmConnectingPoint>(*it);
        if(!!ptr) {
            RASceneConnectingPoint *cp = new RASceneConnectingPoint(ptr);
            this->addChild(cp);
            spoint_list.push_back(cp);
        }
    }
}
RASceneParts::~RASceneParts()
{
    DEBUG_STREAM(self->name());
}
RASceneRobot::RASceneRobot(RoboasmRobotPtr _r, AssemblerManager *_ma)
    : SgPosTransform(), self(_r)
{
    manager = _ma;
    setName("RB:" + self->name());
    partsPtrList lst;
    self->allParts(lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        RASceneParts *pt;
        if (!!manager) {
            DEBUG_STREAM( " path: ");
            DEBUG_STREAM( " path: " << manager->project_directory());
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
    if(lst.size() == 1) {
        AttachHistoryItem itm;
        itm.initial_parts = true;
        itm.parts_name = lst[0]->name();
        itm.parts_type = lst[0]->info->type;
        history.push_back(itm);
    } else if (lst.size() > 1) {
        lst[0]->dumpConnectFromParent(history);
    } else {
        //
    }
}
RASceneRobot::~RASceneRobot()
{
    DEBUG_STREAM(self->name());
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

    SgNodePath pt = event->nodePath();
#if 0
    DEBUG_STREAM(" event->nodePath() : " << pt.size());
    for (int i = 0 ; i < pt.size(); i++) {
        SgNode *ptr = pt[i];
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
    for (int i = 0 ; i < pt.size(); i++) {
        SgNode *ptr = pt[i];
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

    auto menu = event->contextMenu();

    menu->addSeparator();
    menu->addItem("Move")->sigTriggered().connect(
        [this](){ } );

    if (!!lastClickedParts) {
        menu->addSeparator();
        std::string label_ = "Delete This: ";
        label_ += this->name();
        menu->addItem(label_)->sigTriggered().connect(
            [this](){ manager->deleteRobot(this); });
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
