#include "RobotAssemblerHelper.h"
#include "AssemblerManager.h"
// context menu
#include <cnoid/MenuManager>
#include <cnoid/SceneCameras>

//#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;
using namespace cnoid::robot_assembler;

class RASceneRobotBase::Impl
{
public:
    Impl() {};
    ~Impl() {};

public:
    int pre_x;
    int pre_y;
};

RASceneRobotBase::RASceneRobotBase(RoboasmRobotPtr _r, AssemblerManager *_ma)
    : RASceneRobot(_r)
{
    impl = new RASceneRobotBase::Impl();
    impl->pre_x = -1;
    impl->pre_y = -1;
    manager = _ma;
    if (!!manager) {
        initializeRobotStructure(manager->project_directory());
    }
}
RASceneRobotBase::~RASceneRobotBase()
{
    DEBUG_STREAM(self->name());
    delete impl;
}
//// overrides : SceneWidgetEventHandler
void RASceneRobotBase::onSceneModeChanged(SceneWidgetEvent* event)
{
    // SgNode should have Operable
    DEBUG_PRINT();
}
bool RASceneRobotBase::onButtonPressEvent(SceneWidgetEvent* event)
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

    manager->selectRobot(this);
    if(!!lastClickedPoint) {
        manager->pointClicked(cp_);
    } else if (!!lastClickedParts) {
        manager->partsClicked(pt_);
    }

    return false;
    //if return true, handling events after this function may not occurred
}
bool RASceneRobotBase::onDoubleClickEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    // disable default behavior / default: double click -> toggle edit-mode
    return true;
}
bool RASceneRobotBase::onPointerMoveEvent(SceneWidgetEvent* event)
{
    //DEBUG_PRINT();
    if (event->modifiers() & Qt::AltModifier) {
        //DEBUG_STREAM(" mod:alt");
        //DEBUG_STREAM(" x: " << event->x() << " y: " << event->y());
        //DEBUG_STREAM(" ratio : " << event->pixelSizeRatio());
        if (impl->pre_x > 0 || impl->pre_y > 0) {
            //DEBUG_STREAM(" diff_x: " << impl->pre_x - event->x() << " diff_y: " << impl->pre_y - event->y());
            //event->sceneWidget()
            int diffx = impl->pre_x - event->x();
            int diffy = impl->pre_y - event->y();
            if (diffx == 0 && diffy == 0) {
                // not detecting movement
                return true;
            }
            coordinates cam_pos(event->cameraPosition());
            const SgPerspectiveCamera *cam = dynamic_cast<const SgPerspectiveCamera *>(event->camera());
            if (!!cam) {
                coordinates cds(robot()->worldcoords());
                if (event->modifiers() & Qt::ShiftModifier) {
                    Vector3 ax(diffy, -diffx, 0);
                    double norm = ax.norm();
                    double ang = norm / 160;// 1 radian / 160 pixel
                    ax /= norm;
                    cds.rotate(ang, ax, cam_pos);
                    setCoords(cds);
                    notifyUpdate(SgUpdate::MODIFIED);
                } else {
                    Vector3 trs(diffx, diffy, 0);
                    trs *= -1.0 / event->pixelSizeRatio();
                    cds.translate(trs, cam_pos);
                    setCoords(cds);
                    notifyUpdate(SgUpdate::MODIFIED);
                }
            }
        }
        impl->pre_x = event->x();
        impl->pre_y = event->y();
    } else {
        // end dragging
        impl->pre_x = -1;
        impl->pre_y = -1;
    }
    //return false;
    return true;
}
void RASceneRobotBase::onPointerLeaveEvent(SceneWidgetEvent* event)
{
    //DEBUG_PRINT();
    // end dragging
    impl->pre_x = -1;
    impl->pre_y = -1;
}
#if 0
bool RASceneRobotBase::onButtonReleaseEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    return false;
}
bool RASceneRobotBase::onKeyPressEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    return false;
}
bool RASceneRobotBase::onKeyReleaseEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    return false;
}
bool RASceneRobotBase::onScrollEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    return false;
}
#endif
void RASceneRobotBase::onFocusChanged(SceneWidgetEvent* event, bool on)
{
    // may call when mode was changed
    DEBUG_PRINT();
}
bool RASceneRobotBase::onContextMenuRequest(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    if (!manager || !(manager->isAssembler())) {
        return false;
    }
    auto menu = event->contextMenu();

    menu->addSeparator();
    {
#if 0
        std::string label = "Move : " + this->name();
        menu->addItem(label)->sigTriggered().connect(
            [this](){ } );
#endif
        //
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
    }

    if (!!lastClickedParts) {
        menu->addSeparator();
        //
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
