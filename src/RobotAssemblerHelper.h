#ifndef CNOID_ROBOT_ASSEMBLER_HELPER_H
#define CNOID_ROBOT_ASSEMBLER_HELPER_H

#include "RobotAssembler.h"
#include <cnoid/SceneGraph>
#include <cnoid/SceneWidgetEventHandler>
#include <cnoid/Body>
#include <cnoid/ValueTree>
#include <set>

namespace cnoid {
class AssemblerManager;

namespace robot_assembler {
class RASceneConnectingPoint;
class RASceneParts;
class RASceneRobot;

typedef SgPosTransform RASceneBase;

class RASceneConnectingPoint : public RASceneBase
{
public:
    enum Clicked {
        DEFAULT,
        SELECT_GOOD0,
        SELECT_GOOD1,
        SELECT_BAD0,
        SELECT_BAD1,
        CAN_CONNECT0,
        CAN_CONNECT1,
        CAN_CONNECT,
        SELECTED,
        NOT_CONNECT
    };
    RASceneConnectingPoint() = delete;
    RASceneConnectingPoint(RoboasmConnectingPointPtr _c);
    ~RASceneConnectingPoint();
    //~RASceneConnectingPoint();
    RoboasmConnectingPointPtr point() { return self; }
    void switchOn(bool on) {  if (!!switch_node) switch_node->setTurnedOn(on); }
    void setScale(double _scl) { if (!!scale_node) scale_node->setScale(_scl); }
    void changeState(Clicked _clk = DEFAULT);
    RASceneRobot *scene_robot() { return robot_ptr; }

    void updateCoords() {}
protected:
    RASceneRobot *robot_ptr;
    RoboasmConnectingPointPtr self;
    SgMaterialPtr material;
    SgSwitchableGroupPtr switch_node;
    SgScaleTransformPtr scale_node;
    Clicked current_state;

    friend RASceneParts;
    friend RASceneRobot;
};
typedef ref_ptr<RASceneConnectingPoint> RASceneConnectingPointPtr;

class RASceneParts : public RASceneBase
{
public:
    RASceneParts() = delete;
    RASceneParts(RoboasmPartsPtr _p, const std::string &_proj_dir = std::string());
    ~RASceneParts();
    //~RASceneParts();
    RoboasmPartsPtr parts() { return self; }
    RASceneRobot *scene_robot() { return robot_ptr; }
    void drawBoundingBox(bool _on = true);
    bool updateColor(Vector3f &_color);
    void updateCoords();
protected:
    RASceneRobot *robot_ptr;
    RoboasmPartsPtr self;
    SgGroupPtr partsScene;
    SgBoundingBoxPtr bbEffect;
    std::vector<RASceneConnectingPoint*> spoint_list;
    SgMaterial *material;
    friend RASceneConnectingPoint;
    friend RASceneRobot;
};
typedef ref_ptr<RASceneParts> RAScenePartsPtr;

class RASceneRobot : public RASceneBase, public SceneWidgetEventHandler
{
public:
    RASceneRobot() = delete;
    RASceneRobot(RoboasmRobotPtr _r, AssemblerManager *_ma = nullptr);
    ~RASceneRobot();

    RoboasmRobotPtr robot() { return self; }
    void updateFromSelf()
    {   // [TODO] concern initial
        Position p; self->toPosition(p);
        this->position() = p;
    }
    void setCoords(coordinates &_coords)
    {
        self->newcoords(_coords);
        self->updateDescendants();
        updateFromSelf();
    }
    void updateStructure()
    {
        self->updateDescendants();
        for(auto it = sparts_set.begin(); it != sparts_set.end(); it++) {
            (*it)->updateCoords();
        }
    }
    void setInitialCoords(coordinates &_coords)
    {
        // [TODO]
    }
    void updateByInfo()
    {
        // coords update
        // color update
    }
    RASceneParts *searchParts(RoboasmPartsPtr _pt);
    RASceneConnectingPoint *searchConnectingPoint(RoboasmConnectingPointPtr _pt);
    bool mergeRobot(RASceneRobot *_rb);
#if 0
    void debug() {
        partsPtrList plst;
        self->allParts(plst);
        if(plst.size() != sparts_set.size()) {
            DEBUG_STREAM(" Roboasm: " << plst.size() << " != Scene: " << sparts_set.size());
        }
        for(auto it = plst.begin(); it != plst.end(); it++) {
            bool exist = false;
            coordinates scoords;
            for(auto sit = sparts_set.begin(); sit != sparts_set.end(); sit++) {
                if((*it) == (*sit)->parts()) {
                    scoords = (*sit)->position();
                    exist = true;
                    break;
                }
            }
            if(!exist) {
                DEBUG_STREAM_NL(" Roboasm: " << (*it)->name() << " not in scene" << std::endl);
            } else {
                coordinates rcoords;
                self->worldcoords().transformation(rcoords, (*it)->worldcoords());
                if(!rcoords.equal(scoords)) {
                    DEBUG_STREAM_NL(" coords not equal r: " << (*it)->name() << std::endl);
                    //std::cout << "r: " << rcoords << ", s: " << scoords << std::endl;
                }
            }
        }
        for(auto sit = sparts_set.begin(); sit != sparts_set.end(); sit++) {
            bool exist = false;
            for(auto it = plst.begin(); it != plst.end(); it++) {
                if((*it) == (*sit)->parts()) {
                    exist = true;
                    break;
                }
            }
            if(!exist) {
                DEBUG_STREAM_NL(" Scene: " << (*sit)->name() << " not in roboasm" << std::endl);
            }
        }
    }
#endif

    //// overrides : SceneWidgetEventHandler
    virtual void onSceneModeChanged(SceneWidgetEvent* event) override;
    virtual bool onButtonPressEvent(SceneWidgetEvent* event) override;
    virtual bool onDoubleClickEvent(SceneWidgetEvent* event) override;
#if 0
    virtual bool onButtonReleaseEvent(SceneWidgetEvent* event) override;
    virtual bool onPointerMoveEvent(SceneWidgetEvent* event) override;
    virtual void onPointerLeaveEvent(SceneWidgetEvent* event) override;
    virtual bool onKeyPressEvent(SceneWidgetEvent* event) override;
    virtual bool onKeyReleaseEvent(SceneWidgetEvent* event) override;
    virtual bool onScrollEvent(SceneWidgetEvent* event) override;
#endif
    virtual void onFocusChanged(SceneWidgetEvent* event, bool on) override;
    virtual bool onContextMenuRequest(SceneWidgetEvent* event) override;
    void attachHistory(AttachHistory &_hist,
                       const std::string &_parent,
                       const std::string &_parent_point,
                       const std::string &_parts_name,
                       const std::string &_parts_type,
                       const std::string &_parts_point,
                       coordinates &_coords)
    {
        _attachHistory(_hist, _parent, _parent_point, _parts_name, _parts_type, _parts_point);
        _hist[0].connecting_offset = _coords;
        for(auto it = _hist.begin(); it != _hist.end(); it++) {
            history.push_back(*it);
        }
    }
    std::set<RASceneParts*> sparts_set;
    std::set<RASceneConnectingPoint*> spoint_set;
    AttachHistory history;
    MappingPtr info;
    // AssembleConfig -> info
protected:
    bool _attachHistory(AttachHistory &_hist,
                        const std::string &_parent,
                        const std::string &_parent_point,
                        const std::string &_parts_name,
                        const std::string &_parts_type,
                        const std::string &_parts_point)
    {
        if(_hist.size() < 1) return false;
        _hist[0].parts_name  = _parts_name;
        _hist[0].parts_type  = _parts_type;
        _hist[0].parts_point = _parts_point;
        _hist[0].parent      = _parent;
        _hist[0].parent_point = _parent_point;
        _hist[0].initial_parts = false;
        return true;
    }
    RoboasmRobotPtr self;
    cnoid::AssemblerManager *manager;
    RASceneParts *lastClickedParts;
    RASceneConnectingPoint *lastClickedPoint;

    friend RASceneConnectingPoint;
    friend RASceneParts;
    friend cnoid::AssemblerManager;
};
typedef ref_ptr<RASceneRobot> RASceneRobotPtr;

// SgPosTransform -> SgPosTransform(partsScene) -> SgShape (parts) -> material(???)
//                -> SgPosTransform -> SgShepe (connecting-point) -> SgMaterial (from list???
//                                                                -> SgSwitch
// notifyUpdate(SgUpdate::REMOVED | SgUpdate::ADDED | SgUpdate::MODIFIED); / on scene graph
 
} }

#endif
