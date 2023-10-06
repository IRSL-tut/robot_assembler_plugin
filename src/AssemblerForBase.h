#ifndef CNOID_ASSEMBLER_FOR_BASE_H
#define CNOID_ASSEMBLER_FOR_BASE_H

#include "RobotAssemblerHelper.h"
#include <cnoid/SceneWidgetEventHandler>

#include "exportdecl.h"

namespace cnoid {
class AssemblerManager;

namespace robot_assembler {

class CNOID_EXPORT RASceneRobotBase : public RASceneRobot, public SceneWidgetEventHandler
{
public:
    RASceneRobotBase() = delete;
    RASceneRobotBase(RoboasmRobotPtr _r, AssemblerManager *_ma = nullptr);
    ~RASceneRobotBase();

    //// overrides : SceneWidgetEventHandler
    virtual void onSceneModeChanged(SceneWidgetEvent* event) override;
    virtual bool onButtonPressEvent(SceneWidgetEvent* event) override;
    virtual bool onDoubleClickEvent(SceneWidgetEvent* event) override;
    //
    virtual bool onPointerMoveEvent(SceneWidgetEvent* event) override;
#if 0
    virtual void onPointerLeaveEvent(SceneWidgetEvent* event) override;
    virtual bool onButtonReleaseEvent(SceneWidgetEvent* event) override;
    virtual bool onKeyPressEvent(SceneWidgetEvent* event) override;
    virtual bool onKeyReleaseEvent(SceneWidgetEvent* event) override;
    virtual bool onScrollEvent(SceneWidgetEvent* event) override;
#endif
    virtual void onFocusChanged(SceneWidgetEvent* event, bool on) override;
    virtual bool onContextMenuRequest(SceneWidgetEvent* event) override;

protected:
    cnoid::AssemblerManager *manager;
    RASceneParts *lastClickedParts;
    RASceneConnectingPoint *lastClickedPoint;

    friend RASceneConnectingPoint;
    friend RASceneParts;
    friend cnoid::AssemblerManager;

private:
    class Impl;
    Impl *impl;
};
typedef ref_ptr<RASceneRobotBase> RASceneRobotBasePtr;

} }
#endif
