#ifndef CNOID_ROBOT_ASSEMBLER_PLUGIN_MANAGER_H
#define CNOID_ROBOT_ASSEMBLER_PLUGIN_MANAGER_H

#include "RobotAssembler.h"
#include "RobotAssemblerHelper.h"
#include "AssemblerItem.h"

#include "exportdecl.h"

namespace ra = cnoid::robot_assembler;

namespace cnoid {
struct PartsTabInfo {
  std::string name;
  std::vector<std::string> parts;
};
struct PanelSettings {
  std::vector<PartsTabInfo> tab_list;
  std::vector<std::string> combo_list;
};

class CNOID_EXPORT AssemblerManager : public SceneWidgetEventHandler
{
public:
    //static void initializeClass(ExtensionManager* ext);
    static AssemblerManager* instance(); // instance??

    AssemblerManager();
    virtual ~AssemblerManager();

    void loadSettings(const std::string _fname);
    const std::string &project_directory() { return _project_directory; }
    void setProjectDirectory(const std::string &_proj) { _project_directory = _proj; };
    bool parseButtonYaml(const std::string &filename, PanelSettings &_res);

    void partsButtonClicked(const std::string &_name);
    void addAssemblerItem(ra::RoboasmRobotPtr _rb);
    void save_model(ra::RASceneRobot *_sr);
    void save_history(ra::RASceneRobot *_sr);
    int pointClicked(ra::RASceneConnectingPoint *_cp);
    int partsClicked(ra::RASceneParts *_pt);

    void updateConnectingPoints();
    void updateMatchedPoints(ra::RASceneConnectingPoint *_pt, bool clearSelf = true,
                             ra::RASceneConnectingPoint::Clicked clearState   = ra::RASceneConnectingPoint::DEFAULT,
                             ra::RASceneConnectingPoint::Clicked matchedState = ra::RASceneConnectingPoint::CAN_CONNECT1);
    void clearAllPoints();
    void updateRobots();
    bool robotExist(ra::RASceneRobot *_rb) {
        auto it = srobot_set.find(_rb);
        return (it != srobot_set.end());
    }
    void deleteRobot(ra::RASceneRobot *_rb);
    void deleteAllRobots();
    void attachRobots(bool _just_align = false, int _increment = 1);
    void itemSelected(AssemblerItemPtr itm, bool on);
    void loadRoboasm(const std::string &_fname);
    void com_attach()  { attachRobots(); }
    void com_align()   { attachRobots(true); }
    void com_align_back()   { attachRobots(true, -1); }
    void com_unalign() { }
    void com_undo()    { }
    void com_save_model();
    void com_save_history();
    void com_load();
    void com_delete_all() { deleteAllRobots(); }

    void notifyUpdate() {
        for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
            (*it)->notifyUpdate(SgUpdate::Added | SgUpdate::Removed | SgUpdate::Modified);
        }
    }
    SignalProxy<void(ra::RoboasmRobotPtr _rb, bool on)>  sigRobotSelected() { return robotSelectedFunc; }
    SignalProxy<void(ra::RoboasmCoordsPtr _pt)> sigCoordsSelected() { return coordsSelectedFunc; }
    SignalProxy<void()> sigUpdateRobots() { return updateRobotsFunc; }

    int uniq_id;
    ra::RASceneConnectingPoint *clickedPoint0;
    ra::RASceneConnectingPoint *clickedPoint1;
    std::set<ra::RASceneConnectingPoint *> selectable_spoint_set;

    int current_align_configuration;
    ra::SettingsPtr ra_settings; // from option
    ra::RoboasmUtilPtr ra_util; // from settings
    std::set<ra::RASceneRobot*> srobot_set;
    std::string _project_directory;
protected:
    //// overrides : SceneWidgetEventHandler
    virtual void onSceneModeChanged(SceneWidgetEvent* event) override;
    //virtual bool onButtonPressEvent(SceneWidgetEvent* event) override;
    virtual bool onDoubleClickEvent(SceneWidgetEvent* event) override;
    //virtual bool onButtonReleaseEvent(SceneWidgetEvent* event) override;
    //virtual bool onPointerMoveEvent(SceneWidgetEvent* event) override;
    //virtual void onPointerLeaveEvent(SceneWidgetEvent* event) override;
    //virtual bool onKeyPressEvent(SceneWidgetEvent* event) override;
    //virtual bool onKeyReleaseEvent(SceneWidgetEvent* event) override;
    //virtual bool onScrollEvent(SceneWidgetEvent* event) override;
    //virtual void onFocusChanged(SceneWidgetEvent* event, bool on) override;
    virtual bool onContextMenuRequest(SceneWidgetEvent* event) override;

    Signal<void(ra::RoboasmRobotPtr _rb, bool on)> robotSelectedFunc;
    Signal<void(ra::RoboasmCoordsPtr _pt)> coordsSelectedFunc;
    Signal<void()> updateRobotsFunc;


private:
    class Impl;
    Impl* impl;
};
}
#endif
