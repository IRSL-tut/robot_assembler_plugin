#ifndef CNOID_ROBOT_ASSEMBLER_PLUGIN_MANAGER_H
#define CNOID_ROBOT_ASSEMBLER_PLUGIN_MANAGER_H

#include "AssemblerForBase.h"
#include "AssemblerItem.h"

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

class AssemblerManager : public SceneWidgetEventHandler
{
public:
    //static void initializeClass(ExtensionManager* ext);
    static AssemblerManager* instance(); // instance??

    AssemblerManager();
    virtual ~AssemblerManager();

    void loadSettings(const std::string _fname);
    const std::string &project_directory() { return _project_directory; }
    void setProjectDirectory(const std::string &_proj) { _project_directory = _proj; };
    const std::string &original_project() { return _original_project; }
    void setOriginalProject(const std::string &_proj) { _original_project = _proj; };
    const std::string &assembler_project() { return _assembler_project; }
    void setAssemblerProject(const std::string &_proj) { _assembler_project = _proj; };
    bool parseButtonYaml(const std::string &filename, PanelSettings &_res);

    void partsButtonClicked(const std::string &_name, const Vector3f &_color = Vector3f::Zero());
    AssemblerItemPtr addAssemblerItem(ra::RoboasmRobotPtr _rb, MappingPtr _info = nullptr);
    void save_model(ra::RASceneRobot *_sr);
    void save_history(ra::RASceneRobot *_sr);
    int pointClicked(ra::RASceneConnectingPoint *_cp);
    int partsClicked(ra::RASceneParts *_pt);
    void coordsSelected(ra::RoboasmCoordsPtr _coords);
    void updateConnectingPoints();
    void updateMatchedPoints(ra::RASceneConnectingPoint *_pt, bool clearSelf = true,
                             ra::RASceneConnectingPoint::Clicked clearState   = ra::RASceneConnectingPoint::DEFAULT,
                             ra::RASceneConnectingPoint::Clicked matchedState = ra::RASceneConnectingPoint::CAN_CONNECT1);
    void clearAllPoints();
    void updateRobots();
    void updateRobotsCoords() {
        for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
            (*it)->updateFromSelf();
            (*it)->notifyUpdate(SgUpdate::MODIFIED);
        }
    }
    bool robotExist(ra::RASceneRobot *_rb) {
        auto it = srobot_set.find(_rb);
        return (it != srobot_set.end());
    }
    bool selectRobot(ra::RASceneRobot *_rb);
    void deleteRobot(ra::RASceneRobot *_rb);
    void detachSceneRobot(ra::RASceneParts *_scpt);
    void deleteAllRobots();
    void attachRobots(bool _swap_order = true, bool _just_align = false, int _increment = 1);
    void itemSelected(AssemblerItemPtr itm, bool on);
    void loadRoboasm(const std::string &_fname, bool _rename=false);
    ra::RASceneBase *coordsToScene(ra::RoboasmCoordsPtr _coords);
    ra::RASceneRobot *coordsToScene(ra::RoboasmRobotPtr _coords)
    {
        if (!!_coords) {
            ra::RASceneRobot *pt_;
            for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
                if( (*it)->robot() == _coords ) {
                    pt_ = (*it);
                    if(!!pt_) return pt_;
                }
            }
        }
        return nullptr;
    }
    ra::RASceneParts *coordsToScene(ra::RoboasmPartsPtr _coords)
    {
        if (!!_coords) {
            ra::RASceneParts *pt_;
            for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
                pt_ = (*it)->searchParts(_coords);
                if(!!pt_) return pt_;
            }
        }
        return nullptr;
    }
    ra::RASceneConnectingPoint *coordsToScene(ra::RoboasmConnectingPointPtr _coords)
    {
        if (!!_coords) {
            ra::RASceneConnectingPoint *pt_;
            for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
                pt_ = (*it)->searchConnectingPoint(_coords);
                if(!!pt_) return pt_;
            }
        }
        return nullptr;
    }

    bool isRunningAssembler();

    //// simple command without arguments for toolbar, etc.
    void com_attach()  { attachRobots(swap_order); }
    void com_attach_o(){ attachRobots(false); }
    void com_align()   { attachRobots(swap_order, true); }
    void com_align_back() { attachRobots(swap_order, true, -1); }
    void com_ordered_attach(bool _in) { swap_order = !_in; }
    void com_unalign() { }
    void com_undo()    { }
    void com_save_model();
    void com_save_history();
    void com_load();
    void com_delete_all() { deleteAllRobots(); }
    void com_swap_mode();
    void com_unselect_points() {
        clickedPoint0=nullptr;
        clickedPoint1=nullptr;
        updateConnectingPoints();
        notifyUpdate();
    }
    void notifyUpdate() {
        for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
            //(*it)->notifyUpdate(SgUpdate::Added | SgUpdate::Removed | SgUpdate::Modified);
            //for(auto pit = (*it)->sparts_set.begin(); pit != (*it)->sparts_set.end(); pit++) {
            //    (*pit)->notifyUpdate(SgUpdate::Added | SgUpdate::Removed | SgUpdate::Modified);
            //}
            for(auto pit = (*it)->spoint_set.begin(); pit != (*it)->spoint_set.end(); pit++) {
                (*pit)->notifyUpdate(SgUpdate::Added | SgUpdate::Removed | SgUpdate::Modified);
            }
        }
    }
    SignalProxy<void(ra::RoboasmRobotPtr _rb, bool on)> sigRobotSelected() { return robotSelectedFunc; }
    SignalProxy<void(ra::RoboasmCoordsPtr _pt, MappingPtr _info)> sigCoordsSelected() { return coordsSelectedFunc; }
    SignalProxy<void()> sigUpdateRobots() { return updateRobotsFunc; }

    bool isAssembler() { return (_current_mode == ASSEMBLER); }

    int uniq_id;
    ra::RASceneConnectingPoint *clickedPoint0;
    ra::RASceneConnectingPoint *clickedPoint1;
    std::set<ra::RASceneConnectingPoint *> selectable_spoint_set;
    ra::RASceneParts *clickedParts;

    int current_align_configuration;
    ra::SettingsPtr ra_settings; // from option
    ra::RoboasmUtilPtr ra_util; // from settings
    std::set<ra::RASceneRobot*> srobot_set;
protected:
    //// overrides : SceneWidgetEventHandler
    virtual void onSceneModeChanged(SceneWidgetEvent* event) override;
    //virtual bool onButtonPressEvent(SceneWidgetEvent* event) override;
    virtual bool onDoubleClickEvent(SceneWidgetEvent* event) override;
    //virtual bool onButtonReleaseEvent(SceneWidgetEvent* event) override;
    //virtual bool onPointerMoveEvent(SceneWidgetEvent* event) override;
    //virtual void onPointerLeaveEvent(SceneWidgetEvent* event) override;
    virtual bool onKeyPressEvent(SceneWidgetEvent* event) override;
    virtual bool onKeyReleaseEvent(SceneWidgetEvent* event) override;
    //virtual bool onScrollEvent(SceneWidgetEvent* event) override;
    //virtual void onFocusChanged(SceneWidgetEvent* event, bool on) override;
    virtual bool onContextMenuRequest(SceneWidgetEvent* event) override;

    int pointClickedProcess(ra::RASceneConnectingPoint *_cp);
    int partsClickedProcess(ra::RASceneParts *_pt);

    Signal<void(ra::RoboasmRobotPtr _rb, bool on)> robotSelectedFunc;
    Signal<void(ra::RoboasmCoordsPtr _pt, MappingPtr _info)> coordsSelectedFunc;
    Signal<void()> updateRobotsFunc;

private:
    class Impl;
    Impl* impl;

    enum MODE {NONE, ASSEMBLER, CNOID};
    bool swap_order;
    MODE _current_mode;
    std::string _project_directory;
    std::string _original_project;
    std::string _assembler_project;
};
}
#endif
