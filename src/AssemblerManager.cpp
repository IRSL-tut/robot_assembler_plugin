#include "AssemblerManager.h"

#include "RobotAssemblerBody.h"
#include "AssemblerView.h"
#include "RobotAssemblerInfo.h"

#include <cnoid/StdBodyWriter>
#include <cnoid/YAMLReader>
#include <cnoid/MenuManager>
#include <cnoid/RootItem>
#include <cnoid/ItemList>
#include <cnoid/FileDialog>
#include <cnoid/SceneView>
#include <cnoid/SceneWidget>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>

#include <cnoid/ProjectManager>

#include <vector>
#include <QLabel>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QComboBox>

//#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;
using namespace cnoid::robot_assembler;

namespace ra = cnoid::robot_assembler;
namespace filesystem = cnoid::stdx::filesystem;

namespace cnoid {

class AssemblerManager::Impl
{
public:
    Impl(AssemblerManager *_self) { self = _self; }
    ~Impl() {}

    AssemblerManager *self;
};

}

AssemblerManager* AssemblerManager::instance()
{
    static AssemblerManager* instance_ = new AssemblerManager();
    return instance_;
}
AssemblerManager::AssemblerManager() : clickedPoint0(nullptr), clickedPoint1(nullptr), clickedParts(nullptr)
{
    DEBUG_PRINT();

    impl = new Impl(this);

    swap_order = true;
    _current_mode = ASSEMBLER;
    uniq_id = SceneWidget::issueUniqueCustomModeId();
    SceneView::instance()->sceneWidget()->activateCustomMode(this, uniq_id);
}
AssemblerManager::~AssemblerManager()
{
    delete impl;
}
void AssemblerManager::loadSettings(const std::string _fname)
{
    filesystem::path path_(fromUTF8(_fname));
    std::string ppath_;
    if(path_.is_relative()) {
        auto ap_ = filesystem::absolute(path_);
        ppath_ = ap_.parent_path().generic_string();
        DEBUG_STREAM("rel: ppath: " << ppath_);
    } else {
        ppath_ = path_.parent_path().generic_string();
        DEBUG_STREAM("abs: ppath: " << ppath_);
    }
    ra::SettingsPtr _ra_settings = std::make_shared<ra::Settings> ();
    bool ret = _ra_settings->parseYaml(_fname);
    if (ret) {
        setProjectDirectory(ppath_);
        ra_settings = _ra_settings;
        ra_util = std::make_shared<ra::RoboasmUtil>(_ra_settings);
        AssemblerView *ptr = AssemblerView::instance();
        if(!!ptr) {
            PanelSettings pnl_;
            if(parseButtonYaml(_fname, pnl_)) {
                ptr->createButtons(pnl_);
            } else {
                ptr->createButtons(pnl_);
            }
        }
    }
}
void AssemblerManager::partsButtonClicked(const std::string &_name, const Vector3f &_color)
{
    DEBUG_STREAM( " parts: " << _name );
    std::string rb_name;
    if (srobot_set.size() == 0) {
        rb_name = "AssembleRobot";
    } else {
        rb_name = _name;
    }
    ra::RoboasmRobotPtr rb_ = ra_util->makeRobot(rb_name, _name, _color);
    if(!!rb_) {
        addAssemblerItem(rb_);
    } else {
        ERROR_STREAM(" robot create error from parts : " << _name);
    }
}
void AssemblerManager::detachSceneRobot(ra::RASceneParts *_scpt)
{
    DEBUG_STREAM(" detach parts : " << _scpt->name());
    ra::RASceneRobot *scrb_ = _scpt->scene_robot();
    ra::RoboasmRobotPtr rb_ = scrb_->robot();

    ra::RoboasmPartsPtr pt_ = _scpt->parts();

    ra::RoboasmRobotPtr newrb_ = rb_->detach(pt_);
    if(!!newrb_) {
        DEBUG_STREAM(" newrb");
        // clone-info [TODO]
        deleteRobot(scrb_);
        addAssemblerItem(rb_);
        DEBUG_STREAM(" add newrb");
        addAssemblerItem(newrb_);
    }
}
AssemblerItemPtr AssemblerManager::addAssemblerItem(ra::RoboasmRobotPtr _rb, MappingPtr _info)
{
    AssemblerItemPtr itm = AssemblerItem::createItem(_rb, this);
    if (!!itm) {
        // scene boundingbox
        const BoundingBox &bb = SceneView::instance()->sceneWidget()->scene()->boundingBox();
        Vector3 cent = bb.center();
        Vector3 size = bb.size();
        DEBUG_STREAM(" cent: " << cent(0) << ", " << cent(1) << ", " << cent(2) );
        DEBUG_STREAM(" size: " << size(0) << ", " << size(1) << ", " << size(2) );
        // robot boundingbox
        ra::RASceneRobot* rb_scene = dynamic_cast<ra::RASceneRobot*> (itm->getScene());
        const BoundingBox &rb_bb = rb_scene->boundingBox();
        Vector3 rb_cent = rb_bb.center();
        Vector3 rb_size = rb_bb.size();
        DEBUG_STREAM(" rb_cent: " << rb_cent(0) << ", " << rb_cent(1) << ", " << rb_cent(2) );
        DEBUG_STREAM(" rb_size: " << rb_size(0) << ", " << rb_size(1) << ", " << rb_size(2) );
        // move robot
        coordinates cds(Vector3(0, cent(1) - rb_cent(1) + size(1)/2 +  1.2 * rb_size(1)/2, 0));
        rb_scene->setCoords(cds);
        //
        if(!!_info) {
            rb_scene->info = _info;
        } else {
            rb_scene->info = ra::createInfo(_rb);
        }
        //
        current_align_configuration = -1;
        clickedPoint0 = nullptr;
        clickedPoint1 = nullptr;
        clickedParts = nullptr;
        selectable_spoint_set.clear();
        itm->setChecked(true);
        itm->setSelected(true);
        RootItem::instance()->addChildItem(itm);
        //
        updateRobots();
        clearAllPoints();
        notifyUpdate();
        SceneView::instance()->sceneWidget()->viewAll();

        return itm;
    } else {
        ERROR_STREAM(" item create error : " << _rb->name());
    }
    return nullptr;
}
int AssemblerManager::pointClicked(ra::RASceneConnectingPoint *_cp)
{
    ra::RoboasmCoordsPtr ptr_ = _cp->point();
    ra::RASceneRobot *rb_ =_cp->scene_robot();
    MappingPtr info_;
    if (!!rb_) info_ = rb_->info;
    coordsSelectedFunc(ptr_, info_);
    return pointClickedProcess(_cp);
}
int AssemblerManager::partsClicked(ra::RASceneParts *_pt)
{
    ra::RoboasmCoordsPtr ptr_ = _pt->parts();
    ra::RASceneRobot *rb_ =_pt->scene_robot();
    MappingPtr info_;
    if (!!rb_) info_ = rb_->info;
    coordsSelectedFunc(ptr_, info_);
    return partsClickedProcess(_pt);
}
int AssemblerManager::pointClickedProcess(ra::RASceneConnectingPoint *_cp)
{
    DEBUG_STREAM(" " << _cp->name() );
    //_cp->switchOn(false);
    //_cp->notifyUpdate(SgUpdate::REMOVED | SgUpdate::ADDED | SgUpdate::MODIFIED);// on scene graph

    if ( !robotExist(_cp->scene_robot()) ) {
        DEBUG_STREAM(" robot not exist??" );
    }
    bool modified = false;
    if (clickedPoint0 == _cp) {
        if (!!clickedPoint1) {
            // move 1 -> 0 / (0: a, 1: b) :=> (0: b, 1: null)
            DEBUG_STREAM(" state0 : " << _cp->name() );
            clickedPoint0 = clickedPoint1;
            clickedPoint1 = nullptr;
            modified = true;
        } else {
            // toggle selection0
            DEBUG_STREAM(" state1 : " << _cp->name() );
            clickedPoint0 = nullptr;
            modified = true;
        }
    } else if (clickedPoint1 == _cp) {
        // toggle selection1
        DEBUG_STREAM(" state2 : " << _cp->name() );
        clickedPoint1 = nullptr;
        modified = true;
    } else if (!clickedPoint0 && !clickedPoint1) {
        // add first one
        DEBUG_STREAM(" state3 : " << _cp->name() );
        clickedPoint0 = _cp;
        modified = true;
    } else if (!!clickedPoint0 && !!clickedPoint1) {
        DEBUG_STREAM(" state4 : " << _cp->name() );
        if(clickedPoint0->scene_robot() == _cp->scene_robot()) {
            DEBUG_STREAM(" state4.0 : " << _cp->name() );
            // same robot of selection0
            clickedPoint0 = _cp;
            modified = true;
        } else if (clickedPoint1->scene_robot() == _cp->scene_robot()) {
            DEBUG_STREAM(" state4.1 : " << _cp->name() );
            // same robot of selection1
            clickedPoint1 = _cp;
            modified = true;
        } else {
            DEBUG_STREAM(" state4.2 : " << _cp->name() );
            // replace last one
            clickedPoint1 = _cp;
            modified = true;
        }
    } else if (!!clickedPoint0) {
        DEBUG_STREAM(" state5 : " << _cp->name() );
        if(clickedPoint0->scene_robot() == _cp->scene_robot()) {
            DEBUG_STREAM(" state5.0 : " << _cp->name() );
            // replace selection0
            clickedPoint0 = _cp;
            modified = true;
        } else {
            DEBUG_STREAM(" state5.1 : " << _cp->name() );
            // add second one
            clickedPoint1 = _cp;
            modified = true;
        }
    } else if (!!clickedPoint1) {
        DEBUG_STREAM(" state6 : (not occur!!) " << _cp->name() );
        if(clickedPoint1->scene_robot() == _cp->scene_robot()) {
            DEBUG_STREAM(" state6.0 " << _cp->name() );
            clickedPoint0 = _cp;
            clickedPoint1 = nullptr;
            modified = true;
        } else {
            DEBUG_STREAM(" state6.1 " << _cp->name() );
            // add second one?
            clickedPoint0 = clickedPoint1;
            clickedPoint1 = _cp;
            modified = true;
        }
    } else {
        DEBUG_STREAM(" === unknown state === : " << _cp->name() );
    }
    DEBUG_STREAM(" 0: " << (!!(clickedPoint0) ? clickedPoint0->name() : "null")
                 << " | 1: " << (!!(clickedPoint1) ? clickedPoint1->name() : "null"));
    if(modified) {
        updateConnectingPoints();
        for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
            (*it)->notifyUpdate(SgUpdate::Added | SgUpdate::Removed | SgUpdate::Modified);
        }
    }
    return 1;
}
int AssemblerManager::partsClickedProcess(ra::RASceneParts *_pt)
{
    DEBUG_STREAM(" " << _pt->name() );
    if (_pt == clickedParts) {
        _pt->drawBoundingBox(false);
        clickedParts = nullptr;
    } else {
        _pt->drawBoundingBox();
        if (!!clickedParts) {
            clickedParts->drawBoundingBox(false);
        }
        clickedParts = _pt;
    }
    for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
        (*it)->notifyUpdate(SgUpdate::Added | SgUpdate::Removed | SgUpdate::Modified);
    }
    return 1;
}
void AssemblerManager::coordsSelected(ra::RoboasmCoordsPtr _coords)
{
    DEBUG_STREAM(" " << _coords->name());
    bool in_scene = false;
    MappingPtr info_;
    {// _coords is robot
        ra::RoboasmRobotPtr ptr = ra::RoboasmUtil::toRobot(_coords);
        if (!!ptr) {
            ra::RASceneRobot *pt_;
            for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
                if( (*it)->robot() == ptr ) {
                    pt_ = (*it);
                    break;
                }
            }
            if(!!pt_) {
                in_scene = true;
                info_ = pt_->info;
            }
            goto endprocess;
        }
    }
    {// _coords is parts
        ra::RoboasmPartsPtr ptr = ra::RoboasmUtil::toParts(_coords);
        if (!!ptr) {
            bool is_link = ptr->isLink();
            // partsClicked
            ra::RASceneParts *pt_;
            for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
                pt_ = (*it)->searchParts(ptr);
                if(!!pt_) {
                    info_ = (*it)->info;
                    break;
                }
            }
            if(!!pt_) {
                in_scene = true;
                partsClickedProcess(pt_);
            }
            goto endprocess;
        }
    }
    {// _coords is connecting-point
        ra::RoboasmConnectingPointPtr ptr = ra::RoboasmUtil::toConnectingPoint(_coords);
        if (!!ptr) {
            bool is_actuator = ptr->isActuator();
            bool is_connected = ptr->isConnected();
            // pointClicked
            ra::RASceneConnectingPoint *pt_;
            for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
                pt_ = (*it)->searchConnectingPoint(ptr);
                if(!!pt_) {
                    info_ = (*it)->info;
                    break;
                }
            }
            if(!!pt_) {
                in_scene = true;
                pointClickedProcess(pt_);
            }
            goto endprocess;
        }
    }
    // never occur / not robot && not parts && not connecting-point
    ERROR_STREAM(" ");
    return;
endprocess:
    coordsSelectedFunc(_coords, info_);
}
void AssemblerManager::updateConnectingPoints()
{
    //clickedPoint0
    //clickedPoint1
    if(!!clickedPoint0 && !!clickedPoint1) {
        DEBUG_STREAM(" state 0 : both clicked" );
        //selectable_spoint_set.clear();
        // can match clickedPoint0/clickedPoint1
        bool can_match = ra_util->canMatch(clickedPoint0->point(), clickedPoint1->point());
        updateMatchedPoints(clickedPoint0);
        // updateMatchedPoints(clickedPoint1);
        if(can_match) {
            clickedPoint0->changeState(ra::RASceneConnectingPoint::SELECT_GOOD0);
            clickedPoint1->changeState(ra::RASceneConnectingPoint::SELECT_GOOD1);
        } else {
            clickedPoint0->changeState(ra::RASceneConnectingPoint::SELECT_BAD0);
            clickedPoint1->changeState(ra::RASceneConnectingPoint::SELECT_BAD1);
        }
    } else if (!!clickedPoint0 &&  !clickedPoint1) {
        DEBUG_STREAM(" state 1 : one  clicked" );
        updateMatchedPoints(clickedPoint0);
        clickedPoint0->changeState(ra::RASceneConnectingPoint::SELECT_BAD0);
        //selectable_spoint_set.clear();
    } else if ( !clickedPoint0 && !!clickedPoint1) {
        DEBUG_STREAM(" state 2 : not occur!!" );
        // not occur
        updateMatchedPoints(clickedPoint1);
        clickedPoint1->changeState(ra::RASceneConnectingPoint::SELECT_BAD1);
        //selectable_spoint_set.clear();
    } else {
        DEBUG_STREAM(" state 3 : all clear" );
        //selectable_spoint_set.clear();
        clearAllPoints();
    }
}
void AssemblerManager::clearAllPoints()
{
    for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
        auto pit_end = (*it)->spoint_set.end();
        for(auto pit = (*it)->spoint_set.begin(); pit != pit_end; pit++) {
            (*pit)->changeState(ra::RASceneConnectingPoint::DEFAULT);
        }
    }
}
void AssemblerManager::updateMatchedPoints(ra::RASceneConnectingPoint *_pt, bool clearSelf,
                                           ra::RASceneConnectingPoint::Clicked clearState,
                                           ra::RASceneConnectingPoint::Clicked matchedState)
{
    ra::RoboasmConnectingPointPtr cp_ = _pt->point();
    for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
        if( (*it) == _pt->scene_robot() ) {
            if(clearSelf) {
                auto pit_end = (*it)->spoint_set.end();
                for(auto pit = (*it)->spoint_set.begin(); pit != pit_end; pit++)
                    (*pit)->changeState(clearState);
            }
            continue;
        }
        auto pit_end = (*it)->spoint_set.end();
        for(auto pit = (*it)->spoint_set.begin(); pit != pit_end; pit++) {
            if(ra_util->canMatch(cp_, (*pit)->point())) {
                (*pit)->changeState(matchedState);
            } else {
                (*pit)->changeState(clearState);
            }
        }
    }
}
void AssemblerManager::updateRobots()
{
    ItemList<AssemblerItem> lst =  RootItem::instance()->checkedItems<AssemblerItem>();
    DEBUG_STREAM(" lst : " << lst.size() );
    srobot_set.clear();
    for(auto it = lst.begin(); it != lst.end(); it++) {
        SgNode *node = (*it)->getScene();
        ra::RASceneRobot *rbt = dynamic_cast<ra::RASceneRobot*>(node);
        if(!!rbt) {
            srobot_set.insert(rbt);
        }
    }
    DEBUG_STREAM(" robot_list : " << srobot_set.size() );
}
void AssemblerManager::deleteAllRobots()
{
    ItemList<AssemblerItem> lst =  RootItem::instance()->checkedItems<AssemblerItem>();
    srobot_set.clear();
    for(auto it = lst.begin(); it != lst.end(); it++) {
        (*it)->removeFromParentItem();
    }
    RootItem::instance()->notifyUpdate();//??
    clickedPoint0 = nullptr;
    clickedPoint1 = nullptr;
    clickedParts = nullptr;
    selectable_spoint_set.clear();
    clearAllPoints();
    notifyUpdate();
    SceneView::instance()->sceneWidget()->viewAll();
}
bool AssemblerManager::selectRobot(ra::RASceneRobot *_rb)
{
    DEBUG_STREAM(" select robot : " << _rb->name() );
    ItemList<AssemblerItem> lst =  RootItem::instance()->checkedItems<AssemblerItem>();
    for(auto it = lst.begin(); it != lst.end(); it++) {
        (*it)->setSelected(false);
    }
    for(auto it = lst.begin(); it != lst.end(); it++) {
        SgNode *node = (*it)->getScene();
        ra::RASceneRobot *rbt = dynamic_cast<ra::RASceneRobot*>(node);
        if (!!rbt && (rbt == _rb)) {
            // find robot
            DEBUG_STREAM(" find:" );
            (*it)->setSelected(true, true);
            return true;
        }
    }
    return false;
}
void AssemblerManager::deleteRobot(ra::RASceneRobot *_rb)
{
    DEBUG_STREAM(" delete robot : " << _rb->name() );
    ItemList<AssemblerItem> lst =  RootItem::instance()->checkedItems<AssemblerItem>();
    srobot_set.clear();
    for(auto it = lst.begin(); it != lst.end(); it++) {
        SgNode *node = (*it)->getScene();
        ra::RASceneRobot *rbt = dynamic_cast<ra::RASceneRobot*>(node);
        if (!!rbt) {
            if (rbt == _rb) {
                // delete
                DEBUG_STREAM(" delete:" );
                (*it)->setChecked(false);
                (*it)->removeFromParentItem();
            } else {
                srobot_set.insert(rbt);
            }
        }
    }
    RootItem::instance()->notifyUpdate();//??
    clickedPoint0 = nullptr;
    clickedPoint1 = nullptr;
    clickedParts = nullptr;
    selectable_spoint_set.clear();
    clearAllPoints();
    notifyUpdate();
    SceneView::instance()->sceneWidget()->viewAll();
}
void AssemblerManager::attachRobots(bool _swap_order, bool _just_align, int increment)
{
    DEBUG_STREAM(" swap: " << _swap_order);
    if(!clickedPoint0 || !clickedPoint1) {
        DEBUG_STREAM( " require 2 clicked point" );
        return;
    }

    ra::RASceneConnectingPoint *cp0 = clickedPoint0;
    ra::RASceneConnectingPoint *cp1 = clickedPoint1;
    if(_swap_order && (cp1->scene_robot()->robot()->partsNum() >
                       cp0->scene_robot()->robot()->partsNum() ) ) { // swap 0 and 1
        ra::RASceneConnectingPoint *tmp = cp0;
        cp0 = cp1; cp1 = tmp;
    }

    ra::RoboasmRobotPtr rb0 = cp0->scene_robot()->robot();
    ra::RoboasmRobotPtr rb1 = cp1->scene_robot()->robot();

    DEBUG_STREAM(" rb0: " << rb0->name() << " <=(attach) rb1:" << rb1->name() );
    DEBUG_STREAM(" cp0-point(): " << cp0->point()->name() << " | cp1-point(): " << cp1->point()->name() );

    bool res;
    std::vector<ra::ConnectingTypeMatch*> res_match_lst;
    res = rb0->searchMatch(rb1, cp1->point(), cp0->point(),
                           res_match_lst);
    if(!res) {
        DEBUG_STREAM(" not matched " );
        return;
    }
    DEBUG_STREAM(" matched : " << res_match_lst.size() );
    int counter_ = 0; bool find_ = false;
    int target_config_;
    if(_just_align) {
        target_config_ = current_align_configuration + increment;
    } else {
        if(current_align_configuration < 0) {
            target_config_ = 0; // attach by initial condition
        } else {
            target_config_ = current_align_configuration;
        }
    }
    ra::ConnectingConfigurationID ccid = res_match_lst.front()->allowed_configuration.front();
    for(int i = 0; i < res_match_lst.size(); i++) {
        ra::ConnectingTypeMatch* mt_ = res_match_lst[i];
        for(int j = 0; j < mt_->allowed_configuration.size(); j++) {
            if (counter_ == target_config_) {
                ccid = mt_->allowed_configuration[j];
                find_ = true;
            }
            counter_++;
        }
    }
    if(target_config_ < 0) {
        current_align_configuration = counter_ - 1;
        ccid = res_match_lst.back()->allowed_configuration.back();
    } else if(!find_) {
        current_align_configuration = 0;
    } else {
        current_align_configuration = target_config_;
    }
    if(_just_align) {
        DEBUG_STREAM(" align: " << ccid);
    } else {
        DEBUG_STREAM(" attach: " << ccid);
    }
    ra::RoboasmParts *attached_parts = dynamic_cast<ra::RoboasmParts*>(cp1->point()->parent());
    bool res_attach = rb0->attach(rb1, cp1->point(), cp0->point(), ccid, _just_align);
    if (!res_attach) {
        DEBUG_STREAM(" attach failed " );
        return;
    }
    if(_just_align) {
        rb1->update();
        rb1->updateDescendants();
        cp1->scene_robot()->updateFromSelf();
        notifyUpdate();
        return;
    }
    // process connecting robots
    coordinates attach_coords;
    cp0->point()->worldcoords().transformation(attach_coords, cp1->point()->worldcoords());
    std::string &config_ = ra_settings->listConnectingConfiguration[ccid].name;
    if (cp1->scene_robot()->history.size() == 1) {
        cp0->scene_robot()->attachHistory(
            cp1->scene_robot()->history,
            cp0->point()->parent()->name(), //parent,
            cp0->point()->point_name(),     //parent_point,
            attached_parts->name(),         //parts_name,
            attached_parts->info->type,     //parts_type,
            cp1->point()->point_name(),     //parts_point,
            attach_coords);
    } else {
        ra::AttachHistory hist_;
        attached_parts->dumpConnectionFromParent(hist_); //??
        cp0->scene_robot()->attachHistory(
            hist_,
            cp0->point()->parent()->name(), //parent,
            cp0->point()->point_name(),     //parent_point,
            attached_parts->name(),         //parts_name,
            attached_parts->info->type,     //parts_type,
            cp1->point()->point_name(),     //parts_point,
            attach_coords);
    }
    ra::mergeInfo(cp0->scene_robot()->info,
                  cp1->scene_robot()->info);
    // erase(rb1)
    // update position of cp0,cp1 <- worldcoords
    cp0->scene_robot()->updateFromSelf();
    cp1->scene_robot()->updateFromSelf();

    //marge cp0->scene_robot() <=: cp1->scene_robot()
    ra::RASceneRobot *to_delete = cp1->scene_robot(); // after merged robots, all connecting point should be
    cp0->scene_robot()->mergeRobot(cp1->scene_robot());
    DEBUG_STREAM(" scene_robot0: " << cp0->scene_robot()->name() << " / scene_robot1: " << to_delete->name() );
    deleteRobot(to_delete);
    //notifyUpdate(); // update at delete robot
}
void AssemblerManager::itemSelected(AssemblerItemPtr itm, bool on)
{
    ra::RASceneRobot *rb_ = dynamic_cast<ra::RASceneRobot*>(itm->getScene());
    robotSelectedFunc(rb_->robot(), on);
    if(!on) {
        coordsSelectedFunc(nullptr, nullptr);
    }
}
void AssemblerManager::loadRoboasm(const std::string &_fname, bool _rename)
{
    ra::cnoidRAFile raf(_fname);
    if(!raf.isValid()) {
        ERROR_STREAM(" invalid roboasm : " << _fname);
        return;
    }
    if(_rename) {
        DEBUG_STREAM(" renamed");
        ra::StringMap rmap_;
        ra_util->renamePartsHistory(raf.history, rmap_);
        raf.renameInfo(rmap_);
    }
    std::string name_;
    if(!raf.getRobotName(name_)) name_ = "AssembleRobot";
    ra::RoboasmRobotPtr rb_ = ra_util->makeRobot(name_, raf.history);
    if(!!rb_) {
        raf.updateRobotByInfo(rb_);
        AssemblerItemPtr itm_ = addAssemblerItem(rb_, raf.info);
        //
        if(!!itm_) {
            ra::RASceneRobot *srb_ = dynamic_cast<ra::RASceneRobot *>(itm_->getScene());
            ra::connectingPointPtrList a_act;
            rb_->inactiveActuators(a_act);
            for(auto it = a_act.begin(); it != a_act.end(); it++) {
                double ang;
                if (raf.getActuatorValue((*it)->name(), "current-angle", ang)) {
                    (*it)->applyJointAngle(ang);
                }
            }
            srb_->updateStructure();
            srb_->notifyUpdate(SgUpdate::MODIFIED);
        }
    } else {
        ERROR_STREAM(" robot build failed");
    }
}
ra::RASceneBase *AssemblerManager::coordsToScene(ra::RoboasmCoordsPtr _coords)
{
    // _coords is robot
    ra::RoboasmRobotPtr rptr = ra::RoboasmUtil::toRobot(_coords);
    if (!!rptr) return coordsToScene(rptr);
    // _coords is parts
    ra::RoboasmPartsPtr pptr = ra::RoboasmUtil::toParts(_coords);
    if (!!pptr) return coordsToScene(pptr);
    // _coords is connecting-point
    ra::RoboasmConnectingPointPtr cptr = ra::RoboasmUtil::toConnectingPoint(_coords);
    if (!!cptr) return coordsToScene(cptr);
    return nullptr;
}
//// protected
void AssemblerManager::onSceneModeChanged(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
}
bool AssemblerManager::onDoubleClickEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    // override double-click default behavior(change mode)
    return true;
}
#if 0
////
bool AssemblerManager::onPointerMoveEvent(SceneWidgetEvent* event)
{
    return false;
}
bool AssemblerManager::onButtonPressEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    return false;
}
bool AssemblerManager::onButtonReleaseEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    return false;
}
void AssemblerManager::onPointerLeaveEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
}
////
#endif
bool AssemblerManager::onKeyPressEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    if (event->modifiers() | Qt::AltModifier) {
        DEBUG_STREAM("alt");
    }
    if (event->key() == Qt::Key_Tab) {
        com_unselect_points();
    }
    return false;
}
bool AssemblerManager::onKeyReleaseEvent(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    return false;
}
bool AssemblerManager::onContextMenuRequest(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    if (_current_mode != ASSEMBLER) {
        return false;
    }
    SgNodePath enp = event->nodePath();
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

    auto menu = event->contextMenu();

    menu->addSeparator();
    menu->addItem("Align")->sigTriggered().connect(
        [this](){ com_align(); } );
    menu->addItem("UnAlign")->sigTriggered().connect(
        [this](){ com_unalign(); } );
    menu->addSeparator();
    menu->addItem("Attach")->sigTriggered().connect(
        [this](){ com_attach(); } );
    menu->addItem("Attach Order")->sigTriggered().connect(
        [this](){ com_attach_o(); } );
    menu->addItem("Undo")->sigTriggered().connect(
        [this](){ com_undo(); } );
    menu->addSeparator();
    menu->addItem("Save history")->sigTriggered().connect(
        [this](){ com_save_history(); } );
    menu->addItem("Save model")->sigTriggered().connect(
        [this](){ com_save_model(); } );
    menu->addSeparator();
    menu->addItem("Delete All")->sigTriggered().connect(
        [this](){ com_delete_all(); } );
    menu->addSeparator();
    menu->addItem("Unselect points")->sigTriggered().connect(
        [this](){ com_unselect_points(); } );
    //return true;
    return false;
}
void AssemblerManager::com_save_history()
{
    ItemList<AssemblerItem> lst =  RootItem::instance()->checkedItems<AssemblerItem>();
    DEBUG_STREAM(" lst : " << lst.size());
    if(lst.size() == 1) {
        SgNode *node = lst.front()->getScene();
        ra::RASceneRobot *rbt = dynamic_cast<ra::RASceneRobot*>(node);
        if(!!rbt) {
            save_history(rbt);
        }
    } else {
        ERROR_STREAM(" AssemblerItem more than 1");
    }
}
void AssemblerManager::com_save_model()
{
    ItemList<AssemblerItem> lst =  RootItem::instance()->checkedItems<AssemblerItem>();
    DEBUG_STREAM(" lst : " << lst.size());
    if(lst.size() == 1) {
        SgNode *node = lst.front()->getScene();
        ra::RASceneRobot *rbt = dynamic_cast<ra::RASceneRobot*>(node);
        if(!!rbt) {
            save_model(rbt);
        }
    } else {
        ERROR_STREAM(" AssemblerItem more than 1");
    }
}
void AssemblerManager::save_history(ra::RASceneRobot *_sr)
{
    DEBUG_PRINT();

    auto dialog = new FileDialog();
    int filter_id = 0;
    dialog->sigFilterSelected().connect( [&filter_id](int i) { filter_id = i; });
    dialog->setWindowTitle("Save assemble history");
    dialog->setFileMode(QFileDialog::AnyFile);
    dialog->setAcceptMode(QFileDialog::AcceptSave);
    dialog->setViewMode(QFileDialog::List);
    dialog->setLabelText(QFileDialog::Accept, "Save");
    dialog->setLabelText(QFileDialog::Reject, "Cancel");
    dialog->setOption(QFileDialog::DontConfirmOverwrite);

    QStringList filters;
    filters << "roboasm files (*.roboasm)";
    filters << "Any files (*)";
    dialog->setNameFilters(filters);

    std::string initialname = _sr->robot()->name() + ".roboasm";
    dialog->selectFile(initialname);
    if(dialog->exec() == QDialog::Accepted) {
        DEBUG_STREAM(" accepted");
        auto fnames = dialog->selectedFiles();
        std::string fname;
        if(!fnames.isEmpty()) {
            fname = fnames.front().toStdString();
            filesystem::path path(fromUTF8(fname));
            std::string ext = path.extension().string();
            if(ext == ".roboasm"){
                DEBUG_STREAM(" roboasm : " << fname);
            } else {
                if (filter_id == 0) {
                    fname += ".roboasm";
                }
                DEBUG_STREAM(" roboasm : " << fname);
            }
        }
        if(fname.size() > 0) {
            ra::cnoidRAFile raf;
            raf.history = _sr->history;
            DEBUG_STREAM(" hist: " << raf.history.size());
            raf.info = _sr->info;
            //_sr->robot()->writeConfig(raf.config);
            raf.dumpRoboasm(fname);
        }
    }

    delete dialog;
}
void AssemblerManager::save_model(ra::RASceneRobot *_sr)
{
    DEBUG_PRINT();

    auto dialog = new FileDialog();
    int filter_id = 0;
    dialog->sigFilterSelected().connect( [&filter_id](int i) { filter_id = i; });
    dialog->setWindowTitle("Save a robot model");
    dialog->setFileMode(QFileDialog::AnyFile);
    dialog->setAcceptMode(QFileDialog::AcceptSave);
    dialog->setViewMode(QFileDialog::List);
    dialog->setLabelText(QFileDialog::Accept, "Save");
    dialog->setLabelText(QFileDialog::Reject, "Cancel");
    dialog->setOption(QFileDialog::DontConfirmOverwrite);

    // add option panel to Dialog, see BodyItemFileIO.cpp
    QWidget *optionPanel = new QWidget;
    QVBoxLayout *optionVBox = new QVBoxLayout;
    optionVBox->setContentsMargins(0, 0, 0, 0);
    optionPanel->setLayout(optionVBox);

    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel("Mode: exporting mesh file"));
    QComboBox* extModelFileModeCombo = new QComboBox;
    extModelFileModeCombo->addItem(
        "Embed models", StdSceneWriter::EmbedModels);
    extModelFileModeCombo->addItem(
        "Link to the original model files", StdSceneWriter::LinkToOriginalModelFiles);
    extModelFileModeCombo->addItem(
        "Replace with OBJ model files", StdSceneWriter::ReplaceWithObjModelFiles);
    extModelFileModeCombo->addItem(
        "Replace with standard scene files", StdSceneWriter::ReplaceWithStdSceneFiles);
    hbox->addWidget(extModelFileModeCombo);

    //QCheckBox *transformIntegrationCheck = new QCheckBox;
    //transformIntegrationCheck->setText("Integrate transforms");
    //hbox->addWidget(transformIntegrationCheck);

    optionVBox->addLayout(hbox);
    dialog->insertOptionPanel(optionPanel);

    QStringList filters;
    filters << "body files (*.body)";
    filters << "urdf files (*.urdf)";
    filters << "Any files (*)";
    dialog->setNameFilters(filters);

    std::string initialname = _sr->robot()->name() + ".body";
    dialog->selectFile(initialname);
    if(dialog->exec() == QDialog::Accepted) {
        DEBUG_STREAM(" accepted");
        auto fnames = dialog->selectedFiles();
        std::string fname;
        bool urdf = false;
        if(!fnames.isEmpty()) {
            fname = fnames.front().toStdString();
            filesystem::path path(fromUTF8(fname));
            std::string ext = path.extension().string();
            if(ext == ".body"){
                DEBUG_STREAM(" body : " << fname);
            } else if (ext == ".urdf") {
                DEBUG_STREAM(" urdf : " << fname);
                urdf = true;
            } else {
                DEBUG_STREAM(" filter? " << filter_id); // 0, 1, 2
                // path.extension().string();
                //DEBUG_STREAM(" body : " << fname << ".body");
                if (filter_id == 0) {
                    fname += ".body";
                } else if (filter_id == 1) {
                    urdf = true;
                    fname += ".urdf";
                }
            }
        }
        if(fname.size() > 0) {
            if (urdf) {
                // [TODO] if urdf
            } else {
            // createBody
            ra::RoboasmBodyCreator bc(_project_directory);
            //bc.setMergeFixedJoint(true);
            BodyPtr bd = bc.createBody(_sr->robot(), _sr->info);
            {
                ra::cnoidRAFile raf;
                raf.history = _sr->history;
                raf.info = _sr->info;
                MappingPtr roboasm = new Mapping();
                raf.historyToMap(roboasm);
                raf.addInfo(roboasm);
                bd->info()->insert("roboasm", roboasm);
            }
            StdBodyWriter writer;
            int mode_ = StdBodyWriter::LinkToOriginalModelFiles;
            if (!!extModelFileModeCombo) {
                mode_ = extModelFileModeCombo->currentData().toInt();
            }
            writer.setExtModelFileMode(mode_);
            writer.writeBody(bd, fname);
            }
        }
    }
    delete dialog;
}
void AssemblerManager::com_load()
{
    DEBUG_PRINT();
    auto dialog = new FileDialog();
    int filter_id = 0;
    dialog->sigFilterSelected().connect( [&filter_id](int i) { filter_id = i; });
    dialog->setWindowTitle("Load assemble history");
    dialog->setFileMode(QFileDialog::ExistingFile);
    dialog->setAcceptMode(QFileDialog::AcceptOpen);
    dialog->setViewMode(QFileDialog::List);
    dialog->setLabelText(QFileDialog::Accept, "Load");
    dialog->setLabelText(QFileDialog::Reject, "Cancel");
    dialog->setOption(QFileDialog::DontConfirmOverwrite);

    // add option panel to Dialog, see BodyItemFileIO.cpp
    QWidget *optionPanel = new QWidget;
    QVBoxLayout *optionVBox = new QVBoxLayout;
    optionVBox->setContentsMargins(0, 0, 0, 0);
    optionPanel->setLayout(optionVBox);

    auto hbox = new QHBoxLayout;
    QCheckBox *renameCheck = new QCheckBox;
    renameCheck->setText("Parts renamed");
    hbox->addWidget(renameCheck);

    optionVBox->addLayout(hbox);
    dialog->insertOptionPanel(optionPanel);

    QStringList filters;
    filters << "roboasm files (*.roboasm)";
    filters << "roboasm files / parts renamed (*.roboasm)";
    filters << "Any files (*)";
    filters << "Any files / parts renamed (*)";
    dialog->setNameFilters(filters);

    if(dialog->exec() == QDialog::Accepted) {
        DEBUG_STREAM(" accepted: " << filter_id);
        bool rename_ = false;
        if(!!renameCheck) {
            rename_ = renameCheck->isChecked();
        }
        if( filter_id == 1 || filter_id == 3 ) rename_ = true;
        auto fnames = dialog->selectedFiles();
        std::string fname = fnames.front().toStdString();
        if(fname.size() > 0) {
            loadRoboasm(fname, rename_);
        }
    }
    delete dialog;
}
//
static bool generated_reader(Mapping *mp, PartsTabInfo &result)
{
  { // read string name
    bool res = mp->read("name", result.name);
    // if (!res)
  } // end read
  { // read std::vector<string> parts
    ValueNode *v;
    if ((v = mp->find("parts"))->isValid() && (v->isListing())) {
      Listing *lt = v->toListing();
      for(auto it = lt->begin(); it != lt->end(); it++) {
        std::string val;
        bool res = (*it)->read(val);
        if (res) { result.parts.push_back(val); } else { }
      }
    }
  } // end read
  return true;
} // bool generated_reader(Mapping *mp, PartsTabInfo &result)
static bool generated_reader(Mapping *mp, PanelSettings &result)
{
  { // read std::vector<PartsTabInfo> tab_list
    ValueNode *v;
    if ((v = mp->find("tab_list"))->isValid() && (v->isListing())) {
      Listing *lt = v->toListing();
      for(auto it = lt->begin(); it != lt->end(); it++) {
        if ((*it)->isMapping()) {
          PartsTabInfo val;
          Mapping *nmp = (*it)->toMapping();
          bool res = generated_reader(nmp, val);
          if (res) { result.tab_list.push_back(val); } else { }
        }
      }
    }
  } // end read
  { // read std::vector<string> combo_list
    ValueNode *v;
    if ((v = mp->find("combo_list"))->isValid() && (v->isListing())) {
      Listing *lt = v->toListing();
      for(auto it = lt->begin(); it != lt->end(); it++) {
        std::string val;
        bool res = (*it)->read(val);
        if (res) { result.combo_list.push_back(val); } else { }
      }
    }
  } // end read
  return true;
} // bool generated_reader(Mapping *mp, PanelSettings &result)
bool AssemblerManager::parseButtonYaml(const std::string &filename, PanelSettings &_res)
{
    YAMLReader yaml_reader;
    if (! yaml_reader.load(filename)) {
        std::cerr << "File Loading error : " << filename << std::endl;
        return false;
    }
    bool ret = false;
    for(int i = 0; i < yaml_reader.numDocuments(); i++) {
        ValueNode *val = yaml_reader.document(i);
        if ( val->isMapping() ) {
            std::string key = "PanelSettings";
            ValueNode *target = val->toMapping()->find(key);
            if(target->isValid() && target->isMapping()) {
                ret = generated_reader(target->toMapping(), _res);
            }
            break;
        }
    }
    if (!ret) {
        std::cerr << "failed parseButtonYaml" << std::endl;
        return false;
    }
    return true;
}
bool AssemblerManager::isRunningAssembler()
{
    if (_current_mode == ASSEMBLER) {
        return true;
    }
    return false;
}
void AssemblerManager::com_swap_mode()
{
    DEBUG_STREAM(" swap_mode : " << _current_mode);
    switch(_current_mode) {
    case ASSEMBLER:
    {
        ProjectManager *pm = ProjectManager::instance();
        if (!!pm) {
            std::string fname_ = original_project();
            if(fname_.size() > 0) {
                pm->loadProject(fname_, nullptr);
                _current_mode = CNOID;
            }
        }
    }
    break;
    case CNOID:
    {
        ProjectManager *pm = ProjectManager::instance();
        if (!!pm) {
            std::string fname_ = assembler_project();
            if(fname_.size() > 0) {
                pm->loadProject(fname_, nullptr);
                _current_mode = ASSEMBLER;
            }
        }
    }
    break;
    default:
        ERROR_STREAM(" mode error : " << _current_mode);
        break;
    }
}
