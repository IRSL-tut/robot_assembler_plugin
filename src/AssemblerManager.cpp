#include "AssemblerManager.h"

#include "RobotAssemblerBody.h"
#include <cnoid/StdBodyWriter>

#include <cnoid/MenuManager>
#include <cnoid/RootItem>
#include <cnoid/ItemList>
#include <cnoid/FileDialog>
#include <cnoid/SceneView>
#include <cnoid/SceneWidget>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>

#include <vector>

//#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;
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
AssemblerManager::AssemblerManager()
{
    DEBUG_PRINT();
    impl = new Impl(this);

    uniq_id = SceneWidget::issueUniqueCustomModeId();
    SceneView::instance()->sceneWidget()->activateCustomMode(this, uniq_id);
}
AssemblerManager::~AssemblerManager()
{
    delete impl;
}
void AssemblerManager::partsButtonClicked(const std::string &_name)
{
    DEBUG_STREAM( " index: " << _name );
    //PushButton *bp = partsButtons[index];
    //std::string name = bp->text().toStdString();
    const BoundingBox &bb = SceneView::instance()->sceneWidget()->scene()->boundingBox();
    Vector3 cent = bb.center();
    Vector3 size = bb.size();
    DEBUG_STREAM(" cent: " << cent(0) << ", " << cent(1) << ", " << cent(2) );
    DEBUG_STREAM(" size: " << size(0) << ", " << size(1) << ", " << size(2) );
    std::string rb_name;
    if (srobot_set.size() == 0) {
        rb_name = "AssembleRobot";
    } else {
        rb_name = _name;
    }
    AssemblerItemPtr itm = AssemblerItem::createItem(rb_name, _name, roboasm, this);
    if (!!itm) {
        ra::RASceneRobot* rb_scene = dynamic_cast<ra::RASceneRobot*> (itm->getScene());
        const BoundingBox &rb_bb = rb_scene->boundingBox();
        Vector3 rb_cent = rb_bb.center();
        Vector3 rb_size = rb_bb.size();
        DEBUG_STREAM(" rb_cent: " << rb_cent(0) << ", " << rb_cent(1) << ", " << rb_cent(2) );
        DEBUG_STREAM(" rb_size: " << rb_size(0) << ", " << rb_size(1) << ", " << rb_size(2) );
        coordinates cds(Vector3(0, cent(1) + size(1)/2 +  rb_size(1)/2, 0));
        rb_scene->setCoords(cds);

        current_align_configuration = -1;
        clickedPoint0 = nullptr;
        clickedPoint1 = nullptr;
        selectable_spoint_set.clear();
        itm->setChecked(true);
        RootItem::instance()->addChildItem(itm);

        //
        updateRobots();
        clearAllPoints();
        notifyUpdate();
        SceneView::instance()->sceneWidget()->viewAll();
    } else {
        ERROR_STREAM(" item create error : " << rb_name << " / " << _name);
    }
}
int AssemblerManager::pointClicked(ra::RASceneConnectingPoint *_cp)
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
    if(modified) {
        updateConnectingPoints();
        for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
            (*it)->notifyUpdate(SgUpdate::Added | SgUpdate::Removed | SgUpdate::Modified);
        }
    }
    return 1;
}
int AssemblerManager::partsClicked(ra::RASceneParts *_pt)
{
    DEBUG_STREAM(" " << _pt->name() );
    return 1;
}
void AssemblerManager::updateConnectingPoints()
{
    //clickedPoint0
    //clickedPoint1
    if(!!clickedPoint0 && !!clickedPoint1) {
        DEBUG_STREAM(" state 0 : both clicked" );
        //selectable_spoint_set.clear();
        // can match clickedPoint0/clickedPoint1
        bool can_match = roboasm->canMatch(clickedPoint0->point(), clickedPoint1->point());
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
            if(roboasm->canMatch(cp_, (*pit)->point())) {
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
    selectable_spoint_set.clear();
    clearAllPoints();
    notifyUpdate();
    SceneView::instance()->sceneWidget()->viewAll();
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
    selectable_spoint_set.clear();
    clearAllPoints();
    notifyUpdate();
    SceneView::instance()->sceneWidget()->viewAll();
}
void AssemblerManager::attachRobots(bool _just_align)
{
    DEBUG_PRINT();
    if(!clickedPoint0 || !clickedPoint1) {
        DEBUG_STREAM( " require 2 clicked point" );
        return;
    }

    ra::RASceneConnectingPoint *cp0 = clickedPoint0;
    ra::RASceneConnectingPoint *cp1 = clickedPoint1;
    if(cp1->scene_robot()->robot()->partsNum() >
       cp0->scene_robot()->robot()->partsNum() ) { // swap 0 and 1
        ra::RASceneConnectingPoint *tmp = cp0;
        cp0 = cp1; cp1 = cp0;
    }

    ra::RoboasmRobotPtr rb0 = cp0->scene_robot()->robot();
    ra::RoboasmRobotPtr rb1 = cp1->scene_robot()->robot();

    DEBUG_STREAM(" rb0: " << rb0->name() << " <=(attach) rb1:" << rb1->name() );
    DEBUG_STREAM(" cp0-point(): " << cp0->point()->name() << " cp1-point()" << cp1->point()->name() );

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
    int target_config_ = current_align_configuration;
    if(_just_align) target_config_++;
    ra::ConnectingConfigurationID ccid = res_match_lst[0]->allowed_configuration[0];
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
    if(!find_) {
        current_align_configuration = 0;
    } else {
        current_align_configuration++;
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
    //cp0->point();
#if 0
    DEBUG_STREAM(" attached !! " );
    {
        coordsPtrList lst;
        rb0->allDescendants(lst);
        int cntr = 0;
        for(auto it = lst.begin(); it != lst.end(); it++) {
            std::cerr << cntr++ << " : " << *(*it) << std::endl;
        }
    }
#endif
    std::string &config_ = ra_settings->listConnectingConfiguration[ccid].name;
    if (cp1->scene_robot()->history.size() == 1) {
        cp0->scene_robot()->attachHistory(
            cp1->scene_robot()->history,
            cp0->point()->parent()->name(), //parent,
            cp0->point()->name(),           //robot_point,
            attached_parts->name(),         //parts_name,
            attached_parts->info->type,     //parts_type,
            cp1->point()->name(),           //parts_point,
            config_);
    } else {
        ra::AttachHistory hist_;
        attached_parts->dumpConnectFromParent(hist_); //??
        cp0->scene_robot()->attachHistory(
            hist_,
            cp0->point()->parent()->name(), //parent,
            cp0->point()->name(),           //robot_point,
            attached_parts->name(),         //parts_name,
            attached_parts->info->type,     //parts_type,
            cp1->point()->name(),           //parts_point,
            config_);
    }
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
bool AssemblerManager::onContextMenuRequest(SceneWidgetEvent* event)
{
    DEBUG_PRINT();
    auto menu = event->contextMenu();

    menu->addSeparator();
    menu->addItem("Align")->sigTriggered().connect(
        [this](){ com_align(); } );
    menu->addItem("UnAlign")->sigTriggered().connect(
        [this](){ com_unalign(); } );
    menu->addSeparator();
    menu->addItem("Attach")->sigTriggered().connect(
        [this](){ com_attach(); } );
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

    std::string initialname = _sr->name() + ".roboasm";
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
            ra::RoboasmFile raf;
            raf.history = _sr->history;
            _sr->robot()->writeConfig(raf.config);
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

    QStringList filters;
    filters << "body files (*.body)";
    filters << "urdf files (*.urdf)";
    filters << "Any files (*)";
    dialog->setNameFilters(filters);

    std::string initialname = _sr->name() + ".body";
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
                    urdf = true;
                    fname += ".urdf";
                } else if (filter_id == 1) {
                    fname += ".body";
                }
            }
        }
        if(fname.size() > 0) {
            // createBody
            ra::RoboasmBodyCreator bc;
            BodyPtr bd = bc.createBody(_sr->robot());
            StdBodyWriter writer;
            writer.writeBody(bd, fname);
            // [TODO] if urdf
        }
    }
    delete dialog;
}

