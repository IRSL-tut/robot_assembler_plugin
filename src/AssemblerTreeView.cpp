#include "AssemblerTreeView.h"
#include "AssemblerManager.h"
#include "RobotAssembler.h"
#include <cnoid/ComboBox>
#include <cnoid/TreeWidget>
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>

#include <QStyle>
#include <QLabel>
#include <QBoxLayout>
#include <QAbstractItemModel>
#include <cnoid/LineEdit>

//#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;
namespace ra = cnoid::robot_assembler;

namespace cnoid {
class AssemblerTreeItem : public QTreeWidgetItem
{
public:
    AssemblerTreeItem(ra::RoboasmCoordsPtr _elm) : QTreeWidgetItem()
    {
        element = _elm;
    }
    virtual ~AssemblerTreeItem()
    {
        //DEBUG_STREAM(" " << this);
    }
    ra::RoboasmCoordsPtr element;
    virtual QVariant data(int column, int role) const override
    {
        //DEBUG_STREAM(" col: " << column << ", role: " << role);
        return QTreeWidgetItem::data(column, role);
    }
    virtual void setData(int column, int role, const QVariant& value) override
    {
        //DEBUG_STREAM(" col: " << column << ", role: " << role);
        return QTreeWidgetItem::setData(column, role, value);
    }
};
class AssemblerTreeView::Impl
{
public:
    Impl(AssemblerTreeView *_self);
    ~Impl();
    AssemblerTreeView* self;
    TreeWidget tree;
    ComboBox modeCombo;
    AssemblerManager *manager;
    AssemblerTreeItem *top_item;
    ra::RoboasmRobotPtr current_selected;
    int mode;
    bool emit_selected;

    void setMode(int _mode, bool doUpdate);
    void robotSelected(ra::RoboasmRobotPtr _rb, bool on);
    void updateTree(ra::RoboasmRobotPtr _rb);
    void createTree(ra::RoboasmRobotPtr _rb,
                    std::function<bool(ra::RoboasmCoordsPtr _rb)> test = nullptr);
    void createSubTree(QTreeWidgetItem *_p_itm, ra::RoboasmCoordsPtr _p_coords,
                       std::function<bool(ra::RoboasmCoordsPtr _rb)> test = nullptr);
    void coordsSelected(ra::RoboasmCoordsPtr _coords);
};
}
void AssemblerTreeView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<AssemblerTreeView>("AssemblerTreeView", "AssemblerTree");
}
AssemblerTreeView* AssemblerTreeView::instance()
{
    static AssemblerTreeView* instance_ = ViewManager::getOrCreateView<AssemblerTreeView>();
    return instance_;
}
//
AssemblerTreeView::AssemblerTreeView()
{
    impl = new Impl(this);
}
AssemblerTreeView::~AssemblerTreeView()
{
    delete impl;
}
//
AssemblerTreeView::Impl::Impl(AssemblerTreeView *_self):
    self(_self), emit_selected(false), manager(nullptr), top_item(nullptr)
{
    manager = AssemblerManager::instance();
    if(!!manager) {
        manager->sigRobotSelected().connect(
            [this](ra::RoboasmRobotPtr _rb, bool on) { robotSelected(_rb, on); } );
    }
    self->setDefaultLayoutArea(BottomLeftArea);

    auto vbox = new QVBoxLayout();
    vbox->setSpacing(0);

    modeCombo.addItem("ALL");
    modeCombo.addItem("Parts");
    modeCombo.addItem("Parts + Actuator");
    modeCombo.addItem("Parts + ConnectingPoint");
    modeCombo.addItem("Actuator");
    modeCombo.addItem("ConnectingPoint");
    modeCombo.addItem("Link");
    modeCombo.addItem("Link + Actuator");

    vbox->addWidget(&modeCombo);

    modeCombo.setCurrentIndex(1);
    mode = 1;
    //tree settings
    //tree.setColumnCount(1);
    tree.setSelectionMode(QAbstractItemView::ExtendedSelection);
    tree.setIndentation(12);
    //tree.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    //Qt::ScrollBarAlwaysOn
    //Qt::ScrollBarAsNeeded
    tree.setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    tree.setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);

    vbox->addWidget(&tree);

    self->setLayout(vbox);

    modeCombo.sigCurrentIndexChanged().connect(
        [this](int index){ setMode(index, true); });

    tree.sigCurrentItemChanged().connect (
        [this](QTreeWidgetItem* cur, QTreeWidgetItem* prev) {
            DEBUG_STREAM(" cur: " << (!!cur ? cur->text(0).toStdString() : "none") << ", prev:" << (!!prev ? prev->text(0).toStdString() : "none"));
            AssemblerTreeItem *ati = dynamic_cast<AssemblerTreeItem*>(cur);
            if(!!ati && !!(ati->element)) {
                DEBUG_STREAM(" coords: " << ati->element->name());
                // pointClicked
                emit_selected = true;
                manager->coordsSelected(ati->element);
                emit_selected = false;
            }
        } );
#if 0
    tree.sigItemChanged().connect (
        [this](QTreeWidgetItem* item, int column) {
            DEBUG_STREAM(" changed item: " << (!!item ? item->text(0).toStdString() : "none"));
        } );
    tree.sigItemClicked().connect (
        [this](QTreeWidgetItem* item, int column) {
            DEBUG_STREAM(" clicked item: " << (!!item ? item->text(0).toStdString() : "none"));
        } );
    tree.sigItemPressed().connect (
        [this](QTreeWidgetItem* item, int column) {
            DEBUG_STREAM(" pressed item: " << (!!item ? item->text(0).toStdString() : "none"));
        } );
    tree.sigItemCollapsed().connect (
        [this](QTreeWidgetItem* item) {
            DEBUG_STREAM(" collapsed item: " << (!!item ? item->text(0).toStdString() : "none"));
        } );
    tree.sigItemSelectionChanged().connect (
        [this]() { DEBUG_STREAM(" selection changed"); } );
#endif

    manager->sigCoordsSelected().connect(
        [this] (ra::RoboasmCoordsPtr _coords) { coordsSelected(_coords); } );
}
AssemblerTreeView::Impl::~Impl()
{
}
void AssemblerTreeView::Impl::setMode(int _mode, bool doUpdate)
{
    mode = _mode;
    tree.clear();
    top_item = nullptr;
    DEBUG_STREAM(" mode: " << mode << ", updata: " << doUpdate);
    if(!!current_selected) {
        updateTree(current_selected);
    }
}
void AssemblerTreeView::Impl::robotSelected(ra::RoboasmRobotPtr _rb, bool on)
{
    DEBUG_STREAM(" robot_selected: " << _rb->name() << ", on: "<< on);
    if(on) {
        if(current_selected == _rb) return;
        current_selected = _rb;
        if(!!current_selected) {
            updateTree(current_selected);
        }
    } else { // not on
        if(current_selected == _rb) { // current_selected may be deleted
            tree.clear();
            top_item = nullptr;
            current_selected = nullptr;
        }
    }
}
void AssemblerTreeView::Impl::updateTree(ra::RoboasmRobotPtr _rb)
{
    tree.clear();
    top_item = nullptr;
    switch(mode) {
    case 0: // all
        createTree(_rb);
        break;
    case 1: // parts
        createTree(_rb, [](ra::RoboasmCoordsPtr _rb) { return _rb->isParts(); } );
        break;
    case 2: // parts + act
        createTree(_rb, [](ra::RoboasmCoordsPtr _rb) {
                if (_rb->isParts()) return true;
                ra::RoboasmConnectingPoint *cp_ = _rb->toConnectingPoint();
                if(!!cp_ && cp_->isConnected() && cp_->isActuator()) return true;
                return false; } );
        break;
    case 3: // parts + conn
        createTree(_rb, [](ra::RoboasmCoordsPtr _rb) {
                if (_rb->isParts()) return true;
                ra::RoboasmConnectingPoint *cp_ = _rb->toConnectingPoint();
                if(!!cp_ && cp_->isConnected()) return true;
                return false; } );
        break;

    case 4: // actuator
        createTree(_rb, [](ra::RoboasmCoordsPtr _rb) {
                ra::RoboasmConnectingPoint *cp_ = _rb->toConnectingPoint();
                if(!!cp_ && cp_->isConnected() && cp_->isActuator()) return true;
                return false; } );
        break;
    case 5: // connecting
        createTree(_rb, [](ra::RoboasmCoordsPtr _rb) {
                ra::RoboasmConnectingPoint *cp_ = _rb->toConnectingPoint();
                if(!!cp_ && cp_->isConnected()) return true;
                return false; } );
        break;
    case 6: // link
        createTree(_rb, [](ra::RoboasmCoordsPtr _rb) {
                if (_rb->isParts()) {
                    if(_rb->parent()->isRobot()) return true;
                    if (_rb->parent()->toConnectingPoint()->isActuator() ||
                        _rb->parent()->parent()->toConnectingPoint()->isActuator()) return true; }
                return false; } );
        break;
    case 7: // link + act
        createTree(_rb, [](ra::RoboasmCoordsPtr _rb) {
                if (_rb->isParts()) {
                    if(_rb->parent()->isRobot()) return true;
                    if (_rb->parent()->toConnectingPoint()->isActuator() ||
                        _rb->parent()->parent()->toConnectingPoint()->isActuator()) return true;
                } else {
                    ra::RoboasmConnectingPoint *cp_ = _rb->toConnectingPoint();
                    if(!!cp_ && cp_->isConnected() && cp_->isActuator()) return true; }
                return false; } );
        break;
    }
}
void AssemblerTreeView::Impl::createTree(ra::RoboasmRobotPtr _rb,
                                         std::function<bool(ra::RoboasmCoordsPtr _rb)> test)
{
    DEBUG_STREAM(" createTree: " << _rb->name());
    AssemblerTreeItem *itm = new AssemblerTreeItem(_rb);
    itm->setText(0, _rb->name().c_str());
    tree.addTopLevelItem(itm);
    top_item = itm;
    itm->setExpanded(true);
    createSubTree(itm, _rb, test);
}
void AssemblerTreeView::Impl::createSubTree(QTreeWidgetItem *_p_itm,
                                            ra::RoboasmCoordsPtr _p_coords,
                                            std::function<bool(ra::RoboasmCoordsPtr _rb)> test)
{
    ra::coordsPtrList lst;
    _p_coords->directDescendants(lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        if (!test || test(*it)) {
            AssemblerTreeItem *itm = new AssemblerTreeItem(*it);
            itm->setText(0, (*it)->name().c_str());
            _p_itm->addChild(itm);
            itm->setExpanded(true);
            createSubTree(itm, (*it), test);
        } else {
            createSubTree(_p_itm, (*it), test);
        }
    }
}
void AssemblerTreeView::Impl::coordsSelected(ra::RoboasmCoordsPtr _coords)
{
    DEBUG_STREAM(" emit_selected: " << emit_selected);
    if(emit_selected) return;
    // find _coords
    // find selected -> false;
    // itm->setSelected(true);
    if(!!top_item) {


    }
}
