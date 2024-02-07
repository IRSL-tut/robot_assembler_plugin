#include "AssemblerView.h"
#include "AssemblerItem.h"
#include "AssemblerManager.h"
#include "RobotAssemblerHelper.h"

#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
//#include <cnoid/ItemTreeView>
#include <cnoid/RootItem>
#include <cnoid/SceneView>
#include <cnoid/SceneWidget>
#include <cnoid/ItemFileIO>
//#include <cnoid/Mapping>

// itemtreeview?
#include <cnoid/Archive>
#include <cnoid/ActionGroup>
#include <cnoid/ConnectionSet>
#include <cnoid/Widget>
#include <cnoid/Buttons>
#include <cnoid/ComboBox>
#include <cnoid/FileDialog>

#include <QLabel>
#include <QStyle>
#include <QBoxLayout>
#include <QScrollArea>
#include <QTabWidget>
#include <QTextEdit>

#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>

#include <vector>
#include <algorithm>

//#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;

namespace ra = cnoid::robot_assembler;
namespace filesystem = cnoid::stdx::filesystem;

static const Vector3f color_red   (1.0f, 0.0f, 0.0f);
static const Vector3f color_green (0.0f, 1.0f, 0.0f);
static const Vector3f color_blue  (0.0f, 0.0f, 1.0f);
static const Vector3f color_black (0.05f, 0.05f, 0.05f);
static const Vector3f color_white (0.95f, 0.95f, 0.95f);
static const Vector3f color_yellow(1.0f, 1.0f, 0.0f);
static const Vector3f color_cyan  (0.0f, 1.0f, 1.0f);
static const Vector3f color_purple(1.0f, 0.0f, 1.0f);
namespace cnoid {
// view manager
class AssemblerView::Impl
{
public:
    AssemblerView* self;
    QLabel targetLabel;

    QVBoxLayout *topLayout;
    QTabWidget *partsTab;
    ComboBox *parts_combo;
    ComboBox *color_combo;
    PushButton *parts_button;
    Impl(AssemblerView* self);

    void initialize(bool config);

    void createButtons(PanelSettings &_settings);
    void createButtonsOrg();
    void createButtonClicked(const std::string &_name = std::string());
    void addPartsCombo(const std::string &name);
    std::vector<PushButton *> partsButtons;
    std::vector<std::string> comboItems;
    //
    AssemblerManager *manager;
};
}

void AssemblerView::initializeClass(ExtensionManager* ext)
{
    DEBUG_PRINT();
    ext->viewManager().registerClass<AssemblerView>("AssemblerView", "AssemblerView");
}
AssemblerView* AssemblerView::instance()
{
    static AssemblerView* instance_ = ViewManager::getOrCreateView<AssemblerView>();
    return instance_;
}

AssemblerView::AssemblerView()
{
    DEBUG_PRINT();
    impl = new Impl(this);
}
AssemblerView::~AssemblerView()
{
    delete impl;
}
void AssemblerView::onActivated()
{
    DEBUG_PRINT();
}
void AssemblerView::onDeactivated()
{
    DEBUG_PRINT();
}
void AssemblerView::onAttachedMenuRequest(MenuManager& menu)
{
    DEBUG_PRINT();
}
bool AssemblerView::storeState(Archive& archive)
{
#if 0
    impl->positionWidget->storeState(&archive);
    switch(impl->positionWidget->targetLinkType()){
    case AssemblerWidget::AnyLink:
        archive.write("target_link_type", "any_link");
        break;
    case AssemblerWidget::RootOrIkLink:
        archive.write("target_link_type", "root_or_ik_link");
        break;
    case AssemblerWidget::IkLink:
        archive.write("target_link_type", "ik_link");
        break;
    }
#endif
    return true;
}
bool AssemblerView::restoreState(const Archive& archive)
{
#if 0
    impl->positionWidget->restoreState(&archive);
    string type;
    if(archive.read("target_link_type", type)){
        if(type == "any_link"){
            impl->positionWidget->setTargetLinkType(AssemblerWidget::AnyLink);
        } else if(type == "root_or_ik_link"){
            impl->positionWidget->setTargetLinkType(AssemblerWidget::RootOrIkLink);
        } else if(type == "ik_link"){
            impl->positionWidget->setTargetLinkType(AssemblerWidget::IkLink);
        }
    }
#endif
    return true;
}
void AssemblerView::createButtons(PanelSettings &_settings)
{
    if(!!(impl->manager)) {
        impl->createButtons(_settings);
    }
}
void AssemblerView::addPartsCombo(const std::string &name)
{
    if(!!(impl->manager)) {
        impl->addPartsCombo(name);
    }
}
//// Impl
AssemblerView::Impl::Impl(AssemblerView* self)
    : self(self), partsTab(nullptr), manager(nullptr)
{
    manager = AssemblerManager::instance();
    initialize(false);
    // manager
}
void AssemblerView::Impl::initialize(bool config)
{
    self->setDefaultLayoutArea(MiddleRightArea);

    topLayout = new QVBoxLayout;
    topLayout->setContentsMargins(0, 0, 0, 0);
    topLayout->setSpacing(0);
    self->setLayout(topLayout);

    auto style = self->style();
    int lmargin = style->pixelMetric(QStyle::PM_LayoutLeftMargin);
    int rmargin = style->pixelMetric(QStyle::PM_LayoutRightMargin);
    int tmargin = style->pixelMetric(QStyle::PM_LayoutTopMargin);
    int bmargin = style->pixelMetric(QStyle::PM_LayoutBottomMargin);

    color_combo = new ComboBox();
    color_combo->addItem("default");
    color_combo->addItem("Red");
    color_combo->addItem("Green");
    color_combo->addItem("Blue");
    color_combo->addItem("Black");
    color_combo->addItem("White");
    color_combo->addItem("Yellow");
    color_combo->addItem("Cyan");
    color_combo->addItem("Purple");
    // intent-/saturation-
    // orange, gray silver, lime navy
    parts_combo = new ComboBox();
    parts_combo->addItem("--- not initialized ---"); // test

    parts_button = new PushButton("Create Parts");

    topLayout->addWidget(color_combo);
    topLayout->addWidget(parts_combo);
    topLayout->addWidget(parts_button);

    parts_button->sigClicked().connect([this]() { createButtonClicked(); });
#if 0
    if (!config) {
        return;
    }
    addTabs(config);
#endif
}
void AssemblerView::Impl::createButtons(PanelSettings &_settings)
{
    if(_settings.tab_list.size() == 0 && _settings.combo_list.size() == 0) {
        // No Tab
        createButtonsOrg();
        return;
    }
    if (!!partsTab) {
        partsButtons.clear();
        comboItems.clear();
        delete partsTab;
    }
    if (_settings.tab_list.size() > 0) {
        partsTab = new QTabWidget(self); // parent??
        DEBUG_STREAM(" size: " << _settings.tab_list.size());
        for(auto it = _settings.tab_list.begin();
            it != _settings.tab_list.end(); it++) {
            DEBUG_STREAM(" name: " << (*it).name);
            Widget *wd = new Widget(partsTab);
            QVBoxLayout *qvbox = new QVBoxLayout(wd);
            //
            for(auto nameit = (*it).parts.begin();
                nameit != (*it).parts.end(); nameit++) {
                std::string name_ = (*nameit);
                auto pt_ = manager->ra_settings->mapParts.find(name_); // check exist parts
                if(pt_ == manager->ra_settings->mapParts.end()) continue;
                std::string label_;
                if( pt_->second.class_name.size() > 0 ) {
                    label_ = pt_->second.class_name;
                } else {
                    label_ = name_;
                }
                PushButton *bp = new PushButton(label_.c_str(), partsTab);
                if( pt_->second.description.size() > 0 ) {
                    QString _tip_text(pt_->second.description.c_str());
                    bp->setToolTip(_tip_text);
                }
                bp->sigClicked().connect( [this, name_]() {
                        createButtonClicked(name_); } );
                qvbox->addWidget(bp);
                partsButtons.push_back(bp);
            }
            partsTab->addTab(wd, (*it).name.c_str());
        }
        topLayout->addWidget(partsTab);
    }

    if (parts_combo->count() == 1) {
        parts_combo->removeItem(0);
        parts_combo->addItem("--- choose parts ---");
    }
    comboItems.push_back("dummy");
    for(auto it = _settings.combo_list.begin();
        it != _settings.combo_list.end(); it++) {
        ////
        parts_combo->addItem((*it).c_str());
        comboItems.push_back(*it);
    }
}
void AssemblerView::Impl::createButtonsOrg()
{
    if (!!partsTab) {
        partsButtons.clear();
        delete partsTab;
    }
    DEBUG_STREAM(" manager : " << (!!manager));
    DEBUG_STREAM(" settings : " << (!!(manager->ra_settings)));
    ra::SettingsPtr &ra_settings = manager->ra_settings;
    //RoboasmPtr &roboasm = manager->roboasm;
    std::vector<std::string> pt_lst;
    for(auto pt_ = ra_settings->mapParts.begin();
        pt_ != ra_settings->mapParts.end(); pt_++) {
        pt_lst.push_back(pt_->first);
    }
    std::sort(pt_lst.begin(), pt_lst.end());
    int parts_num = pt_lst.size();
    int tab_num = ((parts_num - 1) / 10) + 1;

    targetLabel.setText("---initialized---");

    partsTab = new QTabWidget(self); // parent??
    //// tabbed
    //auto hbox = new QHBoxLayout;

    int parts_index = 0;
    auto parts = pt_lst.begin();
    for(int tab_idx = 0; tab_idx < tab_num; tab_idx++) {
        Widget *wd = new Widget(partsTab);
        QVBoxLayout *qvbox = new QVBoxLayout(wd);
        //
        for(int j = 0; j < 10; j++) {
            if (parts != pt_lst.end()) {
                std::string name_ = (*parts);
                auto pt_ = manager->ra_settings->mapParts.find(name_); // check exist parts
                if(pt_ == manager->ra_settings->mapParts.end()) continue;
                std::string label_;
                if( pt_->second.class_name.size() > 0 ) {
                    label_ = pt_->second.class_name;
                } else {
                    label_ = name_;
                }
                PushButton *bp = new PushButton(label_.c_str(), partsTab);
                bp->sigClicked().connect( [this, name_]() {
                        createButtonClicked(name_); } );
                qvbox->addWidget(bp);
                partsButtons.push_back(bp);
                parts_index++;
            } else {
                break;
            }
            parts++;
        }
        std::string tab_name = "Tab";
        partsTab->addTab(wd, tab_name.c_str());
    }
    //

    //
    topLayout->addWidget(partsTab);
}
void AssemblerView::Impl::createButtonClicked(const std::string &_name)
{
    int color_id = color_combo->currentIndex();
    DEBUG_STREAM(" color: " << color_id);
    const std::string *nm_ = nullptr;
    if (_name.size() == 0) {
        int parts_id = parts_combo->currentIndex();
        if(parts_id > 0 && parts_id < comboItems.size()) {
            nm_ = &comboItems[parts_id];
        }
    } else {
        nm_ = &_name;
    }
    if(!!nm_ && nm_->size() > 0) {
        Vector3f col_ = Vector3f::Zero();
        switch(color_id) {
        case 1: // Red
        { col_ = color_red; break; }
        case 2: // Green
        { col_ = color_green; break; }
        case 3: // Blue
        { col_ = color_blue; break; }
        case 4: // Black
        { col_ = color_black; break; }
        case 5: // White
        { col_ = color_white; break; }
        case 6: // Yellow
        { col_ = color_yellow; break; }
        case 7: // Cyan
        { col_ = color_cyan; break; }
        case 8: // Purple
        { col_ = color_purple; break; }
        }
        manager->partsButtonClicked(*nm_, col_);
    }
}
void AssemblerView::Impl::addPartsCombo(const std::string &name)
{
    parts_combo->addItem(name.c_str());
    comboItems.push_back(name);
}
