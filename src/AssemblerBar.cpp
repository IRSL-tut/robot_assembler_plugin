#include "AssemblerBar.h"
#include "AssemblerManager.h"
#include <cnoid/FileDialog>
#include <vector>

//#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;

namespace cnoid {

class AssemblerBar::Impl
{
public:
    Impl(AssemblerBar* _self);

    AssemblerBar *self;
    AssemblerManager *manager;
    std::vector<ToolButton *> buttons;

    void addButton(const char *icon, const char *tooltip, std::function<void()> func);

};

}

AssemblerBar* AssemblerBar::instance()
{
    static AssemblerBar* instance = new AssemblerBar;
    return instance;
}

AssemblerBar::AssemblerBar()
    : ToolBar("AssemblerBar")
{
    impl = new Impl(this);
}

AssemblerBar::~AssemblerBar()
{
    delete impl;
}

AssemblerBar::Impl::Impl(AssemblerBar* _self)
{
    manager = AssemblerManager::instance();

    self = _self;
    addButton(":/Body/icon/storepose.svg", "Save History",
              [this](){ manager->com_save_history(); } );
    addButton(":/Body/icon/storepose.svg", "Save Model",
              [this](){ manager->com_save_model(); } );
    addButton(":/Body/icon/storepose.svg", "Align increment",
              [this](){ manager->com_align(); } );
    addButton(":/Body/icon/storepose.svg", "Align decrement",
              [this](){ manager->com_align_back(); } );
    addButton(":/Body/icon/storepose.svg", "Undo Align",
              [this](){ manager->com_unalign(); } );
    addButton(":/Body/icon/storepose.svg", "Attach",
              [this](){ manager->com_attach(); } );
    addButton(":/Body/icon/storepose.svg", "Undo",
              [this](){ manager->com_undo(); } );
    addButton(":/Body/icon/storepose.svg", "Load History",
              [this](){ manager->com_load(); } );
    addButton(":/Body/icon/storepose.svg", "Delete All",
              [this](){ manager->com_delete_all(); } );
}

void AssemblerBar::Impl::addButton(const char *icon, const char *tooltip, std::function<void()> func)
{
    ToolButton *button;
    button = self->addButton(QIcon(icon));
    button->setToolTip(tooltip);
    button->sigClicked().connect(func);

    buttons.push_back(button);
}
