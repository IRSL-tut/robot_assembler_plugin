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
    void addToggleButton(const char *icon, const char *tooltip, std::function<void(bool)> func);
    void addButton(QIcon &icon, const char *tooltip, std::function<void()> func);
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
    //addButton("SaveModel", "Save History", // floppy H
    //[this](){ manager->com_save_history(); } );
    {
        QIcon qi(":/Body/icon/storepose.svg");
        addButton(qi, "Save History", // floppy L
                  [this](){ manager->com_save_history(); } );
    }
    addButton("SaveModel", "Save Model",  // floppy M
              [this](){ manager->com_save_model(); } );
    addButton("<<", "Align decrement", // >
              [this](){ manager->com_align_back(); } );
    addToggleButton("o", "Attach Ordered (the first clicked part is a root)", // o
                    [this](bool _in){ manager->com_ordered_attach(_in); } );
    addButton(">>", "Align increment", // <
              [this](){ manager->com_align(); } );
    addButton("UnAlign", "Undo Align", // ^
              [this](){ manager->com_unalign(); } );
    addButton("Attach", "Attach", // A
              [this](){ manager->com_attach(); } );
    addButton("Undo", "Undo", // U
              [this](){ manager->com_undo(); } );
    {
        QIcon qi(":/Body/icon/restorepose.svg");
        addButton(qi, "Load History", // floppy L
                  [this](){ manager->com_load(); } );
    }
    addButton("DeleteAll", "Delete All", // Delete all
              [this](){ manager->com_delete_all(); } );
    addButton("AddSettings", "Load additional settings",
              [this](){ manager->load_additional_settings(); } );
}

void AssemblerBar::Impl::addButton(const char *icon, const char *tooltip, std::function<void()> func)
{
    ToolButton *button;
    button = self->addButton(icon);
    button->setToolTip(tooltip);
    button->sigClicked().connect(func);

    buttons.push_back(button);
}
void AssemblerBar::Impl::addToggleButton(const char *icon, const char *tooltip, std::function<void(bool)> func)
{
    ToolButton *button;
    button = self->addToggleButton(icon);
    button->setToolTip(tooltip);
    //button->sigClicked().connect(func);
    button->sigToggled().connect(func);

    buttons.push_back(button);
}
void AssemblerBar::Impl::addButton(QIcon &icon, const char *tooltip, std::function<void()> func)
{
    ToolButton *button;
    button = self->addButton(icon);
    button->setToolTip(tooltip);
    button->sigClicked().connect(func);

    buttons.push_back(button);
}
