#include "AssemblerSwapModeBar.h"
#include "AssemblerManager.h"

//#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;

namespace cnoid {

class AssemblerSwapModeBar::Impl
{
public:
    Impl(AssemblerSwapModeBar* _self);

    AssemblerSwapModeBar *self;
    AssemblerManager *manager;
    ToolButton *button;

    void clickedAction();
};

}

AssemblerSwapModeBar* AssemblerSwapModeBar::instance()
{
    static AssemblerSwapModeBar* instance = new AssemblerSwapModeBar;
    return instance;
}

AssemblerSwapModeBar::AssemblerSwapModeBar()
    : ToolBar("AssemblerSwapModeBar")
{
    impl = new Impl(this);
}

AssemblerSwapModeBar::~AssemblerSwapModeBar()
{
    delete impl;
}

AssemblerSwapModeBar::Impl::Impl(AssemblerSwapModeBar* _self)
{
    manager = AssemblerManager::instance();

    self = _self;

    QIcon qi(":/RA/icon/cnoid.svg");

    button = self->addButton(qi);
    button->setToolTip("Swap Mode (choreonoid <-> assembler)");
    button->sigClicked().connect([this](){ this->clickedAction(); });
}

void AssemblerSwapModeBar::Impl::clickedAction()
{
    manager->com_swap_mode();
    if(manager->isRunningAssembler()) {
        QIcon qi(":/RA/icon/cnoid.svg");
        button->setIcon(qi);
    } else {
        QIcon qi(":/RA/icon/roboasm.svg");
        button->setIcon(qi);
    }
}
