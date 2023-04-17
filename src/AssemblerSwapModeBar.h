#ifndef CNOID_ROBOT_ASSEMBLER_PLUGIN_SWAP_MODE_H
#define CNOID_ROBOT_ASSEMBLER_PLUGIN_SWAP_MODE_H

#include <cnoid/ToolBar>

namespace cnoid {

class AssemblerSwapModeBar : public ToolBar
{
public:
    static AssemblerSwapModeBar* instance();
    virtual ~AssemblerSwapModeBar();

private:
    AssemblerSwapModeBar();

    class Impl;
    Impl* impl;
};

}
#endif
