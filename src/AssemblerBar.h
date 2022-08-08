#ifndef CNOID_ROBOT_ASSEMBLER_PLUGIN_BAR_H
#define CNOID_ROBOT_ASSEMBLER_PLUGIN_BAR_H

#include <cnoid/ToolBar>

namespace cnoid {

class AssemblerBar : public ToolBar
{
public:
    static AssemblerBar* instance();
    virtual ~AssemblerBar();

    SignalProxy<void(int flags)> sigAlignDecl() { return sigAlignDecl_func; }
    SignalProxy<void(int flags)> sigAlignIncl() { return sigAlignIncl_func; }
    SignalProxy<void(int flags)> sigUnAlign()   { return sigUnAlign_func; }

    SignalProxy<void(int flags)> sigAttach() { return sigAttach_func; }
    SignalProxy<void(int flags)> sigUndo()   { return sigUndo_func; }
    SignalProxy<void(int flags)> sigRedo()   { return sigRedo_func; }

    SignalProxy<void(int flags)> sigWrite()     { return sigWrite_func; }
    SignalProxy<void(int flags)> sigWriteURDF() { return sigWriteURDF_func; }
    SignalProxy<void(int flags)> sigDeleteAll() { return sigDeleteAll_func; }

private:
    AssemblerBar();

    Signal<void(int flags)> sigAlignDecl_func;
    Signal<void(int flags)> sigAlignIncl_func;
    Signal<void(int flags)> sigUnAlign_func;

    Signal<void(int flags)> sigAttach_func;
    Signal<void(int flags)> sigUndo_func;
    Signal<void(int flags)> sigRedo_func;

    Signal<void(int flags)> sigWrite_func;
    Signal<void(int flags)> sigWriteURDF_func;
    Signal<void(int flags)> sigDeleteAll_func;

    class Impl;
    Impl* impl;
};

}
#endif
