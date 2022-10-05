#ifndef CNOID_ROBOT_ASSEMBLER_PLUGIN_TREE_VIEW_H
#define CNOID_ROBOT_ASSEMBLER_PLUGIN_TREE_VIEW_H

#include <cnoid/View>
#include "RobotAssembler.h"

namespace ra = cnoid::robot_assembler;

namespace cnoid {

class AssemblerTreeView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static AssemblerTreeView* instance(); // instance??

    AssemblerTreeView();
    virtual ~AssemblerTreeView();

private:
    class Impl;
    Impl* impl;
};

}

#endif
