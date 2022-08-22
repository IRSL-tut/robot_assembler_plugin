#ifndef CNOID_ROBOT_ASSEMBLER_PLUGIN_PARTS_VIEW_H
#define CNOID_ROBOT_ASSEMBLER_PLUGIN_PARTS_VIEW_H
#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {
class CNOID_EXPORT AssemblerPartsView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static AssemblerPartsView* instance(); // instance??

    AssemblerPartsView();
    virtual ~AssemblerPartsView();
private:
    class Impl;
    Impl* impl;
};
}
#endif
