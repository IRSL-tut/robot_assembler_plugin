#ifndef CNOID_ROBOT_ASSEMBLER_PLUGIN_PARTS_VIEW_H
#define CNOID_ROBOT_ASSEMBLER_PLUGIN_PARTS_VIEW_H
#include <cnoid/View>

namespace cnoid {
class AssemblerPartsView : public View
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
