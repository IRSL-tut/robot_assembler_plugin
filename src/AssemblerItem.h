#ifndef CNOID_ROBOT_ASSEMBLER_PLUGIN_ITEM_H
#define CNOID_ROBOT_ASSEMBLER_PLUGIN_ITEM_H

#include <cnoid/Item>
// #include <cnoid/LocatableItem>
#include <cnoid/RenderableItem>
#include "RobotAssembler.h"
#include "RobotAssemblerHelper.h"

#include "exportdecl.h"

namespace ra = cnoid::robot_assembler;

namespace cnoid {

class ItemManager;
class AssemblerItem;
class AssemblerManager;
typedef ref_ptr<AssemblerItem> AssemblerItemPtr;

//public LocatableItem
class CNOID_EXPORT AssemblerItem : public Item, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    AssemblerItem();
    AssemblerItem(const std::string& name);
    AssemblerItem(const AssemblerItem& org);
    virtual ~AssemblerItem();

    virtual bool setName(const std::string& name) override;

    static AssemblerItemPtr createItem(ra::RoboasmRobotPtr _rb, AssemblerManager *_ma = nullptr);

    // RenderableItem function
    virtual SgNode* getScene() override;

    float transparency() const;
    void setTransparency(float t);

protected:
    virtual Item* doDuplicate() const override;
    virtual bool doAssign(const Item* item) override;
    virtual void onTreePathChanged() override;
    virtual void onConnectedToRoot() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
