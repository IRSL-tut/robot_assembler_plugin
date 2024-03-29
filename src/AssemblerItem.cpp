#include "AssemblerItem.h"
#include "AssemblerManager.h"

#include <cnoid/RootItem>
//#include <cnoid/LazySignal>
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>

#include <fmt/format.h>
#include <bitset>
#include <algorithm>

//#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace std;
using namespace cnoid;
using namespace cnoid::robot_assembler;

using fmt::format;

namespace cnoid {
class AssemblerItem::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AssemblerItem* self;
    SgNodePtr scene;

    Impl(AssemblerItem* _self);
    Impl(AssemblerItem* _self, const Impl& org);

    ~Impl();

    void itemSelected(bool on)
    {
        if(!!manager) manager->itemSelected(self, on);
    }
    void setTransparency(float t);

    //Signal<void(int flags)> sigModelUpdated;

    float transparency;

    AssemblerManager *manager;
};
}

AssemblerItemPtr AssemblerItem::createItem(RoboasmRobotPtr _rb, AssemblerManager *_ma)
{
    AssemblerItemPtr ret(new AssemblerItem(_rb->name()));
    ret->impl->manager = _ma; //

    RASceneRobotBasePtr rb_scene(new RASceneRobotBase(_rb, _ma));
    ret->impl->scene = dynamic_pointer_cast<SgNode>(rb_scene); //

    return ret;
}
void AssemblerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager* im = &ext->itemManager();
    im->registerClass<AssemblerItem>("AssemblerItem");
}

////
AssemblerItem::AssemblerItem()
{
    setAttributes(FileImmutable | Reloadable);//TODO
    impl = new Impl(this);
}

AssemblerItem::AssemblerItem(const std::string& name)
    : AssemblerItem()
{
    AssemblerItem::setName(name);
}

AssemblerItem::AssemblerItem(const AssemblerItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
    setChecked(org.isChecked());
}

AssemblerItem::~AssemblerItem()
{
    DEBUG_PRINT();
    delete impl;
}

//// Impl
AssemblerItem::Impl::Impl(AssemblerItem* _self)
    : self(_self)
{
    self->sigSelectionChanged().connect([this](bool on){ itemSelected(on); });
}

AssemblerItem::Impl::Impl(AssemblerItem* _self, const Impl& org)
    : self(_self)
{
}

AssemblerItem::Impl::~Impl()
{
}

//// protected / override Item Class
Item* AssemblerItem::doDuplicate() const
{
    DEBUG_PRINT();
    return new AssemblerItem(*this);
}
bool AssemblerItem::doAssign(const Item* srcItem)
{
    DEBUG_PRINT();
    return false;
    //??
    //return impl->doAssign(srcItem);
}
void AssemblerItem::onTreePathChanged()
{
    DEBUG_PRINT();
}
void AssemblerItem::onConnectedToRoot()
{
    DEBUG_PRINT();
}
void AssemblerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    DEBUG_PRINT();
}
#if 0
void AssemblerItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    DEBUG_PRINT();
    putProperty(_("Model name"), body->modelName());
    putProperty(_("Num links"), body->numLinks());
    putProperty(_("Num joints"), body->numJoints());
    putProperty(_("Num devices"), (int)body->devices().size());
    putProperty(_("Root link"), body->rootLink()->name());
    putProperty(_("Base link"), currentBaseLink ? currentBaseLink->name() : "none");
    putProperty.decimals(3)(_("Mass"), body->mass());
    putProperty(_("Static"), body->isStaticModel(),
                [&](bool on){ return on ? makeBodyStatic() : makeBodyDynamic(); });
    putProperty(_("Collision detection"), isCollisionDetectionEnabled,
                [&](bool on){ return setCollisionDetectionEnabled(on); });
    putProperty(_("Self-collision detection"), isSelfCollisionDetectionEnabled,
                [&](bool on){ return setSelfCollisionDetectionEnabled(on); });
    putProperty(_("Location editable"), self->isLocationEditable(),
                [&](bool on){ setLocationEditable(on, true); return true; });
    putProperty(_("Scene sensitive"), self->isSceneSensitive(),
                [&](bool on){ self->setSceneSensitive(on); return true; });
    putProperty.range(0.0, 0.9).decimals(1);
    putProperty(_("Transparency"), transparency,
                [&](float value){ setTransparency(value); return true; });
    putProperty(_("Visible link selection"), self->isVisibleLinkSelectionMode_,
                changeProperty(self->isVisibleLinkSelectionMode_));

    if(isAttachable()){
        putProperty(_("Enable attachment"), isAttachmentEnabled,
                    [&](bool on){ self->setAttachmentEnabled(on, false); return true; });
    }
}
#endif
//// protected

// override
bool AssemblerItem::setName(const std::string& name)
{
    DEBUG_STREAM(" name: " << name);
    if(!!(impl->scene)) {
        RASceneRobotBasePtr p = dynamic_pointer_cast<RASceneRobotBase>(impl->scene);
        if(!!p) {
            p->setName(name);
            // p->robot->name = 
        }
    }
    return Item::setName(name);
}

////
SgNode* AssemblerItem::getScene()
{
    return impl->scene;
}

//// transp
float AssemblerItem::transparency() const
{
    return impl->transparency;
}
void AssemblerItem::setTransparency(float t)
{
    impl->setTransparency(t);
}
void AssemblerItem::Impl::setTransparency(float t)
{
    //[todo]
}
