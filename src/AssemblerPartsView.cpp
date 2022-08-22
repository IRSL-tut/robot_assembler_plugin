#include "AssemblerPartsView.h"
#include "RobotAssembler.h"
#include "AssemblerManager.h"
#include <cnoid/ViewManager>
#include <cnoid/Widget>
#include <cnoid/LineEdit>
#include <cnoid/Buttons>
#include <QGridLayout>
#include <QLabel>

#include <sstream>
#include <iomanip>
//#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;
namespace ra = cnoid::robot_assembler;

static void listString(std::string &_res, double a, double b)
{
    std::ostringstream oss;
    oss << std::setprecision(5);
    oss << "[";
    oss << a << ", ";
    oss << b << "]";
    _res = oss.str();
}
static void listString(std::string &_res, double a, double b, double c)
{
    std::ostringstream oss;
    oss << std::setprecision(5);
    oss << "[";
    oss << a << ", ";
    oss << b << ", ";
    oss << c << "]";
    _res = oss.str();
}
static void listString(std::string &_res, double a, double b, double c, double d)
{
    std::ostringstream oss;
    oss << std::setprecision(4);
    oss << "[";
    oss << a << ", ";
    oss << b << ", ";
    oss << c << ", ";
    oss << d << "]";
    _res = oss.str();
}
class AssemblerPartsView::Impl
{
public:
    Impl(AssemblerPartsView *_self);
    ~Impl();

    AssemblerPartsView *self;
    AssemblerManager *manager;
    QGridLayout *grid_layout;
    void createPanel(ra::RoboasmCoordsPtr _coords);
    void panelRobot(ra::RoboasmCoordsPtr _coords)
    {
        int row = 0;
        addEditorToPanel("name", _coords->name(), row++);
        addDescriptionToPanel("class", ra::RoboasmUtil::typeName(_coords), row++);
        addCoordsToPanel("initial", _coords->worldcoords(), row++);
    }
    void panelParts(ra::RoboasmCoordsPtr _coords)
    {
        int row = 0;
        addEditorToPanel("name", _coords->name(), row++);
        addDescriptionToPanel("class", ra::RoboasmUtil::typeName(_coords), row++);
        if(!!_coords->parent()) addDescriptionToPanel("parent", _coords->parent()->name(), row++);
        // color??
        ra::Parts *info = _coords->toParts()->info;
        if(!!info) {
            //type
            addDescriptionToPanel("type", info->type, row++);
            //mass
            //com
            //inertia tensor
        }
    }
    void panelConnectingPoint(ra::RoboasmCoordsPtr _coords)
    {
        int row = 0;
        addEditorToPanel("name", _coords->name(), row++);
        addDescriptionToPanel("class", ra::RoboasmUtil::typeName(_coords), row++);
        if(!!_coords->parent()) addDescriptionToPanel("parent", _coords->parent()->name(), row++);
        ra::ConnectingPoint *info = _coords->toConnectingPoint()->info;
        if(!!info) {
            //
            if(info->getType() == ra::ConnectingPoint::Parts) {
                addDescriptionToPanel("type", "connecting-point", row++);
            } else { // actuator
                std::string str_ = "actuator/";
                if(info->getType() == ra::ConnectingPoint::Rotational) {
                    str_ += "rotational";
                } else if (info->getType() == ra::ConnectingPoint::Linear) {
                    str_ += "linear";
                } else if (info->getType() == ra::ConnectingPoint::Fixed) {
                    str_ += "fixed";
                } else if (info->getType() == ra::ConnectingPoint::Free) {
                    str_ += "free";
                } else {
                    str_ += "others";
                }
                addDescriptionToPanel("type", str_, row++);
            }
            addCoordsToPanel("coords", info->coords, row++);
            // connecting?(parent? / child?)
            // connecting conf
            if(info->getType() != ra::ConnectingPoint::Parts) {
                ra::Actuator *ainfo = dynamic_cast<ra::Actuator*>(info);
                if(!!ainfo) {
                    addVectorToPanel("axis", ainfo->axis, row++);
                    // limit
                }
            }
        }
    }
    void addDescriptionToPanel(const std::string &_label, const std::string &_desc, int row)
    {
        auto lb1_ = new QLabel(_label.c_str());
        grid_layout->addWidget(lb1_, row, 0, Qt::AlignRight);
        auto lb2_ = new QLabel(_desc.c_str());
        grid_layout->addWidget(lb2_, row, 1, 1, -1, Qt::AlignLeft);
    }
    void addCoordsToPanel(const std::string &_label, coordinates &_cds, int row)
    {
        int sz_ = (self->frameSize().width() * 2)/5 - 1;
        auto lb1_ = new QLabel(_label.c_str());
        grid_layout->addWidget(lb1_, row, 0, Qt::AlignRight);
        std::string pstr_;
        listString(pstr_, _cds.pos(0), _cds.pos(1), _cds.pos(2));
        auto le1_ = new LineEdit(pstr_.c_str());
        le1_->setMinimumWidth(sz_);
        le1_->setSizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Preferred);
        grid_layout->addWidget(le1_, row, 1, Qt::AlignLeft);
        std::string rstr_;
        AngleAxis ax_(_cds.rot);
        listString(rstr_, ax_.axis()(0), ax_.axis()(1), ax_.axis()(2), ax_.angle());
        auto le2_ = new LineEdit(rstr_.c_str());
        le2_->setMinimumWidth(sz_);
        le2_->setSizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Preferred);
        grid_layout->addWidget(le2_, row, 2, 1, -1, Qt::AlignLeft);
    }
    void addVectorToPanel(const std::string &_label, const Vector3 &_vec, int row)
    {
        auto lb_ = new QLabel(_label.c_str());
        grid_layout->addWidget(lb_, row, 0, Qt::AlignRight);
        std::string str_;
        listString(str_, _vec(0), _vec(1), _vec(2));
        auto le_ = new LineEdit(str_.c_str());
        le_->setMinimumWidth((self->frameSize().width() * 4)/5 - 1);
        le_->setSizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Preferred);
        grid_layout->addWidget(le_, row, 1, 1, -1, Qt::AlignLeft); //Qt::AlignJustify);
    }
    void addEditorToPanel(const std::string &_label, const std::string &_init, int row)
    {
        auto lb_ = new QLabel(_label.c_str());
        grid_layout->addWidget(lb_, row, 0, Qt::AlignRight);
        auto le_ = new LineEdit(_init.c_str());
        le_->setMinimumWidth((self->frameSize().width() * 4)/5 - 1);
        le_->setSizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Preferred);
        grid_layout->addWidget(le_, row, 1, 1, -1, Qt::AlignLeft); //Qt::AlignJustify);
    }
};

void AssemblerPartsView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<AssemblerPartsView>("AssemblerPartsView", "AssemblerParts");
}
AssemblerPartsView* AssemblerPartsView::instance()
{
    static AssemblerPartsView* instance_ = ViewManager::getOrCreateView<AssemblerPartsView>();
    return instance_;
}
AssemblerPartsView::AssemblerPartsView()
{
    impl = new Impl(this);
}
AssemblerPartsView::~AssemblerPartsView()
{
    delete impl;
}
AssemblerPartsView::Impl::Impl(AssemblerPartsView *_self)
    : self(_self), manager(nullptr), grid_layout(nullptr)
{
    self->setDefaultLayoutArea(BottomLeftArea);
    //
    manager = AssemblerManager::instance();
    if(!!manager) {
        manager->sigCoordsSelected().connect( [this](ra::RoboasmCoordsPtr _pt) {
                createPanel(_pt);
            } );
    }
}
AssemblerPartsView::Impl::~Impl()
{
    delete grid_layout;
}
void AssemblerPartsView::Impl::createPanel(ra::RoboasmCoordsPtr _coords)
{
    DEBUG_STREAM(" in:" << _coords->name());
    QLayout *layout_ = self->layout();
    if(!!layout_) {
        QLayoutItem *item_;
        while(!!(item_ = layout_->takeAt(0))) {
            layout_->removeItem(item_);
            DEBUG_STREAM(" wd size: " << item_->widget()->width());
            item_->widget()->hide();
            item_->widget()->deleteLater();
            delete item_;
        }
        delete layout_;
    }
    DEBUG_STREAM(" deleted");

    grid_layout = new QGridLayout();

    grid_layout->setAlignment(Qt::AlignTop);
    //grid_layout->setContentsMargins(0,0,0,0);
    grid_layout->setVerticalSpacing(6);
    grid_layout->setColumnStretch(0, 1);
    grid_layout->setColumnStretch(1, 2);
    grid_layout->setColumnStretch(2, 2);
    //grid_layout->setColumnStretch(2, 2);
    if(_coords->isRobot()) {
        panelRobot(_coords);
    } else if (_coords->isParts()) {
        panelParts(_coords);
    } else if (_coords->isConnectingPoint()) {
        panelConnectingPoint(_coords);
    }
    self->setLayout(grid_layout);
}

#if 0
YAMLReader yrdr;
if (yrdr.parse("")) {
    yrdr.numDocuments();
    ValueNode *nd = yrdr.document;
}
read_vector3(nd);
read_vector3_d(nd);
#endif
