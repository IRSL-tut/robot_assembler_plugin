#include "AssemblerPartsView.h"
#include "RobotAssembler.h"
#include "AssemblerManager.h"
#include "RobotAssemblerInfo.h"
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

#define PRINT_PRECISION 5

static void floatString(std::string &_res, double a)
{
    std::ostringstream oss;
    oss << std::setprecision(PRINT_PRECISION);
    oss << a;
    _res = oss.str();
}
static void listString(std::string &_res, double a)
{
    std::ostringstream oss;
    oss << std::setprecision(PRINT_PRECISION);
    oss << "[ " << a << " ]";
    _res = oss.str();
}
static void listString(std::string &_res, double a, double b)
{
    std::ostringstream oss;
    oss << std::setprecision(PRINT_PRECISION);
    oss << "[";
    oss << a << ", ";
    oss << b << "]";
    _res = oss.str();
}
static void listString(std::string &_res, double a, double b, double c)
{
    std::ostringstream oss;
    oss << std::setprecision(PRINT_PRECISION);
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
    MappingPtr current_info;
    ra::RoboasmCoordsPtr current_coords;
    void createPanel(ra::RoboasmCoordsPtr _coords, MappingPtr _info);
    void panelRobot(ra::RoboasmCoordsPtr _coords, MappingPtr _info);
    void panelParts(ra::RoboasmCoordsPtr _coords, MappingPtr _info);
    void panelConnectingPoint(ra::RoboasmCoordsPtr _coords, MappingPtr _info);

    void addDescriptionToPanel(const std::string &_label, const std::string &_desc, int row)
    {
        auto lb1_ = new QLabel(_label.c_str());
        grid_layout->addWidget(lb1_, row, 0, Qt::AlignRight);
        auto lb2_ = new QLabel(_desc.c_str());
        grid_layout->addWidget(lb2_, row, 1, 1, -1, Qt::AlignLeft);
    }
    void addDescriptionToPanel(const std::string &_label, Vector3 &_vec, int row)
    {
        std::string str_;
        listString(str_, _vec(0), _vec(1), _vec(2));
        addDescriptionToPanel(_label, str_, row);
    }
    void addDescriptionToPanel(const std::string &_label, Vector3f &_vec, int row)
    {
        std::string str_;
        listString(str_, _vec(0), _vec(1), _vec(2));
        addDescriptionToPanel(_label, str_, row);
    }
    void addDescriptionToPanel(const std::string &_label, double a, double b, int row)
    {
        std::string str_;
        listString(str_, a, b);
        addDescriptionToPanel(_label, str_, row);
    }
    void addDescriptionToPanel(const std::string &_label, double a, int row)
    {
        std::string str_;
        floatString(str_, a);
        addDescriptionToPanel(_label, str_, row);
    }
    void addDescriptionToPanel(const std::string &_label, coordinates &_cds, int row)
    {
        std::string pstr_;
        listString(pstr_, _cds.pos(0), _cds.pos(1), _cds.pos(2));
        std::string rstr_;
        AngleAxis ax_(_cds.rot);
        listString(rstr_, ax_.axis()(0), ax_.axis()(1), ax_.axis()(2), ax_.angle());
        std::string str_ = pstr_ + " / " + rstr_;
        addDescriptionToPanel(_label, str_, row);
    }
    void addCoordsToPanel(const std::string &_label, coordinates &_cds, int row,
                          std::function<void(const std::string &_txt)> funcTrans = nullptr,
                          std::function<void(const std::string &_txt)> funcRot = nullptr)
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
        if(!!funcTrans) {
            le1_->sigEditingFinished().connect(
                [le1_, funcTrans]() { funcTrans(le1_->string()); } );
        }
        std::string rstr_;
        AngleAxis ax_(_cds.rot);
        listString(rstr_, ax_.axis()(0), ax_.axis()(1), ax_.axis()(2), ax_.angle());
        auto le2_ = new LineEdit(rstr_.c_str());
        le2_->setMinimumWidth(sz_);
        le2_->setSizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Preferred);
        grid_layout->addWidget(le2_, row, 2, 1, -1, Qt::AlignLeft);
        if(!!funcRot) {
            le2_->sigEditingFinished().connect(
                [le2_, funcRot]() { funcRot(le2_->string()); } );
        }
    }
    void addEditorToPanel(const std::string &_label, const std::string &_init, int row,
                          std::function<void(const std::string &_txt)> func = nullptr)
    {
        auto lb_ = new QLabel(_label.c_str());
        grid_layout->addWidget(lb_, row, 0, Qt::AlignRight);
        auto le_ = new LineEdit(_init.c_str());
        le_->setMinimumWidth((self->frameSize().width() * 4)/5 - 1);
        le_->setSizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Preferred);
        grid_layout->addWidget(le_, row, 1, 1, -1, Qt::AlignLeft); //Qt::AlignJustify);
        if(!!func) {
            le_->sigEditingFinished().connect(
                [le_, func]() { func(le_->string()); } );
        }
    }
    void addVectorToPanel(const std::string &_label, const Vector3 &_vec, double a, int row,
                          std::function<void(const std::string &_txt)> func = nullptr)
    {
        std::string str_;
        listString(str_, _vec(0), _vec(1), _vec(2), a);
        addEditorToPanel(_label, str_, row, func);
    }
    void addVectorToPanel(const std::string &_label, const Vector3 &_vec, int row,
                          std::function<void(const std::string &_txt)> func = nullptr)
    {
        std::string str_;
        listString(str_, _vec(0), _vec(1), _vec(2));
        addEditorToPanel(_label, str_, row, func);
    }
    void addVectorToPanel(const std::string &_label, const Vector3f &_vec, int row,
                          std::function<void(const std::string &_txt)> func = nullptr)
    {
        std::string str_;
        listString(str_, _vec(0), _vec(1), _vec(2));
        addEditorToPanel(_label, str_, row, func);
    }
    void addVectorToPanel(const std::string &_label, double a, double b, int row,
                          std::function<void(const std::string &_txt)> func = nullptr)
    {
        std::string str_;
        listString(str_, a, b);
        addEditorToPanel(_label, str_, row, func);
    }
    void addVectorToPanel(const std::string &_label, double a, int row,
                          std::function<void(const std::string &_txt)> func = nullptr)
    {
        std::string str_;
        floatString(str_, a);
        addEditorToPanel(_label, str_, row, func);
    }
    void infoRobot(const std::string &_key, const std::string &_str)
    {
        if(!!current_info) {
            Mapping *mp_ = ra::getRobotInfo(current_info);
            if(!mp_) {
                MappingPtr pt_ = new Mapping();
                addToMapping(pt_, _key, _str);
                ra::addMapping(current_info, "robot-info", pt_);
            } else {
                addToMapping(mp_, _key, _str);
            }
        }
        // update??
    }
    void transInfo(const std::string &_str)
    {
        if(!current_info) return;
        Vector3 trs;
        bool ret = parseFromString(trs, _str);
        if (ret) {
            Mapping *mp_ = ra::getRobotInfo(current_info);
            if(!mp_) {
                MappingPtr cds_ = new Mapping();
                addToMapping(cds_, "translation", trs);
                MappingPtr pt_ = new Mapping();
                addToMapping(pt_, "initial-coords", cds_);
                ra::addMapping(current_info, "robot-info", pt_);
            } else {
                Mapping *cds_ = ra::getMapping(mp_, "initial-coords");
                if(!cds_) {
                    cds_ = new Mapping();
                    addToMapping(mp_, "initial-coords", cds_);
                }
                addToMapping(cds_, "translation", trs);
            }
        }
    }
    void aaxisInfo(const std::string &_str)
    {
        if(!current_info) return;
        Vector3 ax; double ang;
        bool ret = parseFromString(ax, ang, _str);
        if (ret) {
            Mapping *mp_ = ra::getRobotInfo(current_info);
            if(!mp_) {
                MappingPtr cds_ = new Mapping();
                addToMapping(cds_, "rotation", ax, ang);
                MappingPtr pt_ = new Mapping();
                addToMapping(pt_, "initial-coords", cds_);
                ra::addMapping(current_info, "robot-info", pt_);
            } else {
                Mapping *cds_ = ra::getMapping(mp_, "initial-coords");
                if(!cds_) {
                    cds_ = new Mapping();
                    addToMapping(mp_, "initial-coords", cds_);
                }
                addToMapping(cds_, "rotation", ang, ax);
            }
        }
    }
    void transCoords(const std::string &_str)
    {
        if(!current_coords) return;
        Vector3 trs;
        bool ret = parseFromString(trs, _str);
        if (ret) {
            current_coords->set(trs);
            current_coords->update();
            if(!!manager) {
                manager->updateRobotsCoords();
            }
        }
    }
    void aaxisCoords(const std::string &_str)
    {
        if(!current_coords) return;
        Vector3 ax; double ang;
        bool ret = parseFromString(ax, ang, _str);
        if (ret) {
            AngleAxis aa_(ang, ax);
            current_coords->set(aa_);
            current_coords->update();
            if(!!manager) {
                manager->updateRobotsCoords();
            }
        }
    }
    void colorInfo(const std::string &_parts_name, ra::RoboasmCoordsPtr _coords, const std::string &_txt)
    {
        infoLinkVec3(_parts_name, "color", _txt);
        // TODO : parse twice
        Vector3f vec_;
        if(parseFromString(vec_, _txt)){
            _coords->toParts()->color = vec_;
            if(!!manager) {
                ra::RASceneBase *res = manager->coordsToScene(_coords);
                if (!!res) {
                    ra::RASceneParts *pt = dynamic_cast<ra::RASceneParts *>(res);
                    if (!!pt) {
                        pt->updateColor(vec_);
                        pt->notifyUpdate(SgUpdate::MODIFIED);
                    }
                }
            }
        }
    }
    void infoLink(const std::string &_parts, const std::string &_key, const std::string &_str)
    {
        info_("parts-info", _parts, _key, _str);
    }
    void infoActuator(const std::string &_act, const std::string &_key, const std::string &_str)
    {
        info_("actuator-info", _act, _key, _str);
    }
    void infoDevice(const std::string &_dev, const std::string &_key, const std::string &_str)
    {
        info_("device-info", _dev, _key, _str);
    }
    void infoLinkVec3(const std::string &_parts, const std::string &_key, const std::string &_txt)
    {
        info_vec3_("parts-info", _parts, _key, _txt);
    }
    void infoActuatorVec2(const std::string &_act, const std::string &_key, const std::string &_txt)
    {
        info_vec2_("actuator-info", _act, _key, _txt);
    }
    void info_(const std::string &_type, const std::string &_name,
               const std::string &_key, const std::string &_str)
    {
        if(!!current_info) {
            Mapping *mp_ = ra::getMapping(current_info, _type, _name);
            if(!mp_) {
                DEBUG_STREAM("new: " << _type << ", " << _name << ", " << _key << ", " << _str);
                MappingPtr tgt_ = new Mapping();
                addToMapping(tgt_, _key, _str);
                ra::addMapping(current_info, _type, _name, tgt_);
            } else {
                addToMapping(mp_, _key, _str);
            }
        }
    }
    bool info_float_(const std::string &_type, const std::string &_name,
                     const std::string &_key, const std::string &_txt, double &_val)
    {
        DEBUG_SIMPLE("info_float_");
        if(parseFromString(_val, _txt)) {
            DEBUG_SIMPLE("float parsed : " << _val);
            if(!!current_info) {
                DEBUG_SIMPLE("cur_info");
                Mapping *mp_ = ra::getMapping(current_info, _type, _name);
                if(!mp_) {
                    MappingPtr tgt_ = new Mapping();
                    addToMapping(tgt_, _key, _val);
                    ra::addMapping(current_info, _type, _name, tgt_);
                } else {
                    addToMapping(mp_, _key, _val);
                }
                return true;
            }
        }
        return false;
    }
    void info_vec2_(const std::string &_type, const std::string &_name,
                   const std::string &_key, const std::string &_txt)
    {
        double a, b;
        DEBUG_SIMPLE("vec2");
        if(parseFromString(a, b, _txt)) {
            DEBUG_SIMPLE("vec2 parsed");
            if(!!current_info) {
                DEBUG_SIMPLE("cur_info");
                Mapping *mp_ = ra::getMapping(current_info, _type, _name);
                if(!mp_) {
                    MappingPtr tgt_ = new Mapping();
                    addToMapping(tgt_, _key, a, b);
                    ra::addMapping(current_info, _type, _name, tgt_);
                } else {
                    addToMapping(mp_, _key, a, b);
                }
            }
        }
    }
    void info_vec3_(const std::string &_type, const std::string &_name,
                   const std::string &_key, const std::string &_txt)
    {
        Vector3 vec_;
        if(parseFromString(vec_, _txt)) {
            if(!!current_info) {
                Mapping *mp_ = ra::getMapping(current_info, _type, _name);
                if(!mp_) {
                    MappingPtr tgt_ = new Mapping();
                    addToMapping(tgt_, _key, vec_);
                    ra::addMapping(current_info, _type, _name, tgt_);
                } else {
                    addToMapping(mp_, _key, vec_);
                }
            }
        }
    }
    void info_vec4_(const std::string &_type, const std::string &_name,
                   const std::string &_key, const std::string &_txt)
    {
        Vector3 vec_; double a;
        if(parseFromString(vec_, a, _txt)) {
            if(!!current_info) {
                Mapping *mp_ = ra::getMapping(current_info, _type, _name);
                if(!mp_) {
                    MappingPtr tgt_ = new Mapping();
                    addToMapping(tgt_, _key, a, vec_);
                    ra::addMapping(current_info, _type, _name, tgt_);
                } else {
                    addToMapping(mp_, _key, a, vec_);
                }
            }
        }
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
        manager->sigCoordsSelected().connect(
            [this](ra::RoboasmCoordsPtr _pt, MappingPtr _info) { createPanel(_pt, _info); } );
    }
}
AssemblerPartsView::Impl::~Impl()
{
    delete grid_layout;
}
void AssemblerPartsView::Impl::createPanel(ra::RoboasmCoordsPtr _coords, MappingPtr _info)
{
    if(!!_coords) {
        DEBUG_STREAM(" in:" << _coords->name() << ", info: " << !!_info);
    } else {
        DEBUG_STREAM(" delete panel");
    }
    current_info = _info;
    current_coords = _coords;
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
    if(!_coords) {
        // do nothing
    } else if(_coords->isRobot()) {
        panelRobot(_coords, _info);
    } else if (_coords->isParts()) {
        panelParts(_coords, _info);
    } else if (_coords->isConnectingPoint()) {
        panelConnectingPoint(_coords, _info);
    }
    self->setLayout(grid_layout);
}
void AssemblerPartsView::Impl::panelRobot(ra::RoboasmCoordsPtr _coords, MappingPtr _info)
{
    int row = 0;
    std::string nm_ = _coords->name();
    if (!!current_info) {
        Mapping *mp_ = ra::getRobotInfo(current_info);
        if(!!mp_) {
            readFromMapping(mp_, "name", nm_);
        }
    }
    addEditorToPanel("name", nm_, row++,
                     [this] (const std::string &_s) { infoRobot("name", _s); } );
    addDescriptionToPanel("class", ra::RoboasmUtil::typeName(_coords), row++);
    addCoordsToPanel("cuurent", _coords->worldcoords(), row++,
                     [this] (const std::string &_s) { transCoords(_s); },
                     [this] (const std::string &_s) { aaxisCoords(_s); } );
    coordinates cds_;
    if (!!current_info) {
        ra::getRobotCoords(current_info, cds_);
    }
    addCoordsToPanel("initial", cds_, row++,
                     [this] (const std::string &_s) { transInfo(_s); },
                     [this] (const std::string &_s) { aaxisInfo(_s); } );
}
void AssemblerPartsView::Impl::panelParts(ra::RoboasmCoordsPtr _coords, MappingPtr _info)
{
    int row = 0;
    std::string nm_ = _coords->name();
    addDescriptionToPanel("name", nm_, row++);
    addDescriptionToPanel("class", ra::RoboasmUtil::typeName(_coords), row++);
    if(!!_coords->parent()) addDescriptionToPanel("parent", _coords->parent()->name(), row++);
    // link-name
    std::string lnm_ = nm_;
    if(!!current_info) {
        Mapping *mp_ = ra::getPartsInfo(current_info, _coords->name());
        if(!!mp_) {
            readFromMapping(mp_, "name", lnm_);
        }
    }
    addEditorToPanel("link-name", lnm_, row++,
                     [this,nm_] (const std::string &_s) { infoLink(nm_, "name", _s); } );
    // color
    Vector3f v_ = _coords->toParts()->color;
    if(!!current_info) {
        Mapping *mp_ = ra::getPartsInfo(current_info, _coords->name());
        if(!!mp_) {
            readFromMapping(mp_, "color", v_);
        }
    }
    addVectorToPanel("color", v_, row++,
                     [this,nm_,_coords] (const std::string &_s) { colorInfo(nm_, _coords, _s); } );
    //
    ra::Parts *info = _coords->toParts()->info;
    if(!!info) {
        addDescriptionToPanel("type", info->type, row++);
        addDescriptionToPanel("mass", info->mass, row++);
        addDescriptionToPanel("COM", info->COM, row++);
        //addDescriptionToPanel("inertia", info->inertia_tensor, row++);

        for(int i = 0; i < info->extra_data.size(); i++) {
            std::string tp = "unknown";
            switch(info->extra_data[i].type) {
            case ra::ExtraInfo::IMU:
                tp = "IMU";
                break;
            case ra::ExtraInfo::Acceleration:
                tp = "Acceleration";
                break;
            case ra::ExtraInfo::RateGyro:
                tp = "RateGyro";
                break;
            case ra::ExtraInfo::Touch:
                tp = "Touch";
                break;
            case ra::ExtraInfo::Force:
                tp = "Force";
                break;
            case ra::ExtraInfo::Color:
                tp = "Color";
                break;
            case ra::ExtraInfo::Distance:
                tp = "Distance";
                break;
            case ra::ExtraInfo::Camera:
                tp = "Camera";
                break;
            case ra::ExtraInfo::Depth:
                tp = "Depth";
                break;
            case ra::ExtraInfo::RGBD:
                tp = "RGBD";
                break;
            case ra::ExtraInfo::Ray:
                tp = "Ray";
                break;
            case ra::ExtraInfo::Position:
                tp = "Position";
                break;
            case ra::ExtraInfo::Light:
                tp = "Light";
                break;
            case ra::ExtraInfo::PointLight:
                tp = "PointLight";
                break;
            case ra::ExtraInfo::SpotLight:
                tp = "SpotLight";
                break;
            case ra::ExtraInfo::DegitalIO:
                tp = "DegitalIO";
                break;
            case ra::ExtraInfo::AnalogIO:
                tp = "AnalogIO";
                break;
            }

            addDescriptionToPanel("---", "", row++);
            addDescriptionToPanel("  id", info->extra_data[i].name, row++);
            addDescriptionToPanel("type", tp, row++);
            std::string dev_ = _coords->name() + "/" + info->extra_data[i].name;
            std::string dnm_;
            if(!!current_info) {
                Mapping *mp_ = ra::getDeviceInfo(current_info, dev_);
                if(!!mp_) {
                    readFromMapping(mp_, "name", dnm_);
                }
            }
            addEditorToPanel("name", dnm_, row++,
                     [this,dev_] (const std::string &_s) { infoDevice(dev_, "name", _s); } );
        }
    }
}
void AssemblerPartsView::Impl::panelConnectingPoint(ra::RoboasmCoordsPtr _coords, MappingPtr _info)
{
    int row = 0;
    std::string nm_ = _coords->name();
    ra::ConnectingPoint *info = _coords->toConnectingPoint()->info;
    addDescriptionToPanel("name", nm_, row++);
    addDescriptionToPanel("point-name", _coords->point_name(), row++);
    addDescriptionToPanel("class", ra::RoboasmUtil::typeName(_coords), row++);
    if(!!_coords->parent()) addDescriptionToPanel("parent", _coords->parent()->name(), row++);
    if(!!info && info->getType() != ra::ConnectingPoint::Parts) {
        // actuator-name
        std::string lnm_ = nm_;
        if(!!current_info) {
            Mapping *mp_ = ra::getActuatorInfo(current_info, nm_);
            if(!!mp_) {
                readFromMapping(mp_, "name", lnm_);
            }
        }
        addEditorToPanel("joint-name", lnm_, row++,
                         [this,nm_] (const std::string &_s) { infoActuator(nm_, "name", _s); } );
    }
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
        addDescriptionToPanel("coords", info->coords, row++);
        // connecting offset
        // connecting?(parent? / child?)
        if(info->getType() != ra::ConnectingPoint::Parts) {
            ra::Actuator *ainfo = dynamic_cast<ra::Actuator*>(info);
            if(!!ainfo) {
                addDescriptionToPanel("axis", ainfo->axis, row++);
                {// curreng_angle
                double a = 0;
                if(!!current_info) {
                    Mapping *mp_ = ra::getActuatorInfo(current_info, nm_);
                    if(!!mp_) {
                        readFromMapping(mp_, "current_angle", a);
                        DEBUG_SIMPLE("cur_ang: " << a);
                    }
                }
                addVectorToPanel("current_angle", a, row++,
                                 [this,nm_, _coords] (const std::string &_s) {
                                     double _val;
                                     if (info_float_("actuator-info", nm_, "current_angle", _s, _val)) {
                                         // update robot
                                         ra::RASceneBase *res = manager->coordsToScene(_coords);
                                         if (!!res) {
                                             ra::RASceneConnectingPoint *pt = dynamic_cast<ra::RASceneConnectingPoint*>(res);
                                             if (!!pt) {
                                                 ra::RASceneRobot *rb = pt->scene_robot();
                                                 rb->robot()->applyJointAngle(nm_, _val);
                                                 rb->updateStructure();
                                                 rb->notifyUpdate(SgUpdate::MODIFIED);
                                             }
                                         }
                                     } } );
                }
                {// curreng_angle
                double a = 0;
                if(!!current_info) {
                    Mapping *mp_ = ra::getActuatorInfo(current_info, nm_);
                    if(!!mp_) {
                        readFromMapping(mp_, "initial_angle", a);
                    }
                }
                addVectorToPanel("initial_angle", a, row++,
                                 [this,nm_] (const std::string &_s) {
                                     double _a; info_float_("actuator-info", nm_, "initial_angle", _s, _a); } );
                }
                {// limit
                double a = ainfo->limit[0];
                double b = ainfo->limit[1];
                if(!!current_info) {
                    Mapping *mp_ = ra::getActuatorInfo(current_info, nm_);
                    if(!!mp_) {
                        readFromMapping(mp_, "limit", a, b);
                        DEBUG_SIMPLE("lim: " << a << ", " << b);
                    }
                }
                addVectorToPanel("limit", a, b, row++,
                                 [this,nm_] (const std::string &_s) { infoActuatorVec2(nm_, "limit", _s); } );
                }
                {// vlimit
                double a = ainfo->vlimit[0];
                double b = ainfo->vlimit[1];
                if(!!current_info) {
                    Mapping *mp_ = ra::getActuatorInfo(current_info, nm_);
                    if(!!mp_) {
                        readFromMapping(mp_, "vlimit", a, b);
                    }
                }
                addVectorToPanel("vlimit", a, b, row++,
                                 [this,nm_] (const std::string &_s) { infoActuatorVec2(nm_, "vlimit", _s); } );
                }
                {// tqlimit
                double a = ainfo->tqlimit[0];
                double b = ainfo->tqlimit[1];
                if(!!current_info) {
                    Mapping *mp_ = ra::getActuatorInfo(current_info, nm_);
                    if(!!mp_) {
                        readFromMapping(mp_, "tqlimit", a, b);
                    }
                }
                addVectorToPanel("tqlimit", a, b, row++,
                                 [this,nm_] (const std::string &_s) { infoActuatorVec2(nm_, "tqlimit", _s); } );
                }
            }
        }
    }
}
