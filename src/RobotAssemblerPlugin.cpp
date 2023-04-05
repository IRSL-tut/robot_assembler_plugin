#include <cnoid/OptionManager> //??

#include "RobotAssemblerPlugin.h"
#include "AssemblerItem.h"
#include "AssemblerView.h"
#include "AssemblerTreeView.h"
#include "AssemblerPartsView.h"
#include "AssemblerBar.h"
#include "AssemblerManager.h"

#include <fmt/format.h>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>

#include "RobotAssembler.h"

//#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;

namespace po = boost::program_options;
namespace ra = cnoid::robot_assembler;
namespace filesystem = cnoid::stdx::filesystem;

namespace {
RobotAssemblerPlugin* instance_ = nullptr;
}

namespace cnoid {
class RobotAssemblerPlugin::Impl
{
public:
    Impl(RobotAssemblerPlugin *_self) { self = _self; }

    RobotAssemblerPlugin *self;

    void onSigOptionsParsed(po::variables_map& variables);
};
}
void RobotAssemblerPlugin::Impl::onSigOptionsParsed(po::variables_map& variables)
{
    //// Order of initializaiton
    // plugin created
    // initializeClass
    // creating project ( from .cnoid )
    // onSigOptionParsed
    DEBUG_PRINT();
    if(variables.count("assembler")) {
        std::string fname_ = variables["assembler"].as<std::string>();
        DEBUG_STREAM("robot_assembler config file: " << fname_);
        AssemblerManager *manager = AssemblerManager::instance();
        if (!!manager) {
            manager->loadSettings(fname_);
        }
#if 0
        filesystem::path path_(fromUTF8(fname_));
        std::string ppath_;
        if(path_.is_relative()) {
            auto ap_ = filesystem::absolute(path_);
            ppath_ = ap_.parent_path().generic_string();
            DEBUG_STREAM("rel: ppath: " << ppath_);
        } else {
            ppath_ = path_.parent_path().generic_string();
            DEBUG_STREAM("abs: ppath: " << ppath_);
        }
        ra::SettingsPtr ra_settings = std::make_shared<ra::Settings> ();
        bool ret = ra_settings->parseYaml(fname_);
        if (ret) {
            AssemblerManager *manager = AssemblerManager::instance();
            if(!!manager) {
                manager->setProjectDirectory(ppath_);
                manager->ra_settings = ra_settings;
                manager->ra_util = std::make_shared<ra::RoboasmUtil>(ra_settings);
                AssemblerView *ptr = AssemblerView::instance();
                ptr->createButtons();
            }
            if(variables.count("assembler-robot")) {
                std::string fname_ = variables["assembler-robot"].as<std::string>();
                if(!!manager) {
                    manager->loadRoboasm(fname_);
                }
            }
        }
#endif
        if(variables.count("assembler-robot")) {
            std::string fname_ = variables["assembler-robot"].as<std::string>();
            AssemblerManager *manager = AssemblerManager::instance();
            if(!!manager) {
                manager->loadRoboasm(fname_);
            }
        }

        if(variables.count("original-project")) {
            std::string fname_ = variables["original-project"].as<std::string>();
            DEBUG_STREAM(" original-project : " << fname_);
            AssemblerManager *manager = AssemblerManager::instance();
            if(!!manager) {
                manager->setOriginalProject(fname_);
            }
        }

        if(variables.count("project")) {
            std::vector<std::string> _fls = variables["project"].as<std::vector<std::string>>();
            DEBUG_STREAM(" project : ");
            for(int i = 0; i < _fls.size(); i++) {
                DEBUG_STREAM(" " << i << ": " << _fls[i]);
            }
            if(!!manager) {
                manager->setAssemblerProject(_fls[0]);
            }
        }
        if(variables.count("input-file")) {
            std::vector<std::string> _fls = variables["input-file"].as<std::vector<std::string>>();
            DEBUG_STREAM(" input-file : ");
            for(int i = 0; i < _fls.size(); i++) {
                DEBUG_STREAM(" " << i << ": " << _fls[i]);
            }
            if(!!manager) {
                manager->setAssemblerProject(_fls[0]);
            }
        }
    }
}
RobotAssemblerPlugin* RobotAssemblerPlugin::instance()
{
    return instance_;
}
RobotAssemblerPlugin::RobotAssemblerPlugin()
    : Plugin("RobotAssembler")
{
    DEBUG_PRINT();
    setActivationPriority(0);
    instance_ = this;

    impl = new Impl(this);
}
RobotAssemblerPlugin::~RobotAssemblerPlugin()
{
    delete impl;
}
bool RobotAssemblerPlugin::initialize()
{
    DEBUG_PRINT();
    OptionManager& om = this->optionManager();
    om.addOption("assembler", po::value<std::string>(), "load robot_assembler config file");
    om.addOption("assembler-robot", po::value<std::string>(), "load robot_assembler .roboasm file");
    om.addOption("original-project", po::value<std::string>(), "project file for original choreonoid");
    //om.sigOptionsParsed(1).connect(onSigOptionsParsed);
    om.sigOptionsParsed(1).connect(
        [&](po::variables_map& _v) { impl->onSigOptionsParsed(_v); } );

    // classes
    AssemblerItem::initializeClass(this);

    //View
    AssemblerView::initializeClass(this);
    AssemblerTreeView::initializeClass(this);
    AssemblerPartsView::initializeClass(this);

    //ToolBar
    addToolBar(AssemblerBar::instance());
    DEBUG_STREAM(" FINISH Plugin initialize");
    return true;
}
bool RobotAssemblerPlugin::finalize()
{
    DEBUG_PRINT();
    instance_ = nullptr;
    return true;
}
const char* RobotAssemblerPlugin::description() const
{
    static std::string text =
        fmt::format("RobotAssembler Plugin Version {}\n", CNOID_FULL_VERSION_STRING) +
        "\n" +
        "Copyrigh (c) 2022 IRSL-tut Development Team.\n"
        "\n" +
        MITLicenseText() +
        "\n"  ;

    return text.c_str();
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(RobotAssemblerPlugin);
