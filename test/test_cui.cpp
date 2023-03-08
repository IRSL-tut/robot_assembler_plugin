#include "../src/RobotAssembler.h"
#include "../src/RobotAssemblerBody.h"

#include <cnoid/StdBodyWriter>
#include <iostream>

using namespace cnoid;
namespace ra = cnoid::robot_assembler;
using namespace ra;

int main(int argc, char **argv) {

    std::shared_ptr<RoboasmUtil> ra_util;
#if 0
    bool ret = false;
    if (argc > 1) {
        std::cerr << "argc > 1" << std::endl;
        ra_util = std::make_shared<RoboasmUtil>(argv[1]);
        ret = ra_util->isReady();
    }
#endif
    ra::SettingsPtr ra_settings = std::make_shared<ra::Settings>();
    if (argc > 1) {
        std::cerr << "argc > 1" << std::endl;
        bool ret = ra_settings->parseYaml(argv[1]);
        if (ret) {
            ra_util = std::make_shared<RoboasmUtil>(ra_settings);
            ra_util->isReady();
        }
    }

    {
        RoboasmFile roboasm_file("assembled_robot.roboasm");
        if(roboasm_file.isValid()) {
            std::cout << "valid file" << std::endl;
            if(!roboasm_file.dumpRoboasm("/tmp/hoge.roboasm")) {
                std::cout << "dump fail" << std::endl;
            }
        }
    }

    RoboasmRobotPtr roboasm_rb = ra_util->makeRobotFromFile("assembled_robot.roboasm");
    if(!!roboasm_rb) {
        //coordsPtrList lstp;
        //roboasm_rb->allDescendants(lstp);
        //std::cout << "roboasm_rb : " << lstp.size() << std::endl;
        //print_lst(lstp);

        if(roboasm_rb->checkValidity()) {
            std::cout << "VALID" << std::endl;
        }
        RoboasmFile rb_file;
        roboasm_rb->createRoboasm(rb_file);
        if(!rb_file.dumpRoboasm("/tmp/fuga.roboasm")) {
            std::cout << "dump fail fuga" << std::endl;
        }

        //RoboasmBodyCreator rbc(_directory);
        RoboasmBodyCreator rbc;
        rbc.setMergeFixedJoint();
        BodyPtr bd = rbc.createBody(roboasm_rb);
        if (!!bd) {
            std::cout << "body created" << std::endl;
            StdBodyWriter writer;
            writer.writeBody(bd, "/tmp/new.body");
        }
    }
}
