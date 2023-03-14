#ifndef IRSL_DEBUG
#define IRSL_DEBUG
#endif
#include "../src/RobotAssemblerSettings.h"

using namespace cnoid;
namespace ra = cnoid::robot_assembler;
using namespace ra;

int main(int argc, char **argv) {

    //
    ra::SettingsPtr ra_settings;
    if (argc > 1) {
        bool ret = ra_settings->parseYaml(argv[1]);
        if (!ret) {
            return -1;
        }
    } else {
        return 1;//
    }
    //
    return 0;
}
