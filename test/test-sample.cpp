#include "gtest/gtest.h"
#include "../src/RobotAssembler.h"

namespace ra = cnoid::robot_assembler;
TEST(test0, name0) {
    ra::RoboasmCoordsPtr a = std::make_shared<ra::RoboasmCoords> ("testcoords");

    //EXCEPT
    ASSERT_TRUE(a->hasParent());
}
