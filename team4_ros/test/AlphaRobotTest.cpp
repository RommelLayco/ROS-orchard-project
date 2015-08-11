#include <gtest/gtest.h>
#include "../src/Robot.h"
#include "../src/Robot.cpp"


//when they fail, ASSERT_* yields a fatal failure and returns from the current function, 
//while EXPECT_* yields a nonfatal failure, allowing the function to continue running
//Normally, EXPECT_* is the better option since the rest of the test can continue to run and can give useful output.
//However, ASSERT_* is better if the test shouldn't continue.
//=================Resident test cases==========================================
TEST(initialPositionTest, alphaRobotTestCase1){
    // Create a robot
    Robot testRobot = Robot(1, 2, 3);
    ASSERT_EQ(Orienting, testRobot.getState());

}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
