#include <gtest/gtest.h>
#define private public
#define protected public
#include "../src/Robot.h"
#include "../src/Robot.cpp"
#include <iostream>


using namespace std;

//when they fail, ASSERT_* yields a fatal failure and returns from the current function, 
//while EXPECT_* yields a nonfatal failure, allowing the function to continue running
//Normally, EXPECT_* is the better option since the rest of the test can continue to run and can give useful output.
//However, ASSERT_* is better if the test shouldn't continue.
//=================Resident test cases==========================================


/**
This test gets the state at the initialisation of the robot and ensures
that the state is set to "Orienting"
**/
TEST(testBasicRobot, testInitialState)
{
    // Create a robot
    Robot testRobot = Robot(1, 2, 3);
    ASSERT_EQ(Orienting, testRobot.getState());
}

/**
This test checks that the variables assigned when the robot is initialised
are the same as what is expected.
**/
TEST(testBasicRobot, testInitialPosition)
{
    // Create a robot
    Robot *testRobot = new Robot(1, 2, 3);

    ASSERT_EQ(1, testRobot->current_x);
    ASSERT_EQ(2, testRobot->current_y);
    ASSERT_EQ(3, testRobot->current_theta);
}

/**
This test checks the initial velocity of the robot at its initialisation
**/
TEST(testBasicRobot, testInitalVelocity)
{
    Robot *testRobot = new Robot(1, 2, 3);

    ASSERT_EQ(0, testRobot->linear_velocity_x);
    ASSERT_EQ(0, testRobot->linear_velocity_y);
    ASSERT_EQ(0, testRobot->angular_velocity);
}

/**
This test makes a odometry type message and sets the position of the robot.
Then it tests that the class correctly receives messages on a topic from stage/other nodes.
**/
TEST(testBasicRobot, testPositionCallback){
    Robot *testRobot = new Robot(1, 2, 3);

    // Create Odometry message
    nav_msgs::Odometry positionMessage;
    double w = 0.9;
    double z = 1.5;
    positionMessage.pose.pose.position.x = 10;
    positionMessage.pose.pose.position.y = 20;
    positionMessage.pose.pose.orientation.x = 0;
    positionMessage.pose.pose.orientation.y = 0;
    positionMessage.pose.pose.orientation.z = z;
    positionMessage.pose.pose.orientation.w = w;

    double yaw = tf::getYaw(tf::Quaternion(0, 0, z, w));

    testRobot->positionCallback(positionMessage);

    ASSERT_EQ(10, testRobot->current_x);
    ASSERT_EQ(20, testRobot->current_y);
    ASSERT_EQ(yaw, testRobot->current_theta);


}


/**
This test checks that the angular velocity and linear velocity
are as expected when the left side collision is detected.
**/
TEST(testBasicRobot, testLeftCollison)
{
	Robot *testRobot = new Robot(1, 2, 3);
	testRobot->leftCollisionDetected();

	ASSERT_EQ(1, testRobot->linear_velocity_x);
	ASSERT_EQ(-0.5, testRobot->angular_velocity);
}

/**
This test checks that the angular velocity and linear velocity
are as expected when the right side collision is detected.
**/
TEST(testBasicRobot, testRightCollision)
{
	Robot *testRobot = new Robot(1, 2, 3);
	testRobot->rightCollisionDetected();

	ASSERT_EQ(1, testRobot->linear_velocity_x);
	ASSERT_EQ(0.5, testRobot->angular_velocity);
}

/**
This test checks that the angular velocity and linear velocity
are as expected when the center side collision is detected.
**/
TEST(testBasicRobot, testCenterCollision)
{
	Robot *testRobot = new Robot(1, 2, 3);
	testRobot->centerCollisionDetected();

	ASSERT_EQ(0, testRobot->linear_velocity_x);
	ASSERT_EQ(-1.0, testRobot->angular_velocity);
}

/**
This test checks that when the robot reaches
its final goal, the goalIndex is set to 0. So it knows
it need to stop
**/
TEST(testBasicRobot, testReachedLastGoal)
{
	Robot *testRobot = new Robot(1, 2, 3);
	testRobot->goalIndex = 0;
	testRobot->reachedLastGoal();

	ASSERT_EQ(1, testRobot->goalIndex);
}

/**
Test that if there is no goal, then the velocity should not
change
**/
TEST(testBasicRobot, testNoGoals)
{
    Robot *testRobot = new Robot(0, 0, 0);
    double x_vel = testRobot->linear_velocity_x;
    double ang_vel = testRobot->angular_velocity;

    testRobot->updateVelocity();
    // Since the robot has no goals defined, it's velocity should not have changed
    ASSERT_EQ(x_vel, testRobot->linear_velocity_x);
    ASSERT_EQ(ang_vel, testRobot->angular_velocity);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
