

#include <gtest/gtest.h>

//#include "../src/AlphaRobot.cpp"

//when they fail, ASSERT_* yields a fatal failure and returns from the current function, 
//while EXPECT_* yields a nonfatal failure, allowing the function to continue running
//Normally, EXPECT_* is the better option since the rest of the test can continue to run and can give useful output.
//However, ASSERT_* is better if the test shouldn't continue.
//=================Resident test cases==========================================
TEST(initialPositionTest, alphaRobotTestCase1){
	//ASSERT_EQ(1.600,currentLocation.orientation.x);
	//ASSERT_EQ(-21.000, currentLocation.orientation.y);

	EXPECT_TRUE(true);
}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
