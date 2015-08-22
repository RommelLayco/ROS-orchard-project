#include<iostream>
#include<fstream>
#include <vector>
#include <string>
#include "Robot.cpp"
#include "Person.cpp"
#include "Dog.cpp"
#include "Util.cpp"

class myClass: public SpeedListener {
    private:
        ros::Publisher publisher;
    public:
        myClass(ros::Publisher pub);
        void speedUpdate(geometry_msgs::Twist speedMsg);
};

myClass::myClass(ros::Publisher pub)
{
    publisher = pub;
}

void myClass::speedUpdate(geometry_msgs::Twist speedMsg)
{
    publisher.publish(speedMsg);
}


int main(int argc, char **argv)
{
    // NOTE: The order in which entities are instantiaed here must
    // match the order in which they are defined in the world file.

    std::vector<Robot*> entityList;

    ros::init(argc, argv, "MainNode");

    // Create robot object
    Robot myRobot = Robot(0, 0, 0, 2, 60);
    entityList.push_back(&myRobot);

    // Set loop rate to 10 Hz
	ros::Rate loop_rate(10);

    // Add some goals to robot
    geometry_msgs::Point desiredLocation1;
    desiredLocation1.x = 14.0;
    desiredLocation1.y = 18.0;
    desiredLocation1.z = 0;

    geometry_msgs::Point desiredLocation2;
    desiredLocation2.x = 1.6;
    desiredLocation2.y = -2;
    desiredLocation2.z = 0;

    myRobot.addGoal(desiredLocation1);
    myRobot.addGoal(desiredLocation2);
    

    // Instantiate a person
    Person myPerson = Person(1, 2, 3, 2, 110);
    entityList.push_back(&myPerson);

    myPerson.addGoal(desiredLocation1);

    // Read location of trees for dog
    char filename[] = "dogLocation";
    std::vector<geometry_msgs::Point> trees = Util::readFile(filename);

    // Instantiate a dog
    Dog myDog = Dog(0, 0, 0, 1, 180);
    entityList.push_back(&myDog);

    // Add some goals to dog
    for(int i = 0; i < trees.size(); i++)
    {
        myDog.addGoal(trees[i]);
    }

    myDog.addGoal(desiredLocation1);
    myDog.addGoal(desiredLocation2);

    while (ros::ok()) 
	{ 
        // Loop through all robot objects calling updateVelocity method on each of them
        for (int i = 0; i < entityList.size(); i++)
        {
            entityList[i]->updateVelocity();
        }

		ros::spinOnce();
		loop_rate.sleep();
	} 

	return 0; 

}
