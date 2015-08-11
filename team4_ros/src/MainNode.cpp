#include "Robot.cpp"
#include "Person.cpp"

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

    std::vector<Robot*> entityList;

    // Create robot object
    Robot myRobot = Robot(0, 0, 0);
    entityList.push_back(&myRobot);

    ros::init(argc, argv, "MainNode");

    // Setup ros handles
    ros::NodeHandle publisherHandle;
    ros::NodeHandle subscriberHandle;

    // Create robot position publisher
    ros::Publisher mypub_object = publisherHandle.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);

    // Attach publisher to robot object
    myClass temp = myClass(mypub_object);
    myRobot.addSpeedListener(&temp);

    // loop 10 Hz
	ros::Rate loop_rate(10);

    ros::Subscriber groundtruthSub = subscriberHandle.subscribe<nav_msgs::Odometry>("robot_0/base_pose_ground_truth",1000, &Robot::positionCallback, &myRobot);
    ros::Subscriber SensorSub = subscriberHandle.subscribe("robot_0/base_scan", 1000, &Robot::sensorCallback, &myRobot);


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
    Person myPerson = Person(1, 2, 3);
    entityList.push_back(&myPerson);

    // Create person position publisher
    ros::Publisher mypub_object1 = publisherHandle.advertise<geometry_msgs::Twist>("robot_1/cmd_vel",1000);

    // Attach publisher to robot object
    myClass temp1 = myClass(mypub_object1);
    myPerson.addSpeedListener(&temp1);

    ros::Subscriber groundtruthSub1 = subscriberHandle.subscribe<nav_msgs::Odometry>("robot_1/base_pose_ground_truth",1000, &Person::positionCallback, dynamic_cast<Robot*>(&myPerson));
    ros::Subscriber SensorSub1 = subscriberHandle.subscribe("robot_1/base_scan", 1000, &Person::sensorCallback, dynamic_cast<Robot*>(&myPerson));

    myPerson.addGoal(desiredLocation1);


    while (ros::ok()) 
	{ 
        // In future, will loop through all robot objects calling this method on each of them
        for (int i = 0; i < entityList.size(); i++)
        {
            entityList[i]->updateVelocity();
        }

		ros::spinOnce();
		loop_rate.sleep();
	} 

	return 0; 

}
