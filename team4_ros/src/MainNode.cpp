#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "Robot.cpp"
#include "Person.cpp"
#include "Dog.cpp"
#include "Util.cpp"
#include "PositionListener.cpp"

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

    if (argc < 2)
    {
        printf("Must specify directory that contains generated locations\n");
        printf("Example:\n");
        printf("%s ~/catkin_ws/src/se306-1/locations/\n", argv[0]);
        return -1;
    }

    std::vector<Robot*> entityList;

    ros::init(argc, argv, "MainNode");

    std::string GoalsLocation = argv[1];

    // Read goals for pickers
    std::string name = GoalsLocation + "pickerLocations";
    std::vector<geometry_msgs::Point> picker_points = Util::readFile(name.c_str());

    int id = 1;
    for (int i = 0; i < 13; i+=2)
    {

        // Create robot object
        Robot *myRobot = new Robot(2, 120,id,"picker");
        entityList.push_back(myRobot);
        // Add some goals to robot
        ROS_INFO("X: %f", picker_points[i].x);
        ROS_INFO("Y: %f", picker_points[i].y);
        myRobot->addGoal(picker_points[i]);
        myRobot->addGoal(picker_points[i+1]);

        id  = id + 1;
    }

    // Set loop rate to 10 Hz
	ros::Rate loop_rate(10);

    // Read location of trees for dog
    std::string filename = GoalsLocation + "dogLocation";
    std::vector<geometry_msgs::Point> trees = Util::readFile(filename.c_str());

    // Instantiate a dog
    Dog myDog = Dog(1, 220,1,"animal");
    entityList.push_back(&myDog);

    // Add some goals to dog
    for(int i = 0; i < trees.size(); i++)
    {
        myDog.addGoal(trees[i]);
    }


    // Instantiate a person
    Person myPerson = Person(2, 110,1,"human");
    entityList.push_back(&myPerson);


    PositionListener* posLis = new PositionListener(entityList);
    
    for (int i = 0; i < entityList.size(); i++)
    {
       entityList[i]->addPositionListener(posLis);
    }
    
    


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
