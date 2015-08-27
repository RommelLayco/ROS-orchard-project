#ifndef UTIL_H
#define UTIL_H
#include <fstream>


namespace Util
{

int id = 0;
int getNextId()
{
    int currentId = id;
    id++;
    return currentId;
}


std::vector<geometry_msgs::Point> readFile(const char filename[])
{
    std::ifstream myReadFile;
    myReadFile.open(filename);
    std::string astring1;
    std::string astring2;
    std::vector<geometry_msgs::Point> points;

    if (myReadFile.is_open())
    {
        while (!myReadFile.eof())
        {
            getline(myReadFile, astring1, ' ');
            getline(myReadFile, astring2, '\n');
            if (astring1 == "") {break;}
            geometry_msgs::Point tempPoint;
            tempPoint.x = std::stod(astring1);
            tempPoint.y = std::stod(astring2);
            tempPoint.z = 0;
            points.push_back(tempPoint);

        }
    }

    myReadFile.close();
    return points;
}


/* Calculate the straight line distance between two points */
double getDistance(double x1, double y1, double x2, double y2)
{
    double xDiff = x1 - x2;
    double yDiff = y1 - y2;
    return sqrtf(  pow(xDiff, 2) +  pow(yDiff, 2) );
}



}

#endif
