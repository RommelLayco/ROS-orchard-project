#ifndef UTIL_H
#define UTIL_H

namespace Util
{

    int id = 0;
    int getNextId()
    {
        int currentId = id;
        id++;
        return currentId;
    }


    struct Point
    {
        double x;
        double y;
    };

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

}

#endif
