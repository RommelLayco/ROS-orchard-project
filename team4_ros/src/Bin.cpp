#include "Bin.h"

Bin::Bin(double xPos, double yPos, double cap = 100)
{
    x = xPos;
    y = yPos;
    maxCapacity = cap;
    currentCapacity = 0;
}

void Bin::addItem()
{
    currentCapacity++;
    ROS_INFO("CAPACITY %f/%f", currentCapacity, maxCapacity);
}

bool Bin::isFull()
{
    if (currentCapacity == maxCapacity)
    {
        return true;
    }
    else
    {
        return false;
    }
}

