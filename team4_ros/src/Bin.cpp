#include "Bin.h"

Bin::Bin(double xPos, double yPos, double cap = 100)
{
    x = xPos;
    y = yPos;
    maxCapacity = cap;
}

void Bin::addItem()
{
    currentCapacity++;
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
