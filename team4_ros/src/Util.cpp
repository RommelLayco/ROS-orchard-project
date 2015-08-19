namespace Util
{

    int id = 0;

    int getNextId()
    {
        int currentId = id;
        id++;
        return currentId;
    }

}
