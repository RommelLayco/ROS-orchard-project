
class Bin
{
    public:
        Bin(double x, double y, double capacity);
        void addItem();
        bool isFull();
    protected:
	    double x;
	    double y;
        double maxCapacity;
        double currentCapacity;
        
};

