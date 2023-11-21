#include "sin/sin.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sin");
    SinPathCreator creator;
    creator.process();

    return 0;
}