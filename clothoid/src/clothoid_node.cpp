#include "clothoid/clothoid.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "clothoid");
    ClothoidPathCreator creator;
    creator.process();

    return 0;
}