// Small test driver
#include "Config.h"
#include "Conversions.cpp"
#include <iostream>

int main()
{
    float_type longitude, latitude;
    latitude = 34.082729;
    longitude = -117.931480;
    init(longitude, latitude);
    computeBasis();
    std::cout << global::ourX[X] << "   " << global::ourX[Y] << "   " << global::ourX[Z] << '\n';
    std::cout << global::ourY[X] << "   " << global::ourY[Y] << "   " << global::ourY[Z] << '\n';
    std::cout << global::ourZ[X] << "   " << global::ourZ[Y] << "   " << global::ourZ[Z] << '\n';
    return 0;
}