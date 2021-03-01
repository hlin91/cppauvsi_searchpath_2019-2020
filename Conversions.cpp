/**
 * @file Conversions.cpp
 * @brief Provides functions for converting GPS coordinates to and from 2D coordinates.
 * Converts coordinates to 2D points on the plane tangent to the Earth approximated as a sphere
 * at the anchor coordinate.
 * @author Harvey Lin
 */
#pragma once
#include "Polygon.cpp"
#include <vector>
#define EARTH_RADIUS 6378137 // Radius of the Earth in meters

//================================================================
// Prototypes, Constants, and Global Variables for
// GPS to Cartesian conversion
//================================================================
/**
 * @brief Initialize global reference vectors used for basis vector conversion from an anchor coordinate.
 * Call this before anything else.
 * @param longitude the longitude of the anchor coordinate in radians
 * @param latitude the latitude of the anchor coordinate in radians
 */
void init(float_type longitude, float_type latitude);
/**
 * @brief Converts GPS longitude and latitude to 3D Cartesian coordinates with standard basis vectors.
 * Origin is at the center of the Earth
 * @param longitude the longitude of the coordinate in radians
 * @param latitude the latitude of the coordinate in radians
 */
std::vector<float_type> GPStoCartesian(const float_type longitude, const float_type latitude);
/**
 * @brief Converts GPS longitude and latitude to 2D Cartesian coordinates measured in meters.
 * @param longitude the longitude of the coordinate in radians
 * @param latitude the latitude of the coordinate in radians
 * @return equivalent 2D coordinate
 * @see Coord
 */
Coord GPStoCoord(const float_type longitude, const float_type latitude); // Converts GPS longitude and latitude to Cartesian coordinates measured in meters.
/**
 * @brief Converts a 2D coordinate back to GPS longitude and latitude.
 * @param c coordinate to convert
 * @param longitude stores the resulting longitude
 * @param latitude stores the resulting latitude
 * @return resulting longitude is stored in longitude, latitude is stored in latitude
 */
void CoordtoGPS(const Coord &c, float_type &longitude, float_type &latitude); // Converts a coordinate in our Cartesian system to GPS longitude and latitude
/**
 * @brief Compute the basis vectors for our Cartesian system. Make sure this is run after init at the start of main.
 * @see init
 */
inline void computeBasis(); // Compute the basis vectors for our Cartesian system. Make sure this is run at the start of main.
/**
 * @brief Convert degrees to radians.
 * @param degrees degrees to convert
 * @return equivalent in radians
 * @see float_type
 */
inline float_type toRadians(float_type degrees); // Convert degrees to radians
/**
 * @brief Convert radians to degrees.
 * @param radians radians to convert
 * @return requivalent in degrees
 * @see float_type
 */
inline float_type toDegrees(float_type radians); // Convert radians to degrees
/**
 * @brief Convert feet to meters.
 * @param feet feet to convert
 * @return equivalent in meters
 * @see float_type
 */
inline float_type toMeters(float_type feet); // Convert feet to meters
/**
 * @brief Convert meters to feet.
 * @param meters meters to convert
 * @return equivalent in feet
 * @see float_type
 */
inline float_type toFeet(float_type meters); // Convert meters to feet

/**
 * @brief Indeces used to store X, Y, and Z coordinates in a vector.
 */
enum {X = 0, Y = 1, Z = 2};
namespace conv // Global constants used to keep function calls simple
{
    /**
     * @brief GPS reference point in standard basis 3D Cartesian coordinates.
     */
    std::vector<float_type> refCart;
    /**
     * @brief Basis vectors for our cartesian system.
     */
    std::vector<float_type> ourX(3), ourY(3), ourZ(3);
    /**
     * @brief Conversion matrix from standard basis to our basis.
     */
    float_type convMatrix[3][3] = {};
    /**
     * @brief Longitude of GPS coordinate used as the origin of our Cartesian system in radians.
     */
    float_type REF_LONG = 0;
    /**
     * @brief Latitude of GPS coordinate used as the origin of our Cartesian system in radians.
     */
    float_type REF_LAT = 0;
}

//================================================================
// Definitions
//================================================================
void init(float_type longitude, float_type latitude)
{
    conv::REF_LONG = longitude;
    conv::REF_LAT = latitude;
    conv::refCart = GPStoCartesian(conv::REF_LONG, conv::REF_LAT);
}

std::vector<float_type> GPStoCartesian(const float_type longitude, const float_type latitude) // Converts GPS to 3D Cartesian coordinates with standard basis vectors
{ // IMPORTANT: Assumes longitude and latitude are given in radians
    // Reference: https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
    std::vector<float_type> coord;
    coord.push_back(EARTH_RADIUS * cos(latitude) * cos(longitude)); // x coordinate
    coord.push_back(EARTH_RADIUS * cos(latitude) * sin(longitude)); // y coordinate
    coord.push_back(EARTH_RADIUS * sin(latitude)); // z coordinate
    return coord;
 }

Coord GPStoCoord(const float_type longitude, const float_type latitude) // Converts GPS long and alt to Cartesian coordinates measured in meters
{
    // Reference: https://stackoverflow.com/questions/1185408/converting-from-longitude-latitude-to-cartesian-coordinates#1185413
    // Right multiply the GPS coordinate converted to standard basis Cartesian with the conversion matrix constructed from our new basis vectors
    std::vector<float_type> standardCart = GPStoCartesian(longitude, latitude); // The coordinate with standard basis vectors
    Coord result;
    result.x = result.y = 0;
    // Shift so that our reference coordinate is the origin
    // std::cout << "GPS Cartesian (before shift): " << standardCart[X] << ", " << standardCart[Y] << ", " << standardCart[Z] << '\n';
    standardCart[X] -= conv::refCart[X];
    standardCart[Y] -= conv::refCart[Y];
    standardCart[Z] -= conv::refCart[Z];
    // Convert to our basis vectors
    // Because of how we defined our basis vectors, we should be able to disregard the resulting Z-coordinate since it will be approximately 0
    result.x = conv::convMatrix[0][0] * standardCart[X] + conv::convMatrix[0][1] * standardCart[Y] + conv::convMatrix[0][2] * standardCart[Z];
    result.y = conv::convMatrix[1][0] * standardCart[X] + conv::convMatrix[1][1] * standardCart[Y] + conv::convMatrix[1][2] * standardCart[Z];
    // std::cout statement for debugging
    // std::cout << "After conversion: " << result.x << ", " << result.y << ", "
    //   	      << conv::convMatrix[2][0] * standardCart[X] + conv::convMatrix[2][1] * standardCart[Y] + conv::convMatrix[2][2] * standardCart[Z] << '\n';
    return result;
}

void CoordtoGPS(const Coord& c, float_type &longitude, float_type &latitude) // Converts a coordinate in our coordinate system to GPS longitude and latitude in radians
{
    float_type standardCoord[3] = {};
    standardCoord[X] = c.x * conv::ourX[X] + c.y * conv::ourY[X]; // Get the X value in standard basis
    standardCoord[Y] = c.x * conv::ourX[Y] + c.y * conv::ourY[Y]; // Get the Y value in standard basis
    standardCoord[Z] = c.x * conv::ourX[Z] + c.y * conv::ourY[Z]; // Get the Z value in standard basis
    // Shift the origin back to the center of the Earth
    standardCoord[X] += conv::refCart[X];
    standardCoord[Y] += conv::refCart[Y];
    standardCoord[Z] += conv::refCart[Z];
    longitude = atan2(standardCoord[Y], standardCoord[X]);
    latitude = asin(standardCoord[Z] / EARTH_RADIUS);
}

void computeBasis() // Compute our basis vectors based on the given reference point
{
    // Compute our Z basis vector
    conv::ourZ = conv::refCart;
    float_type length = sqrt(conv::ourZ[X] * conv::ourZ[X] + conv::ourZ[Y] * conv::ourZ[Y] + conv::ourZ[Z] * conv::ourZ[Z]);
    // Compute our X basis vector using equation of the plane at reference point
    conv::ourX[X] = (1.0);
    conv::ourX[Y] = (0);
    conv::ourX[Z] = ((conv::ourZ[X] * conv::refCart[X] - conv::ourZ[X] * conv::ourX[X] + conv::ourZ[Y] * conv::refCart[Y] - conv::ourZ[Y] * conv::ourX[Y] + conv::ourZ[Z] * conv::refCart[Z]) / conv::ourZ[Z]);
    // Make our X the positional vector anchored at conv::refCart and pointing towards the point we just calculated on the plane
    conv::ourX[X] -= conv::refCart[X];
    conv::ourX[Y] -= conv::refCart[Y];
    conv::ourX[Z] -= conv::refCart[Z];
    // Cross our X and our Z to get our Y
    conv::ourY[X] = (conv::ourZ[Y] * conv::ourX[Z] - conv::ourZ[Z] * conv::ourX[Y]);
    conv::ourY[Y] = (-(conv::ourZ[X] * conv::ourX[Z] - conv::ourZ[Z] * conv::ourX[X]));
    conv::ourY[Z] = (conv::ourZ[X] * conv::ourX[Y] - conv::ourZ[Y] * conv::ourX[X]);
    // Scale basis vectors to unit length
    for (unsigned int i = 0; i < conv::ourZ.size(); ++i)
	conv::ourZ[i] /= length;
    length = sqrt(conv::ourX[X] * conv::ourX[X] + conv::ourX[Y] * conv::ourX[Y] + conv::ourX[Z] * conv::ourX[Z]);
    for (unsigned int i = 0; i < conv::ourX.size(); ++i)
	conv::ourX[i] /= length;
    length = sqrt(conv::ourY[X] * conv::ourY[X] + conv::ourY[Y] * conv::ourY[Y] + conv::ourY[Z] * conv::ourY[Z]);
    for (unsigned int i = 0; i < conv::ourY.size(); ++i)
	conv::ourY[i] /= length;
    // Compute the conversion matrix by inverting the standard matrix
    float_type standardMatrix[3][3] =
	{
	    {conv::ourX[X], conv::ourY[X], conv::ourZ[X]},
	    {conv::ourX[Y], conv::ourY[Y], conv::ourZ[Y]},
	    {conv::ourX[Z], conv::ourY[Z], conv::ourZ[Z]}
	};
    float_type determinate = (standardMatrix[0][0] * (standardMatrix[1][1] * standardMatrix[2][2] - standardMatrix[1][2] * standardMatrix[2][1])) -
	(standardMatrix[0][1] * (standardMatrix[1][0] * standardMatrix[2][2] - standardMatrix[1][2] * standardMatrix[2][0])) +
	(standardMatrix[0][2] * (standardMatrix[1][0] * standardMatrix[2][1] - standardMatrix[1][1] * standardMatrix[2][0]));
    for (unsigned int i = 0; i < 3; ++i) // Transpose the standard matrix
	for (unsigned int j = i; j < 3; ++j)
	    std::swap(standardMatrix[i][j], standardMatrix[j][i]);
    conv::convMatrix[0][0] = standardMatrix[1][1] * standardMatrix[2][2] - standardMatrix[1][2] * standardMatrix[2][1];
    conv::convMatrix[0][1] = standardMatrix[1][0] * standardMatrix[2][2] - standardMatrix[1][2] * standardMatrix[2][0];
    conv::convMatrix[0][2] = standardMatrix[1][0] * standardMatrix[2][1] - standardMatrix[1][1] * standardMatrix[2][0];
    conv::convMatrix[1][0] = standardMatrix[0][1] * standardMatrix[2][2] - standardMatrix[0][2] * standardMatrix[2][1];
    conv::convMatrix[1][1] = standardMatrix[0][0] * standardMatrix[2][2] - standardMatrix[0][2] * standardMatrix[2][0];
    conv::convMatrix[1][2] = standardMatrix[0][0] * standardMatrix[2][1] - standardMatrix[0][1] * standardMatrix[2][0];
    conv::convMatrix[2][0] = standardMatrix[0][1] * standardMatrix[1][2] - standardMatrix[0][2] * standardMatrix[1][1];
    conv::convMatrix[2][1] = standardMatrix[0][0] * standardMatrix[1][2] - standardMatrix[0][2] * standardMatrix[1][0];
    conv::convMatrix[2][2] = standardMatrix[0][0] * standardMatrix[1][1] - standardMatrix[0][1] * standardMatrix[1][0];
    for (unsigned int i = 0; i < 3; ++i)
	for (unsigned int j = 0; j < 3; ++j)
	    conv::convMatrix[i][j] /= determinate;
    conv::convMatrix[0][1] *= -1;
    conv::convMatrix[1][0] *= -1;
    conv::convMatrix[1][2] *= -1;
    conv::convMatrix[2][1] *= -1;
}


inline float_type toRadians(float_type degrees) // Degrees to radians
{ return degrees * (PI / 180.0); }

inline float_type toDegrees(float_type radians) // Radians to degrees
{ return radians * (180.0 / PI); }

inline float_type toMeters(float_type feet) // Feet to meters
{ return feet * 0.3048; }

inline float_type toFeet(float_type meters) // Meters to feet
{ return meters * 3.28084; }
