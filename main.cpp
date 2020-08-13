//================================================================
// Main driver for search path generation
//================================================================
#include "Polygon.cpp"
#include <cctype>
#include <iomanip>

#define EARTH_RADIUS 6378137 // Radius of the Earth in meters
#define OUT_FILE "C:\\Users\\Public\\Downloads\\InteropCode\\interop\\MissionPointsWithSearch.txt" // Our output file path
#define MISSION_FILE "C:\\Users\\Public\\Downloads\\InteropCode\\interop\\MissionPointsParsed.txt" // Mission points file path
#define BOUNDS_FILE "C:\\Users\\Public\\Downloads\\InteropCode\\interop\\BoundaryPoints.txt" // Boundary points file path
#define SEARCH_FILE "C:\\Users\\Public\\Downloads\\InteropCode\\interop\\SearchGridParsed.txt" // Search grid points file path
#define BUFF_MAX 30 // Max size of c-string buffer
#define ALTITUDE 150 // Output altitude in feet

//================================================================
// Prototypes, Constants, and Global Variables for
// GPS to Cartesian conversion
//================================================================
std::vector<float_type> GPStoCartesian(const float_type, const float_type); // Converts GPS longitude and latitude to Cartesian coordinates with standard basis vectors and origin at the center of the Earth.
Coord GPStoCoord(const float_type, const float_type); // Converts GPS longitude and latitude to Cartesian coordinates measured in meters using the Haversine formula.
void CoordtoGPS(const Coord&, float_type&, float_type&); // Converts a coordinate in our Cartesian system to GPS longitude and latitude
inline void computeBasis(); // Compute the basis vectors for our Cartesian system. Make sure this is run at the start of main.
inline float_type toRadians(float_type); // Convert degrees to radians
inline float_type toDegrees(float_type); // Convert radians to degrees
inline float_type toMeters(float_type); // Convert feet to meters
inline float_type toFeet(float_type); // Convert meters to feet

enum {X = 0, Y = 1, Z = 2};
namespace global // Global constants used to keep function calls simple
{
    std::vector<float_type> refCart; // GPS reference point in standard basis Cartesian coordinates
    std::vector<float_type> ourX, ourY, ourZ; // Basis vectors for our cartesian system
    float_type convMatrix[3][3] = {}; // Conversion matrix from standard basis to our basis
    float_type REF_LONG = 0; // Longitude of GPS coordinate used as the origin of our Cartesian system in radians
    float_type REF_LAT = 0; // Latitude of GPS coordinate used as the origin of our Cartesian system in radians
}

//================================================================
// Main
//================================================================
int main()
{
    float_type altitude;
    int ordinal;
    float_type longitude, latitude;
    char input[BUFF_MAX] = {};
    Polygon searchArea; // The search grid polygon
    Polygon boundary; // The boundary polygon
    Coord lastMissionPoint; // The coordinate the drone will be in before pathing to the search grid
    std::list<Coord> intermPath; // Path from lastMissionPoint to first point in search path
    std::list<Coord> path; // The search path
    std::ifstream missionFile(MISSION_FILE);
    std::ifstream searchFile(SEARCH_FILE);
    std::ifstream boundsFile(BOUNDS_FILE);
    std::ofstream outFile(OUT_FILE);
    unsigned int i = 1;
    if (!missionFile)
    {
	std::cout << "Could not open mission file.\n";
	return 1;
    }
    if (!searchFile)
    {
	std::cout << "Could not open search grid file.\n";
	return 1;
    }
    if (!boundsFile)
    {
	std::cout << "Could not open boundary points file.\n";
	return 1;
    }
    if (!outFile)
    {
	std::cout << "Could not create output file.\n";
	return 1;
    }
    
    // Read from searchFile
    // Use the first search grid coordinate read as the origin point of our Cartesian system
    searchFile.getline(input, BUFF_MAX, ','); // Get the ordinal number
    ordinal = (int) atof(input);
    searchFile.getline(input, BUFF_MAX, ','); // Get latitude
    latitude = toRadians(atof(input));
    searchFile.getline(input, BUFF_MAX, ','); // Get longitude
    longitude = toRadians(atof(input));
    // Set the reference longitude, latitude, and cartesian coordinate
    global::REF_LONG = longitude;
    global::REF_LAT = latitude;
    global::refCart = GPStoCartesian(longitude, latitude);
    computeBasis(); // Compute the basis vectors
    searchArea.v.push_back(Coord(0, 0)); // Treat the first coordinate read as the origin
    while(!searchFile.eof())
    {
	searchFile.getline(input, BUFF_MAX, ','); // Get the ordinal number
	ordinal = (int) atof(input);
	searchFile.getline(input, BUFF_MAX, ','); // Get latitude
	latitude = toRadians(atof(input));
	searchFile.getline(input, BUFF_MAX, ','); // Get longitude
	longitude = toRadians(atof(input));
	searchArea.v.push_back(GPStoCoord(longitude, latitude));
    }
    searchFile.close();
    if (clockwise(searchArea.v)) // Ensure points are in counter-clockwise order
	std::reverse(searchArea.v.begin(), searchArea.v.end());
    
    // Read from boundaryFile
    while(!boundsFile.eof())
    {
	boundsFile.getline(input, BUFF_MAX, ','); // Get the ordinal number
	ordinal = (int) atof(input);
	boundsFile.getline(input, BUFF_MAX, ','); // Get latitude
	latitude = toRadians(atof(input));
	boundsFile.getline(input, BUFF_MAX, ','); // Get longitude
	longitude = toRadians(atof(input));
	boundary.v.push_back(GPStoCoord(longitude, latitude));
    }
    boundsFile.close();
    if (clockwise(boundary.v))
	std::reverse(boundary.v.begin(), boundary.v.end());

    // Read from missionFile
    outFile << std::fixed << std::setprecision(7);
    while (!missionFile.eof())
    {	
	missionFile.getline(input, BUFF_MAX, ','); // Get the ordinal number
	ordinal = (int) atof(input);
	missionFile.getline(input, BUFF_MAX, ','); // Get latitude
	latitude = toRadians(atof(input));
	missionFile.getline(input, BUFF_MAX, ','); // Get longitude
	longitude = toRadians(atof(input));
	missionFile.getline(input, BUFF_MAX, ','); // Get altitude
	altitude = atof(input);
	if (i != 1)
	    outFile << ',';
	outFile << i << ',' << toDegrees(latitude) << ',' << toDegrees(longitude) << ',' << (int) altitude; // Duplicate MissionPointsParsed into our output file
	++i;
    }
    missionFile.close();
    lastMissionPoint = GPStoCoord(longitude, latitude);

    // Generate paths
    path = searchPath(searchArea);
    intermPath = pathTo(lastMissionPoint, path.front(), boundary);

    // Write output
    for (std::list<Coord>::iterator it = intermPath.begin(); it != intermPath.end(); ++it)
    {
	CoordtoGPS(*it, longitude, latitude);
	if (i != 1)
	    outFile << ',';
	outFile << i << ',' << toDegrees(latitude) << ',' << toDegrees(longitude) << ',' << ALTITUDE;
	++i;
    }
    for (std::list<Coord>::iterator it = path.begin(); it != path.end(); ++it)
    {
	CoordtoGPS(*it, longitude, latitude);
	if (i != 1)
	    outFile << ',';
	outFile << i << ',' << toDegrees(latitude) << ',' << toDegrees(longitude) << ',' << ALTITUDE;
	++i;
    }
    outFile.close();
    return 0;
}

//================================================================
// Definitions
//================================================================
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
    standardCart[X] -= global::refCart[X];
    standardCart[Y] -= global::refCart[Y];
    standardCart[Z] -= global::refCart[Z];
    // Convert to our basis vectors
    // Because of how we defined our basis vectors, we should be able to disregard the resulting Z-coordinate since it will be approximately 0
    result.x = global::convMatrix[0][0] * standardCart[X] + global::convMatrix[0][1] * standardCart[Y] + global::convMatrix[0][2] + standardCart[Z];
    result.y = global::convMatrix[1][0] * standardCart[X] + global::convMatrix[1][1] * standardCart[Y] + global::convMatrix[1][2] * standardCart[Z];
    // std::cout statement for debugging
    // std::cout << "After conversion: " << result.x << ", " << result.y << ", "
    //  	      << global::convMatrix[2][0] * standardCart[X] + global::convMatrix[2][1] * standardCart[Y] + global::convMatrix[2][2] * standardCart[Z] << '\n';
    return result;
}

void CoordtoGPS(const Coord& c, float_type &longitude, float_type &latitude) // Converts a coordinate in our coordinate system to GPS longitude and latitude in radians
{
    float_type standardCoord[3] = {};
    standardCoord[X] = c.x * global::ourX[X] + c.y * global::ourY[X]; // Get the X value in standard basis
    standardCoord[Y] = c.x * global::ourX[Y] + c.y * global::ourY[Y]; // Get the Y value in standard basis
    standardCoord[Z] = c.x * global::ourX[Z] + c.y * global::ourY[Z]; // Get the Z value in standard basis
    // Shift the origin back to the center of the Earth
    standardCoord[X] += global::refCart[X];
    standardCoord[Y] += global::refCart[Y];
    standardCoord[Z] += global::refCart[Z];
    longitude = atan2(standardCoord[Y], standardCoord[X]);
    latitude = asin(standardCoord[Z] / EARTH_RADIUS);
}

void computeBasis() // Compute our basis vectors based on the given reference point
{
    // Compute our Z basis vector
    global::ourZ = global::refCart;
    float_type length = sqrt(global::ourZ[X] * global::ourZ[X] + global::ourZ[Y] * global::ourZ[Y] + global::ourZ[Z] * global::ourZ[Z]);
    // Compute our X basis vector using equation of the plane at reference point
    global::ourX.push_back(1.0);
    global::ourX.push_back(0);
    global::ourX.push_back((global::ourZ[X] * global::refCart[X] - global::ourZ[X] * global::ourX[X] + global::ourZ[Y] * global::refCart[Y] - global::ourZ[Y] * global::ourX[Y] + global::ourZ[Z] * global::refCart[Z]) / global::ourZ[Z]);
    // Make our X the positional vector anchored at global::refCart and pointing towards the point we just calculated on the plane
    global::ourX[X] -= global::refCart[X];
    global::ourX[Y] -= global::refCart[Y];
    global::ourX[Z] -= global::refCart[Z];
    // Cross our X and our Z to get our Y
    global::ourY.push_back(global::ourZ[Y] * global::ourX[Z] - global::ourZ[Z] * global::ourX[Y]);
    global::ourY.push_back(-(global::ourZ[X] * global::ourX[Z] - global::ourZ[Z] * global::ourX[X]));
    global::ourY.push_back(global::ourZ[X] * global::ourX[Y] - global::ourZ[Y] * global::ourX[X]);
    for (unsigned int i = 0; i < global::ourZ.size(); ++i)
	global::ourZ[i] /= length;
    length = sqrt(global::ourX[X] * global::ourX[X] + global::ourX[Y] * global::ourX[Y] + global::ourX[Z] * global::ourX[Z]);
    for (unsigned int i = 0; i < global::ourX.size(); ++i)
	global::ourX[i] /= length;
    length = sqrt(global::ourY[X] * global::ourY[X] + global::ourY[Y] * global::ourY[Y] + global::ourY[Z] * global::ourY[Z]);
    for (unsigned int i = 0; i < global::ourY.size(); ++i)
	global::ourY[i] /= length;
    // Compute the conversion matrix by inverting the standard matrix
    float_type standardMatrix[3][3] =
	{
	    {global::ourX[X], global::ourY[X], global::ourZ[X]},
	    {global::ourX[Y], global::ourY[Y], global::ourZ[Y]},
	    {global::ourX[Z], global::ourY[Z], global::ourZ[Z]}
	};
    float_type determinate = (standardMatrix[0][0] * (standardMatrix[1][1] * standardMatrix[2][2] - standardMatrix[1][2] * standardMatrix[2][1])) -
	(standardMatrix[0][1] * (standardMatrix[1][0] * standardMatrix[2][2] - standardMatrix[1][2] * standardMatrix[2][0])) +
	(standardMatrix[0][2] * (standardMatrix[1][0] * standardMatrix[2][1] - standardMatrix[1][1] * standardMatrix[2][0]));
    for (unsigned int i = 0; i < 3; ++i) // Transpose the standard matrix
	for (unsigned int j = i; j < 3; ++j)
	    std::swap(standardMatrix[i][j], standardMatrix[j][i]);
    global::convMatrix[0][0] = standardMatrix[1][1] * standardMatrix[2][2] - standardMatrix[1][2] * standardMatrix[2][1];
    global::convMatrix[0][1] = standardMatrix[1][0] * standardMatrix[2][2] - standardMatrix[1][2] * standardMatrix[2][0];
    global::convMatrix[0][2] = standardMatrix[1][0] * standardMatrix[2][1] - standardMatrix[1][1] * standardMatrix[2][0];
    global::convMatrix[1][0] = standardMatrix[0][1] * standardMatrix[2][2] - standardMatrix[0][2] * standardMatrix[2][1];
    global::convMatrix[1][1] = standardMatrix[0][0] * standardMatrix[2][2] - standardMatrix[0][2] * standardMatrix[2][0];
    global::convMatrix[1][2] = standardMatrix[0][0] * standardMatrix[2][1] - standardMatrix[0][1] * standardMatrix[2][0];
    global::convMatrix[2][0] = standardMatrix[0][1] * standardMatrix[1][2] - standardMatrix[0][2] * standardMatrix[1][1];
    global::convMatrix[2][1] = standardMatrix[0][0] * standardMatrix[1][2] - standardMatrix[0][2] * standardMatrix[1][0];
    global::convMatrix[2][2] = standardMatrix[0][0] * standardMatrix[1][1] - standardMatrix[0][1] * standardMatrix[1][0];
    for (unsigned int i = 0; i < 3; ++i)
	for (unsigned int j = 0; j < 3; ++j)
	    global::convMatrix[i][j] /= determinate;
    global::convMatrix[0][1] *= -1;
    global::convMatrix[1][0] *= -1;
    global::convMatrix[1][2] *= -1;
    global::convMatrix[2][1] *= -1;
}


inline float_type toRadians(float_type degrees) // Degrees to radians
{ return degrees * (PI / 180.0); }

inline float_type toDegrees(float_type radians) // Radians to degrees
{ return radians * (180.0 / PI); }

inline float_type toMeters(float_type feet) // Feet to meters
{ return feet * 0.3048; }

inline float_type toFeet(float_type meters) // Meters to feet
{ return meters * 3.28084; }
