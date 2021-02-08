/**
 * @file main.cpp
 * @brief Main driver for search path generation.
 * Pass the optional argument "naive" to use naive path generation with no decomposition.
 * Pass either no argument or "decomp" to use path generation with convex polygon decomposition.
 * @author Harvey Lin
 */

#include "Polygon.cpp"
#include "Conversions.cpp"
#include <cctype>
#include <iomanip>

// ---
// Main
// ---
int main(int argc, char **argv)
{
    if (argc > 2)
    {
        std::cout << "Error: Too many arguments passed\n";
        return 1;
    }
    
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
    init(longitude, latitude);
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
    if (argc == 1) // Default behavior for no arguments. Use decomposition
        path = searchPath(searchArea);
    else
    {
        if (strcmp(argv[1], "naive")) // Use naive traversal
            path = naivePath(searchArea);
        else if (strcmp(argv[1], "decomp")) // Use decomposition
            path = searchPath(searchArea);
        else
        {
            std::cout << "Error: Invalid arugment passed\n";
            std::cout << "Available options: naive, decomp\n";
            return 1;
        }
    }
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

