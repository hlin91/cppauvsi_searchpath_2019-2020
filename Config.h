//================================================================
// Pre-defined values to configure the behavior of the algorithm
//================================================================

#define OUT_FILE "C:\\Users\\Public\\Downloads\\InteropCode\\interop\\MissionPointsWithSearch.txt" // Our output file path
#define MISSION_FILE "C:\\Users\\Public\\Downloads\\InteropCode\\interop\\MissionPointsParsed.txt" // Mission points file path
#define BOUNDS_FILE "C:\\Users\\Public\\Downloads\\InteropCode\\interop\\BoundaryPoints.txt" // Boundary points file path
#define SEARCH_FILE "C:\\Users\\Public\\Downloads\\InteropCode\\interop\\SearchGridParsed.txt" // Search grid points file path
#define ALTITUDE 150 // Output altitude in feet
#define RADIUS 36.6 // The turn radius of the drone in meters
#define OFFSET RADIUS // The spacing between each sweep line in meters. Minimum value is RADIUS
#define CORRECTION RADIUS // Distance search path waypoints are scaled inward to avoid exiting the boundary
