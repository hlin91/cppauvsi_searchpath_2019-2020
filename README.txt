Search Path Algorithm for Cal Poly Pomona AUVSI 2019-2020

Compile-time Dependencies: None, but it is not standard for compilers to provide atof() under the cctype header so it may throw an error depending on the system

Usage:
 Make sure all files are present in their expected paths and run the executable.
 To use naive path generation with no decomposition, pass the optional argument "naive" when calling the executable.
 To use path generation with decomposition, pass no argument or pass the optional argument "decomp".

Configuration: To configure behavior, edit Config.h
** Note: Make sure to recompile for changes to take effect. **
- To change the output file path, change the #define statement for OUT_FILE
- To change the MissionPointsParsed file path, change the #define statement for MISSION_FILE
- To change the BoundaryPointsParsed file path, change the #define statement for BOUNDS_FILE
- To change the SearchGridPoints file path, change the #define statement for SEARCH_FILE
- To change the output altitude in feet, change the #define statement for ALTITUDE
- To change the assumed turn radius (in METERS) of the drone, change the #define statement for RADIUS
- To change the offset spacing between each parallel sweep of a traversal (in METERS), change the #define statement for OFFSET
- To change the distance waypoints are scaled inward to avoid exiting the boundary, change the #define statement for CORRECTION

Notes on Compilation:
- Make sure not to forget the O2 flag when calling the compiler to enable compiler optimizations since it's free speed
- To compile on Windows...
  Visual Studio 2019 (RECOMMENDED): Open the developer command prompt from the Tools menu and type "cl \O2 main.cpp"
  Visual Studio 2017: Just make a new project, paste everything in, and build with optimizations. Haven't tested this myself so I can't guarantee that it wont throw an error.
- To compile on UNIX systems...
  Just use g++ or clang

Things that could be improved...
- The minimum distance traversal of sub-regions is found through a n! brute force. This is the best solution for the kinds of polygons we are likely to encounter
  but a better solution will have to be implemented for traversing very complex polygons.
- The edge connecting adjacent sub-regions in the graph is weighted by the distance between the centers of each regions bounding box. While this is a fast approximation,
  calculating the true geometric center may be more accurate.
- In the event that the graph of sub-regions cannot be traversed without traveling to a nonadjacent sub-region, no effort is made to generate a path that will avoid exiting
  the search grid or boundary polygon. Adding calls to pathTo() in searchPath() can fix this if needed.
- pathTo() assumes that each intersection of the sub-path with the boundary polygon can be scaled by a distance of 2 * RADIUS inward without crossing each other or exiting
  the boundary polygon since it was assumed that the turn radius is very small relative to the dimensions of the polygon. Additional checks will have to be added if this
  becomes a problem.
