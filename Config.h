/**
 * @file Config.h
 * @author Harvey Lin
 * @brief Configuration header.
 * This file contains declarations used to configure the behavior of the program.
 */

/**
 * @brief The resulting search path will be written to this file.
 */
#define OUT_FILE "C:\\Users\\Public\\Downloads\\AUVSI-MissionPlanner-2020-2021\\flightplan\\mission\\MissionPointsWithSearch.txt"
/**
 * @brief The initial mission points are read from this file.
 */
#define MISSION_FILE "C:\\Users\\Public\\Downloads\\AUVSI-MissionPlanner-2020-2021\\flightplan\\mission\\MissionPointsParsed.txt"
/**
 * @brief The coordinates of the boundary polygon are read from this file.
 */
#define BOUNDS_FILE "C:\\Users\\Public\\Downloads\\AUVSI-MissionPlanner-2020-2021\\flightplan\\mission\\BoundaryPoints.txt"
/**
 * The coordinates of the search area are read from this file.
 */
#define SEARCH_FILE "C:\\Users\\Public\\Downloads\\AUVSI-MissionPlanner-2020-2021\\flightplan\\mission\\SearchGridParsed.txt"
/**
 * The output altitude of the drone for the search path in feet.
 */
#define ALTITUDE 150
/**
 * The turn radius of the drone in meters.
 */
#define RADIUS 36.6
/**
 * The spacing between each sweep line in meters. Minimum value this can be is the value of RADIUS.
 * @see RADIUS
 */
#define OFFSET RADIUS
/**
 * Distance search path waypoints are scaled inward to avoid exiting the boundary.
 */
#define CORRECTION RADIUS
/**
 * Max number of characters for lines read from mission files.
 */
#define BUFF_MAX 30
/**
 * Epsilon value for the float type we are using.
 */
#define EPSILON DBL_EPSILON
/**
 * The float type we are using.
 */
typedef double float_type;
