<h1>Search Path Algorithm for Cal Poly Pomona AUVSI 2019-2020</h1>
<p>
  Based on the minimum width sum greedy recursive method outlined in the <a href="https://www.sciencedirect.com/science/article/abs/pii/S0957415810001893">paper</a> by Yan Li et al.
</p>
<h2>Table of Contents</h2>
<p>
  <ol>
    <li><a href="#docs">Documentation</a></li>
    <li><a href="#dependencies">Dependencies</a></li>
    <li><a href="#compilation">Compilation Notes</a></li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#config">Configuration</a></li>
    <li><a href="#debug">Notes for Debugging</a></li>
  </ol>
</p>
<h2 id="docs">Documentation</h2>
<p>
  For detailed documentation, click <a href="hlin91.github.io/cppauvsi_searchpath_2019-2020/">here</a>.
</p>
<h2 id="dependencies">Dependencies</h2>
<p>
  None. But it is not standard for compilers to provide atof() under the cctype header so it may throw an error depending on the system. If this is a problem, replace calls to atof() with std::stof().
</p>
<h2 id="compilation">Compilation Notes</h2>
<p>
  <ul>
    <li>Make sure not to forget the O2 flag when calling the compiler to enable compiler optimizations since it's free speed</li>
    <li>To compile on Windows, open the developer command prompt from Visual Studio and type <strong>cl \O2 main.cpp</strong></li>
    <li>To compile on UNIX systems, just use g++ or clang</li>
  </ul>
</p>
<h2 id="usage">Usage</h2>
<p>
  Make sure all files are present in their expected paths and run the executable. To use naive path generation with no decomposition, pass the optional argument <code>naive</code> when calling the executable. This will generate a predictable East-West sweep that does not attempt to stay within the boundary of the search area. To use path generation with decomposition, pass no argument or pass the optional argument <code>decomp</code>.
</p>
<h2 id="config">Configuration</h2>
<p>
  To configure behavior, edit <strong>Config.h</strong>.<br>
  <strong>Note: Make sure to recompile for changes to take effect.</strong>
  <ul>
    <li>To change the output file path, change the #define statement for <strong>OUT_FILE</strong></li>
    <li>To change the MissionPointsParsed file path, change the #define statement for <strong>MISSION_FILE</strong></li>
    <li>To change the BoundaryPointsParsed file path, change the #define statement for <strong>BOUNDS_FILE</strong></li>
    <li>To change the SearchGridPoints file path, change the #define statement for <strong>SEARCH_FILE</strong></li>
    <li>To change the output altitude in feet, change the #define statement for <strong>ALTITUDE</strong></li>
    <li>To change the assumed turn radius (in METERS) of the drone, change the #define statement for <strong>RADIUS</strong></li>
    <li>To change the offset spacing between each parallel sweep of a traversal (in METERS), change the #define statement for <strong>OFFSET</strong></li>
    <li>To change the distance waypoints are scaled inward to avoid exiting the boundary, change the #define statement for <strong>CORRECTION</strong></li>
    <li>In the case that lines from mission files overflow the character buffer used in main.cpp, increase the value of the #define statement for <strong>BUFF_MAX</strong></li>
  </ul>
</p>
<h2 id="debug">Notes for Debugging</h2>
<p>
  <ul>
    <li>main.cpp is the main driver and handles file I/O and calls the necessary functions for search path generation</li>
    <li>Conversions.cpp contains functions for handling conversions from GPS lat long coordinates to 2-D Cartesian coordinates and vis versa</li>
    <li>Polygon.cpp contains structs and functions for implementing polygon decomposition and search path generation. The most relevant functions for client code are searchPath(), naivePath(), and pathTo()</li>
  </ul>
</p>
