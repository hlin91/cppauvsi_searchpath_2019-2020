/**
 * @file Polygon.h
 * @brief Polygon decomposition and traversal algorithm structs and functions.
 * Implements the greedy recursive approach using minimum width sum
 * outlined in a paper by Yan Li et al.
 * All units are in meters.
 * @author Harvey Lin
 */

#pragma once
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <list>
#include <string>
#include <sstream>
#include <fstream>
#include <assert.h>
#include <cfloat>
#include <utility>
#include "Graph.h"
#include "Config.h"

/**
 * @brief Approximate value for pi.
 */
#define PI 3.14159265358979323846
/**
 * @brief An effective infinity that will not overflow the float type.
 * @see float_type
 */
#define INF 1000000 // An effective infinity that will not overflow the float type.

/**
 * @brief State representing which vertex we start the path from.
 * START_V1 = start at vertex 1 of starting edge
 * START_V2 = start at vertex 2 of starting edge
 * END_V1 = start at vertex 1 of end edge
 * END_V2 = start at vertex 2 of end edge
 */
enum State {START_V1, START_V2, END_V1, END_V2};


//============================================================
// Prototypes
//============================================================
struct Coord; // A 2-D coordinate. Also used to represent the positional vector from the origin to the coordinate.
struct Edge; // An edge consisting of 2 coordinates.
struct Span; // A vertex-edge span of a polygon.
struct Polygon; // A polygon consisting of a list of coordinates in CCW order.
struct Node; // Node for the undirected weighted graph.

/**
 * @brief Find the distance between two vertices.
 * @param v1 the first vertex
 * @param v2 the second vertex
 * @return the distance between the two vertices
 * @see Coord float_type
 */
float_type distance(const Coord &v1, const Coord &v2);
/**
 * @brief  Find the distance between a vertex and an edge.
 * @param v the vertex
 * @param e the edge
 * @return the distance between the vertex and edge
 * @see Coord Edge float_type
 */
float_type distance(const Coord &v, const Edge &e);
// Helper calculation for finding the width of a polygon.
//inline float_type deltaDistance(const Polygon &p, int i, int j);
/**
 * @brief Find the width of a convex polygon.
 * @param p the polygon
 * @return the width of the polygon as a span
 * @see Span Polygon
 */
Span getWidth(const Polygon &p);
/**
 * @brief Determine if a given vertex of a polygon is concave.
 * @param p the polygon
 * @param i index of the vertex
 * @return true if the vertex is concave, else false
 * @see Polygon
 */
bool isConcave(const Polygon &p, int i);
/**
 * @brief Splits a polygon into two along an edge.
 * @param p the polygon
 * @param v1 first vertex of edge
 * @param v2 second vertex of edge
 * @param p1 stores the first resulting polygon
 * @param p2 stores the second resulting polygon
 * @return resulting polygons are stored in p1 and p2
 * @see Polygon
 */
void split(const Polygon &p, int v1, int v2, Polygon &p1, Polygon &p2);
/**
 * @brief Decompose a concave polygon into multiple convex polygons.
 * @param p the polygon
 * @param l stores the resulting list of polygons
 * @result resulting polygons are stored in l
 * @see Polygon
 */
void decompose(const Polygon &p, std::list<Polygon> &l);
/**
 * @brief Merge two polygons by their shared edge given the edge's index in each polygon and return the result
 * @param p1 the first polygon
 * @param p2 the second polygon
 * @param i index of the shared edge in p1
 * @param j index of the shared edge in p2
 * @return the merged polygon
 * @see Polygon
 */
Polygon merge(const Polygon &p1, const Polygon &p2, unsigned int i, unsigned int j);
/**
 * @brief Combine applicable subregions after decomposition.
 * @param l list of decomposed polygon subregions
 * @see Polygon decompose
 */
void mergeSubregions(std::list<Polygon> &l);
/**
 * @brief Calculate the cross product of two vectors represented as coordinates.
 * @param c1 the first vector as a coordinate
 * @param c2 the second vector as a coordinate
 * @return the cross product
 * @see Coord float_type
 */
inline float_type cross(const Coord &c1, const Coord &c2);
// Find the intersection of two line segments (represented as Edges) and return the intersection or NULL if it does not exist.
// Params: edges to consider, reference to store the intersection coordinate
/**
 * @brief Find the intersection of two line segments.
 * @param e1 the first line segment as an edge
 * @param e2 the second line segment as an edge
 * @param intersect stores the intersection if one exists
 * @return returns true and stores result in intersect if intersection exists, else returns false
 * @see Coord Edge
 */
bool intersection(const Edge &e1, const Edge &e2, Coord &intersect);
/**
 * @brief Traverse a convex polygon and store the waypoints in a list as Edges.
 * @param p the polygon to traverse
 * @param waypoints list to store the traversal
 * @see Polygon Edge
 */
void traverse(const Polygon &p, std::list<Edge> &waypoints);
/**
 * @brief Helper function to compute the adjacencies and weights of the graph.
 * @param g the graph to compute
 * @see Graph
 */
void computeGraph(Graph<Node, float_type> &g);
/**
 * @brief Compute the total length of a graph traversal.
 * @param g the weighted graph
 * @param path the traversal path
 * @return the total length of the traversal
 * @see Graph float_type
 */
float_type traversalLength(const Graph<Node, float_type> &g, std::list<unsigned int> &path);
/**
 * @brief Compute the minimum cost traversal for the weighted graph.
 * @param g the weighted graph
 * @return the minimum cost traversal as a list of node indeces
 * @see Graph
 */
std::list<unsigned int> minTraversal(Graph<Node, float_type> &g);
/**
 * @brief Determine the start states of each node along the traversal.
 * @param path the traversal
 * @param g the weighted graph
 * @see State
 */
void computeStates(std::list<unsigned int> &path, Graph<Node, float_type> &g);
/**
 * @brief Generates the search path for a polygon.
 * @param p the polygon
 * @return the search path as a list of Coords
 * @see Coord
 */
std::list<Coord> searchPath(const Polygon &p);
/**
 * @brief Determine if the list of coordinates are in clockwise order.
 * @param v the list of coordinates
 * @return true if coordinates are clockwise, else false
 * @see Coord
 */
bool clockwise(const std::vector<Coord> &v);
/**
 * @brief Computes a path from one point to another that does not intersect the boundary polygon.
 * Assumes both points are inside the boundary polygon
 * @param point1 the first point
 * @param point2 the second point
 * @param boundary the boundary polygon
 * @return the path as a list of Coords
 * @see Coord Polygon
 */
std::list<Coord> pathTo(const Coord &point1, const Coord &point2, const Polygon &boundary);
/**
 * @brief Traverse the polygon using a simple parallel traversal.
 * @param p the polygon
 * @param waypoints stores the resulting waypoints of the traversal
 * @return resulting traversal is stored in waypoints
 * @see Polygon Edge
 */
void naiveTraverse(const Polygon &p, std::list<Edge> &waypoints);
/**
 * @brief Generates a search path for a polygon using naive traversal.
 * @param p the polygon
 * @return the search path as a list of Coords
 * @see Coord Polygon naiveTraverse
 */
std::list<Coord> naivePath(const Polygon &p);

//============================================================
// Structs
//============================================================
/**
 * @brief Simple representation of a 2D coordinate.
 */
struct Coord
{
    /**
     * @brief The X and Y values of the coordinate.
     */
    float_type x, y;

    /**
     * @brief Create a new Coord.
     */
    Coord(float_type xCoord = 0, float_type yCoord = 0)
    {
        x = xCoord;
        y = yCoord;
    }
    /**
     * @brief Format the coordinate as a string.
     * @return a string representation of the Coord
     */
    std::string str() const
    {
        std::ostringstream s;
        s << "(" << x << "," << y << ")";
        return s.str();
    }
    /**
     * @brief Overloaded == operator
     */
    bool operator==(const Coord &op) const
    { return ((x == op.x) && (y == op.y)); }
    /**
     * @brief Overloaded != operator
     */
    bool operator !=(const Coord &op) const
    { return !((*this) == op); }
    // Some arithmetic operators to aid in vector arithmetic
    /**
     * @brief Overloaded + operator
     */
    Coord operator+(const Coord &op) const
    { return Coord(x + op.x, y + op.y); }
    /**
     * @brief Overloaded - operator
     */
    Coord operator-(const Coord &op) const
    { return Coord(x - op.x, y - op.y); }
    /**
     * @brief Computes the dot product of two vectors.
     */
    float_type operator*(const Coord &op) const
    { return x * op.x + y * op.y; }
    /**
     * @brief Scalar multiplication.
     */
    Coord operator*(float_type mult) const
    { return Coord(mult * x, mult * y); }
    /**
     * @brief Return length of vector from origin to the coordinate.
     */
    float_type vectorLength() const
    { return sqrt((x * x) + (y * y)); }
};

/**
 * @brief Simple representation of an edge.
 */
struct Edge
{
    /**
     * @brief The vertices of the edge.
     * @see Coord
     */
    Coord v1, v2;
    /**
     * @brief Coefficients for the standard form of the line (ax + by + c).
     */
    float_type a, b, c;

    /**
     * @brief Default constructor.
     */
    Edge()
    {
        v1 = v2 = Coord();
        a = b = c = 0;
    }
    /**
     * @brief Construct an Edge given two Coords
     * @see Coord
     */
    Edge(Coord vert1, Coord vert2)
    {
        v1 = vert1;
        v2 = vert2;
        float_type m;
        if (v1.x == v2.x) // The edge is perfectly vertical
            m = INFINITY; // Avoid division by zero. This value for the slope is merely a placeholder in this case.
        else
            m = (vert2.y - vert1.y) / (vert2.x - vert1.x);
        // Assign the values for the coeffs of the line in standard form
        a = -m;
        b = 1;
        c = (m * vert1.x) - vert1.y;
    }
    /**
     * @brief Get the slope of the edge.
     * @return the slope of the edge
     * @see float_type
     */
    float_type slope() const
    { return -a; }
    /**
     * @brief Determine if the edge is vertical.
     * @return true if the edge is vertical, else false
     */
    bool isVertical() const
    { return v1.x == v2.x; }
    /**
     * @brief Get the length of the edge
     * @return the length of the edge
     * @see float_type
     */
    float_type length() const
    { return distance(v1, v2); }
    /**
     * @brief Get the angle of the edge from the horizontal in radians
     * @return the angle of the edge in radians
     * @see float_type
     */
    float_type theta() const
    {
        if (isVertical())
        {
            if (v1.y < v2.y)
                return PI / 2.0;
            else
                return -(PI / 2.0);
        }
        else if (v1.y == v2.y) // Edge is horizontal
        {
            if (v1.x < v2.x)
                return 0;
            else
                return PI;
        }
        else
            return atan2(v2.y - v1.y, v2.x - v1.x);
    }
    /**
     * @brief Overloaded == operator
     */
    bool operator==(const Edge &op) const
    { return ((v1 == op.v1) && (v2 == op.v2)) || ((v1 == op.v2) && (v2 == op.v1)); }
};

/**
 * @brief Representation of a span in Vertex-Edge form.
 */
struct Span
{
    /**
     * @brief The antipodal vertex of the span.
     */
    Coord v;
    /**
     * @brief The edge component of the span.
     */
    Edge e;

    /**
     * @brief Constructor
     */
    Span(Coord vert, Edge edge)
    {
        v = vert;
        e = edge;
    }
    /**
     * @brief Get the length of the span.
     * @return the length of the span
     * @see float_type
     */
    float_type length() const
    { return distance(v, e); }
    /**
     * @brief Get the angle of the span from the horizontal.
     * @return the angle of the span in radians
     * @see float_type
     */
    float_type theta() const // Assumes e.theta() returns a value in the range [-PI, PI]
    { return e.theta() + (PI / 2.0); }
    /**
     * @brief Add up the lengths of two spans
     * @return the sum of lengths
     */
    float_type operator+(const Span &op) const
    { return length() + op.length(); }
};

/**
 * @brief Represents a polygon as a list of vertices in CCW order.
 */
struct Polygon
{
    // Using std::vector to store the list of vertices for efficient random access. This can be replaced with a more memory efficient container later if necessary
    /**
     * @brief The vertices of the polygon in CCW order
     * @see Coord
     */
    std::vector<Coord> v;

    /**
     * @brief Add a vertex to the polygon
     * @param vert the vertex to add
     * @see Coord
     */
    void addVert(Coord vert)
    { v.push_back(vert); }
    /**
     * @brief Get the number of vertices in the polygon.
     * @return the number of vertices
     */
    unsigned int size() const
    { return v.size(); }
    /**
     * @brief Construct an edge of the polygon given the index of the first vertex.
     * @param i index of the first vertex
     * @return the corresponding edge
     * @see Edge
     */
    Edge edge(unsigned int i) const // Constructs and returns a specific edge of the polygon indexed by the index of the first vertex
    { return Edge(v.at(i), v.at((i + 1) % v.size())); }
    /**
     * @brief Determine if the polygon is adjacent.
     * @param p the polygon
     * @param edgeIndex1 if not null, stores the index of the shared edge for this polygon
     * @param edgeIndex2 if not null, stores the index of the shared edge for p
     * @return true if adjacent and stores indeces in edgeIndex1 and edgeIndex2 if not null, else false and stores -1 in edgeIndex1 and edgeIndex2 if not null
     */
    bool adjacent(Polygon &p, int *edgeIndex1 = NULL, int *edgeIndex2 = NULL) const // Return true if p is adjacent and store the indeces for the edge in edgeIndex1 and 2 respectively. Else, return false and set indices to -1.
    { // This is the O(nm) solution which should be good enough unless our polygons are crazy. If more speed is required, use a hashing solution for linear time complexity.
        for (unsigned int i = 0; i < v.size(); ++i)
        {
            Edge e1 = edge(i);
            for (unsigned int j = 0; j < p.size(); ++j)
            {
                Edge e2 = p.edge(j);
                if (e1 == e2)
                {
                    if (edgeIndex1 != NULL)
                        *edgeIndex1 = i;
                    if (edgeIndex2 != NULL)
                        *edgeIndex2 = j;
                    return true;
                }
            }           
        }
        if (edgeIndex1 != NULL)
            *edgeIndex1 = -1;
        if (edgeIndex2 != NULL)
            *edgeIndex2 = -1;
        return false;
    }
    /**
     * @brief Get the center of the polygon.
     * @return the approximate center of the polygon as a Coord
     * @see Coord
     */
    Coord center() const
    {
        // Return the center of the bounding box for the polygon
        // This is a fairly brittle approximation. For better results,
        // Find the intersection of all angle bisectors
        float_type xMin, xMax;
        float_type yMin, yMax;
        xMin = xMax = v[0].x;
        yMin = yMax = v[0].y;
        for (unsigned int i = 1; i < v.size(); ++i)
        {
            if (v[i].x < xMin)
                xMin = v[i].x;
            if (v[i].x > xMax)
                xMax = v[i].x;
            if (v[i].y < yMin)
                yMin = v[i].y;
            if (v[i].y > yMax)
                yMax = v[i].y;
        }
        return Coord(((xMin + xMax) / 2), ((yMin + yMax) / 2));
    }
    /**
     * @brief Get the string representation of the polygon.
     * @return the string representation of the polygon
     */
    std::string str() const
    {
        std::ostringstream result;
        for (unsigned int i = 0; i < size(); ++i)
            result << v[i].str() << '\n';
        return result.str();
    }
    /**
     * @brief Overloaded assignment operator
     */
    Polygon& operator=(const Polygon &op)
    {
        v = op.v;
        return *this;
    }
};

/**
 * @brief Node class for weighted graph.
 * Consists of the search path for the polygon, a pointer to the polygon, and the start state
 */
struct Node
{
    /**
     * @brief A pointer to the associated polygon
     * @see Polygon
     */
    Polygon *p;
    /**
     * @brief The traversal path for the associated polygon
     * @see Edge
     */
    std::list<Edge> path;
    /**
     * @brief The start state of the traversal
     * @see State
     */
    State startState;

    /**
     * @brief Constructor
     */
    Node(Polygon *poly = NULL, std::list<Edge> *waypoints = NULL, State state = START_V1)
    {
        p = poly;
        if (waypoints != NULL)
            path = *waypoints;
        startState = state;
    }
};

//============================================================
// Functions
//============================================================
float_type distance(const Coord &v1, const Coord &v2) // Find the distance between two vertices
{ return sqrt(pow((v2.y - v1.y), 2) + pow(v2.x - v1.x, 2)); }

float_type distance(const Coord &v, const Edge &e) // Find the distance between a vertex and an edge
{
    if (e.isVertical()) // Special case for distance from a perfectly vertical edge
        return abs(e.v1.x - v.x);
    float_type numer = abs((e.a * v.x) + (e.b * v.y) + e.c);
    float_type denom = sqrt(e.a * e.a + e.b * e.b);
    return numer / denom;
}

// inline float_type deltaDistance(const Polygon &p, int i, int j) // Helper calculation for finding antipodal vertices
// { return (p.v[(i + 1) % p.size()].y - p.v[i].y) * (p.v[(j + 1) % p.size()].x - p.v[j].x) - (p.v[(i + 1) % p.size()].x - p.v[i].x) * (p.v[(j + 1) % p.size()].y - p.v[j].y); }

// Span getWidth(const Polygon &p) // Find the width of a convex polygon in O(n)
// { // This is bugged and incorrect. Because the polygons are likely to not have a large amount of edges, we'll go with the simple O(n^2) solution for safety
//     // The width is the minimum length span
//     assert(p.size() > 2); // Ensure the polygon has at least 3 vertices
//     std::list<Span> spans;
//     unsigned int i = 0, j = 2;
//     float_type dDist1, dDist2;
//     Edge e(p.v[i], p.v[(i + 1) % p.size()]);
//     while (i < p.size()) // Compare the deltaDistances to get the span in V-E form for every edge
//     {
//      dDist1 = deltaDistance(p, i, (j % p.size()));
//      ++j;
//      if ((j % p.size()) == i) // We've wrapped around and there are no more vertices to consider
//      {
//          int prevIndex = j - 1;
//          while (prevIndex < 0)
//              prevIndex += p.size();
//          spans.push_back(Span(p.v[prevIndex], e)); // Set the previous vertex as the antipodal vertex for this edge
//          // Consider the next edge
//          float_type prevSlope = e.slope();
//          ++i;
//          e = Edge(p.v[i], p.v[(i + 1) % p.size()]);
//          while (e.slope() == prevSlope && i < p.size()) // Ignore adjacent edges with equivalent slope as these will yield an equivalent span
//          {
//              ++i;
//              e = Edge(p.v[i], p.v[(i + 1) % p.size()]);
//          }
//      }
//      else
//      {
//          dDist2 = deltaDistance(p, i, j % p.size());
//          if ((dDist1 >= 0 && dDist2 < 0) || (dDist1 <= 0 && dDist2 > 0)) // deltaDistance changed signs
//          {
//              // j is the antipodal vertex of edge i, (i + 1)
//              spans.push_back(Span(p.v[j % p.size()], e)); // Construct the span of edge e with antipodal vertex j
//              float_type prevSlope = e.slope();
//              ++i;
//              e = Edge(p.v[i], p.v[(i + 1) % p.size()]);
//              while (e.slope() == prevSlope && i < p.size())
//              {
//                  ++i;
//                  e = Edge(p.v[i % p.size()], p.v[(i + 1) % p.size()]);
//              }
//          }
//      }
//     }
//     // Get the minimum length span
//     Span width = spans.front();
//     spans.pop_front();
//     while (!spans.empty())
//     {
//      if (spans.front().length() < width.length())
//          width = spans.front();
//      spans.pop_front();
//     }
//     return width;
// }

Span getWidth(const Polygon &p) // Simple O(n^2) implementation
{
    std::vector<Span> spans; // List of candidate spans
    unsigned int minSpan = -1;
    float_type minLength = -1;
    for (unsigned int i = 0; i < p.size(); ++i)
    {
        Edge e = p.edge(i);
        Coord maxVert;
        float_type maxDistance = -1;
        for (unsigned int j = 2; j < p.size(); ++j)
        {
            float_type currDistance = distance(p.v[(i + j) % p.size()], e);
            if (currDistance > maxDistance)
            {
                maxDistance = currDistance;
                maxVert = p.v[(i + j) % p.size()];
            }
        }
        spans.push_back(Span(maxVert, e));
    }
    // Get the min length span
    for (unsigned int i = 0; i < spans.size(); ++i)
    {
        if (spans[i].length() < minLength || minLength == -1 )
        {
            minLength = spans[i].length();
            minSpan = i;
        }
    }
    return spans[minSpan];
}

bool isConcave(const Polygon &p, int i) // Determine if vertex i is concave or not
{
    assert(i >= 0 && (unsigned int)i < p.size());
    // Let vertex i be B, vertex i - 1 be A, and vertex i + 1 be C
    // Get the coordinates of vectors BA and BC
    int prevIndex = i - 1;
    while (prevIndex < 0)
        prevIndex += p.size();
    float_type BAx = p.v[(prevIndex) % p.size()].x - p.v[i].x;
    float_type BAy = p.v[(prevIndex) % p.size()].y - p.v[i].y;
    float_type BCx = p.v[(i + 1) % p.size()].x - p.v[i].x;
    float_type BCy = p.v[(i + 1) % p.size()].y - p.v[i].y;
    // Use the sign of the Z-coord of the cross product to determine concavity
    // Since we are visiting the vertices CCW, vertex i is concave if Z-coord > 0
    return ((BAx * BCy - BAy * BCx) > 0);
}

void split(const Polygon &p, int v1, int v2, Polygon &p1, Polygon &p2) // Splits p by edge v1, v2 and stores result in p1 and p2
{
    assert((v1 >= 0 && v2 >= 0) && ((unsigned int)v1 < p.size()) && ((unsigned int)v2 < p.size())); // Make sure indices are in bounds
    if (abs(v1 - v2) < 2) // Invalid edge
        return;
    if (v1 > v2) // Ensure that v1 is the lower index
    {
        unsigned int temp = v1;
        v1 = v2;
        v2 = temp;
    }
    // p1 will store the polygon formed from v1 to v2, p2 will store the one formed from v2 to v1
    p1.v.clear(); p2.v.clear(); // Re-initialize the vertex lists for p1 and p2
    for (int i = v1; i <= v2; ++i) // Fill p1 with the vertices from v1 to v2
        p1.addVert(p.v[i]);
    for (int i = v2; (i % p.v.size()) != (v1 + 1); ++i) // Fill p2 with the vertices from v2 to v1
        p2.addVert(p.v[i % p.v.size()]);
}

void decompose(const Polygon &p, std::list<Polygon> &l) // Convex polygon decomposition algorithm
// Decomposes concave polygon p by adding a new edge between a concave vertex and convex vertex so as to produce the minimum width sum
{
    // Uncomment std::cout statements for debugging
    assert(p.size() > 2);
    bool acceptConvex = false;
    std::vector<unsigned int> concaveVerts;
    Polygon p1, p2; // The resulting polygons from splitting p
    unsigned int v1 = 0, v2 = 0; // The vertices the polygon will be split at
    float_type minWidthSum = -1;
    for (unsigned int i = 0; i < p.size(); ++i) // Get the concave vertex indices of the polygon
        if (isConcave(p, i))
            concaveVerts.push_back(i);
    if (concaveVerts.size() == 0) // If the polygon is convex, return it
    {
        l.push_back(p);
        return;
    }
    // We'll use the concave vertex to other concave vertex style of splitting the polygon for now
    // Should this not work out, we can try a different style (eg. concave vertex to edge)
    // DEBUG PRINTS
    //---
    // std::cout << "ncc = " << concaveVerts.size() << "\n";
    // std::cout << "Concave verts...\n";
    // for (unsigned int i = 0; i < concaveVerts.size(); ++i)
    //  std::cout << p.v[concaveVerts[i]].str() << ", ";
    // std::cout << "\nDecomposing Polygon...\n" << p.str() << "\n";
    //---
    if (concaveVerts.size() == 1) // Split by convex vertices if there is only one concave vertex
        acceptConvex = true;
    while (minWidthSum == -1)
    {
        for (unsigned int i = 0; i < concaveVerts.size(); ++i)
        { // Find the split that produces the minimum width sum
            for (unsigned int j = 0; j < p.size(); ++j)
            {
                bool adjacent = false;
                if (((concaveVerts[i] + 1) % p.size()) == j)
                    adjacent = true;
                int prevIndex = concaveVerts[i] - 1;
                while (prevIndex < 0)
                    prevIndex += p.size();
                if (prevIndex == (int)j)
                    adjacent = true;
                if ((concaveVerts[i] != j) && !adjacent && (isConcave(p, j) || acceptConvex)) // Ignore vertices that would produce an invalid split
                {
                    // Check against the interior angle to ensure a valid split
                    bool valid;
                    Edge splitEdge(p.v[concaveVerts[i]], p.v[j]);
                    float_type splitTheta = splitEdge.theta();
                    float_type theta1 = p.edge(concaveVerts[i]).theta();
                    float_type theta2 = p.edge(prevIndex % p.size()).theta();
                    if (theta2 - PI < EPSILON)
                        theta2 = 0;
                    else
                        theta2 += PI;
                    if (theta1 > theta2)
                    {
                        valid = true;
                        if (splitTheta > theta2 && splitTheta < theta1)
                            valid = false;
                    }
                    else
                    {
                        valid = false;
                        if (splitTheta >= theta1 && splitTheta <= theta2)
                            valid = true;
                    }
                    if (valid)
                    {
                        //std::cout << "Splitting at " << p.v[concaveVerts[i]].str() << ", " << p.v[j].str() << "\n";
                        split(p, concaveVerts[i], j, p1, p2);
                        //std::cout << "P1...\n" << p1.str() << "\n";
                        //std::cout << "P2...\n" << p2.str() << "\n";
                        float_type widthSum = getWidth(p1) + getWidth(p2);
                        if (widthSum < minWidthSum || minWidthSum < 0)
                        {
                            minWidthSum = widthSum;
                            v1 = concaveVerts[i];
                            v2 = j;
                        }
                    }
                }
            }
        }
        if (minWidthSum == -1) // If we can't split concave to concave, try to split concave to convex
            acceptConvex = true;
    }
    // std::cout << "Splitting at " << p.v[v1].str() << " " << p.v[v2].str() << "\n\n";
    split(p, v1, v2, p1, p2); // Split polygon p to produce the minimum sum width
    decompose(p1, l); // Decompose those polygons in turn
    decompose(p2, l);
}

Polygon merge(const Polygon &p1, const Polygon &p2, unsigned int i, unsigned int j) // Merge two polygons by shared edge at index i of p1 and j of p2 and return the result
{
    Polygon result;
    // v1 of the shared edge is p1.v[i] and v2 is p1.v[(i + 1) % p1.size()]
    for (unsigned int z = 0; z < p1.size(); ++z) // Add the vertices of p1 into result starting from v2
        result.addVert(p1.v[(i + 1 + z) % p1.size()]);
    for (unsigned int z = 1; z < (p2.size() - 1); ++z) // Do the same relative to p2
        result.addVert(p2.v[(j + 1 + z) % p2.size()]);
    return result;
}

void mergeSubregions(std::list<Polygon> &l) // Merge subregions that are adjacent and combine to form a convex polygon
{
    int i, j;
    std::list<Polygon>::iterator it1 = l.begin();
    while (it1 != l.end())
    {
        std::list<Polygon>::iterator it2 = l.begin();
        while(it2 != l.end())
        {
            if (it1 != it2 && it1 != l.end() && it2 != l.end())
            {
                if (it1->adjacent(*it2, &i, &j))
                {
                    Polygon mergedPoly = merge(*it1, *it2, i, j);
                    unsigned int numConcave = 0;
                    for (unsigned int i = 0; i < mergedPoly.size(); ++i)
                        if (isConcave(mergedPoly, i))
                            ++numConcave;
                    if (numConcave == 0)
                    {
                        *it1 = mergedPoly;
                        l.erase(it2);
                        continue;
                    }
                }
            }
            ++it2;
        }
        ++it1;
    }
}

inline float_type cross(const Coord &c1, const Coord &c2) // Cross two coordinates by treating them as positional vectors and return the result
{ return c1.x * c2.y - c2.x * c1.y; }

bool intersection(const Edge &e1, const Edge &e2, Coord &intersect) // Finds the intersection of two line segments and stores the result in intersect.
// Otherwise, intersect will be NULL. We will treat collinear lines as non-intersecting and return null.
{
    // References this: https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/565282#565282
    // and this: https://www.codeproject.com/tips/862988/find-the-intersection-point-of-two-line-segments
    // We'll use the naming convention used in the reference posts
    Coord p1 = e1.v1; Coord p2 = e1.v2;
    Coord q1 = e2.v1; Coord q2 = e2.v2;
    Coord r = Coord(p2.x - p1.x, p2.y - p1.y);
    Coord s = Coord(q2.x - q1.x, q2.y - q1.y);
    float_type rxs = cross(r, s);
    float_type qpxr = cross(Coord(q1.x - p1.x, q1.y - p1.y), r);
    if (abs(rxs) < EPSILON && abs(qpxr) < EPSILON) // The two edges are collinear
        return false; // Treat this as no intersection even if they might overlap
    if (abs(rxs) < EPSILON && !(abs(qpxr) < EPSILON)) // The two lines are parallel and non-intersecting
        return false;
    float_type t = cross(q1 - p1, s) / rxs;
    float_type u = cross (q1 - p1, r) / rxs;
    if (!(abs(rxs) < EPSILON) && (0 <= t && t <= 1) && (0 <= u && u <= 1)) // The two segments meet at point p1 + tr = q1 + us
    {
        intersect = (p1 + (r * t));
        return true;
    }
    return false; // No intersection was found
}

void traverse(const Polygon &p, std::list<Edge> &waypoints) // Traverse convex polygon p and store the waypoints as Edges in a list
{
    assert(p.size() > 2);
    Span width = getWidth(p);
    Edge sweepLine = width.e;
    // Extend sweepLine to INF
    if (sweepLine.isVertical())
    {
        sweepLine.v1.y = -INF;
        sweepLine.v2.y = INF;
    }
    else
    {
        if (sweepLine.v1.x < sweepLine.v2.x) // The edge points left to right
        {
            sweepLine.v1.x -= INF;
            sweepLine.v1.y -= INF * sweepLine.slope();
            sweepLine.v2.x += INF;
            sweepLine.v2.y += INF * sweepLine.slope();
        }
        else // The edge points right to left
        {
            sweepLine.v2.x -= INF;
            sweepLine.v2.y -= INF * sweepLine.slope();
            sweepLine.v1.x += INF;
            sweepLine.v1.y += INF * sweepLine.slope();
        }
    }
    Coord inter1, inter2; // The intersections of the sweep line with the polygon
    unsigned int i = 0, j = 0;
    bool found1 = false, found2 = false; // Were intersections 1 and 2 found
    // The sweep line is currently collinear with width.e and extends to INF.
    // Repeatedly find the intersections of the sweep line + OFFSET and store the intersections as waypoints until we do not find anymore intersections.
    sweepLine.v1.x += OFFSET * cos(width.theta()); sweepLine.v2.x += OFFSET * cos(width.theta()); // Initially offset the position of the sweep line by OFFSET / 2
    sweepLine.v1.y += OFFSET * sin(width.theta()); sweepLine.v2.y += OFFSET * sin(width.theta());
    do
    {
        i = 0;
        found1 = false;
        found2 = false;
        do
        { // Find the first intersection
            found1 = intersection(sweepLine, p.edge(i), inter1);
            ++i;
        } while (i < p.size() && (!found1));
        while (i < p.size() && (!found2))
        { // Find the second intersection
            found2 = intersection(sweepLine, p.edge(i), inter2);
            ++i;
        }
        if (found1 && found2)
        {
            bool valid = true; // Is this pair of waypoints still valid after correction
            Edge beforeCorr(inter1, inter2); // The edge representing the path before correction
            // Shift waypoints inward to account for turn radius
            // Correct the x-coord
            if (inter2.x > inter1.x) // Waypoints are ordered left to right
            {
                inter2.x -= abs(CORRECTION * cos(sweepLine.theta()));
                inter1.x += abs(CORRECTION * cos(sweepLine.theta()));
            }
            else // Waypoints are ordered right to left
            {
                inter2.x += abs(CORRECTION * cos(sweepLine.theta()));
                inter1.x -= abs(CORRECTION * cos(sweepLine.theta()));
            }
            // Correct the y-coord
            if (inter2.y > inter1.y)
            {
                inter2.y -= abs(CORRECTION * sin(sweepLine.theta()));
                inter1.y += abs(CORRECTION * sin(sweepLine.theta()));
            }
            else
            {
                inter2.y += abs(CORRECTION * sin(sweepLine.theta()));
                inter1.y -= abs(CORRECTION * sin(sweepLine.theta()));
            }
            // Check if the waypoints have crossed each other after correction
            Edge afterCorr(inter1, inter2); // The path after correction
            if (abs(beforeCorr.theta()) < EPSILON) // For horizontal paths, simply check the x-coords
            {
                if ((beforeCorr.v1.x > beforeCorr.v2.x && afterCorr.v1.x <= afterCorr.v2.x) || (beforeCorr.v1.x < beforeCorr.v2.x && afterCorr.v1.x >= afterCorr.v2.x))
                    valid = false;
            }
            else // Else check if theta() changed signs
            {
                if ((beforeCorr.theta() > 0 && afterCorr.theta() < 0) || (beforeCorr.theta() < 0 && afterCorr.theta() > 0))
                    valid = false;
            }
            // Add the waypoints to the list
            if (valid)
            {
                // If j is even, make pair (inter1, inter2), else (inter2, inter1)
                if ((j % 2) == 0)
                    waypoints.push_back(Edge(inter1, inter2)); 
                else
                    waypoints.push_back(Edge(inter2, inter1));
            }
        }
        // OFFSET the sweep line
        // std::cout << "Offset: " << "(" << OFFSET * cos(width.theta()) << "," << OFFSET * sin(width.theta()) << ")\n";
        sweepLine.v1.x += OFFSET * cos(width.theta()); sweepLine.v2.x += OFFSET * cos(width.theta());
        sweepLine.v1.y += OFFSET * sin(width.theta()); sweepLine.v2.y += OFFSET * sin(width.theta());
        ++j;
    } while (found1);
    // Ensure that there is at least RADIUS worth of vertical clearance for the vertices of the last edge. Else, remove the last edge (Might not be necessary)
    if (waypoints.size() > 0)
    {   
        std::list<Edge>::reverse_iterator lastEdge = waypoints.rbegin();
        // Check v1
        // std::cout << "Width Theta: "<< width.theta() << '\n';
        Coord testCoord((*lastEdge).v1.x, (*lastEdge).v1.y);
        testCoord.x += RADIUS * cos(width.theta());
        testCoord.y += RADIUS * sin(width.theta());
        Edge testEdge((*lastEdge).v1, testCoord);
        for (unsigned int i = 0; i < p.size(); ++i)
        {
            if (intersection(testEdge, p.edge(i), inter1)) // Not enough clearance
            {
                waypoints.pop_back(); // Delete the last edge
                return;
            }
        }
        // Check v2
        testCoord.x = (*lastEdge).v2.x;
        testCoord.y = (*lastEdge).v2.y;
        testCoord.x += RADIUS * cos(width.theta());
        testCoord.y += RADIUS * sin(width.theta());
        testEdge = Edge((*lastEdge).v2, testCoord);
        for (unsigned int i = 0; i < p.size(); ++i)
        {
            if (intersection(testEdge, p.edge(i), inter1))
            {
                waypoints.pop_back();
                break;
            }
        }
    }
}

void computeGraph(Graph<Node, float_type> &g) // Fill in the edges and weights of the graph based on the adjacencies and distances between joint points
{
    // Find the adjacencies
    for (unsigned int i = 0; i < g.size(); ++i)
        for (unsigned int j = 0; j < g.size(); ++j)
        {
            if (i == j)
                g.w[i][j] = 0;
            else
                g.w[i][j] = INF + distance(g.v[i].p->center(), g.v[j].p->center()); // Initialize the weights
            // Compute the adjacencies
            if (!(g.e[i][j]) && g.v[i].p->adjacent(*(g.v[j].p))) // adjacent() may need to be optimized if this runs too slow
            {
                g.setEdge(i, j);
                g.setEdge(j, i);
            }
        }
    // Compute the weights
    for (unsigned int i = 0; i < g.size(); ++i)
        for (unsigned int j = 0; j < g.size(); ++j)
            if (g.e[i][j])
                g.w[i][j] = distance(g.v[i].p->center(), g.v[j].p->center()); // Use the distance of centers as a heuristic for the distances between two subregions
}

float_type traversalLength(const Graph<Node, float_type> &g, std::list<unsigned int> &path)
{
    assert(path.size() > 0);
    float_type length = 0;
    unsigned int i, j;
    std::list<unsigned int>::iterator it = path.begin();
    for (unsigned int k = 0; k < (path.size() - 1); ++k)
    {
        i = *it;
        ++it;
        j = *it;
        length += g.w[i][j];
    }
    return length;
}

std::list<unsigned int> minTraversal(Graph<Node, float_type> &g) // Computes the minimum cost traversal for the weighted graph as a list of indeces
{
    // We'll just brute force this as this is the fastest approach for graphs with 5 or less nodes and keeps things simple
    std::list<unsigned int> bestPath;
    float_type minDistance = -1;
    std::list<unsigned int> verts;
    for (unsigned int i = 0; i < g.size(); ++i) // Construct a list of verts to generate all permutations
        verts.push_back(i);
    do
    {
        float_type currDistance = traversalLength(g, verts);
        if (currDistance < minDistance || minDistance == -1)
        {
            bestPath = verts;
            minDistance = currDistance;
        }
    } while (std::next_permutation(verts.begin(), verts.end()));
    return bestPath;
}

void computeStates(std::list<unsigned int> &path, Graph<Node, float_type> &g) // Determine the start states of subregions in minimum traversal to optimally link the path
{
    // Set the state of first subregion by comparing the distance from joint points to the center of second subregion
    float_type dist;
    std::list<unsigned int>::iterator it = path.begin();
    g.v[*it].startState = START_V1;
    dist = distance(g.v[*it].path.back().v2, g.v[*(std::next(it))].p->center());
    if (distance(g.v[*it].path.back().v1, g.v[*(std::next(it))].p->center()) < dist)
    {
        dist = distance(g.v[*it].path.back().v1, g.v[*(std::next(it))].p->center());
        g.v[*it].startState = START_V2;
    }
    if (distance(g.v[*it].path.front().v2, g.v[*(std::next(it))].p->center()) < dist)
    {
        dist = distance(g.v[*it].path.front().v2, g.v[*(std::next(it))].p->center());
        g.v[*it].startState = END_V1;
    }
    if (distance(g.v[*it].path.front().v1, g.v[*(std::next(it))].p->center()) < dist)
    {
        dist = distance(g.v[*it].path.front().v1, g.v[*(std::next(it))].p->center());
        g.v[*it].startState = END_V2;
    }
    for (unsigned int i = 0; i < path.size() - 1; ++i)
    {
        Coord joint; // The joint vertex of subregion i that we will measure distances from depends on the start state of i
        switch (g.v[*it].startState)
        {
        case START_V1:
            joint = g.v[*it].path.back().v2;
            break;
        case START_V2:
            joint = g.v[*it].path.back().v1;
            break;
        case END_V1:
            joint = g.v[*it].path.front().v2;
            break;
        case END_V2:
            joint = g.v[*it].path.front().v1;
            break;
        default:
            std::cout << "Warning: Unknown start state encountered\n";
        }
        // Find the joint vertex of the next subregion that gives the minimum linear distance
        ++it;
        g.v[*it].startState = START_V1;
        dist = distance(joint, g.v[*it].path.front().v1);
        if (dist > distance(joint, g.v[*it].path.front().v2))
        {
            g.v[*it].startState = START_V2;
            dist = distance(joint, g.v[*it].path.front().v2);
        }
        if (dist > distance(joint, g.v[*it].path.back().v1))
        {
            g.v[*it].startState = END_V1;
            dist = distance(joint, g.v[*it].path.back().v1);
        }
        if (dist > distance(joint, g.v[*it].path.back().v2))
            g.v[*it].startState = END_V2;
    }
}

std::list<Coord> searchPath(const Polygon &p) // Generates a search path for arbitrary polygon p
{
    std::list<Coord> path;
    std::list<Polygon> subregions;
    unsigned int numConcave = 0;
    decompose(p, subregions); // Decompose p into subregions
    mergeSubregions(subregions); // Merge adjacent subregions with the same width
    for (unsigned int i = 0; i < p.v.size(); ++i)
        if (isConcave(p, i))
            ++numConcave;
    if (numConcave == 0) // If the polygon is already convex, just traverse it
    {
        std::list<Edge> trav;
        traverse(p, trav);
        for (std::list<Edge>::iterator e = trav.begin(); e != trav.end(); ++e)
        {
            path.push_back(e->v1);
            path.push_back(e->v2);
        }
        return path;
    }
    Graph<Node, float_type> g(subregions.size());
    // Construct the list of nodes
    unsigned int i = 0;
    for (std::list<Polygon>::iterator it = subregions.begin(); it != subregions.end(); ++it) // Associate each node's pointer with a subregion
    {
        g.v[i].p = &(*it);
        ++i;
    }
    i = 0;
    while (i < subregions.size()) // Get the traversals for each subregion
    {
        traverse(*(g.v[i].p), g.v[i].path);
        ++i;
    }
    computeGraph(g); // Compute the edges and weights
    std::list<unsigned int> travOrder = minTraversal(g); // Get the min traversal for the graph
    computeStates(travOrder, g); // Compute the start states of each node
    for (std::list<unsigned int>::iterator it = travOrder.begin(); it != travOrder.end(); ++it)
    {
        unsigned int j = *it;
        if (g.v[j].path.size() > 0)
        {
            switch (g.v[j].startState) // Write out each subregion's path. The start state of p will affect the order in which waypoints are read.
            {
            case START_V1: // Read search path as normal. End point is final edge v2.
                for (std::list<Edge>::iterator e = g.v[j].path.begin(); e != g.v[j].path.end(); ++e)
                {
                    path.push_back(e->v1);
                    path.push_back(e->v2);
                }
                break;
            case START_V2: // Read edges in order, but vertices in reverse. End point is final edge v1.
                for (std::list<Edge>::iterator e = g.v[j].path.begin(); e != g.v[j].path.end(); ++e)
                {
                    path.push_back(e->v2);
                    path.push_back(e->v1);
                }
                break;
            case END_V1: // Read edges in reverse but vertices in order. End point is start edge v2.
                for (std::list<Edge>::reverse_iterator e = g.v[j].path.rbegin(); e != g.v[j].path.rend(); ++e)
                {
                    path.push_back(e->v1);
                    path.push_back(e->v2);
                }
                break;
            case END_V2: // Read edges in reverse and vertices in reverse. End point is start edge v1.
                for (std::list<Edge>::reverse_iterator e = g.v[j].path.rbegin(); e != g.v[j].path.rend(); ++e)
                {
                    path.push_back(e->v2);
                    path.push_back(e->v1);
                }
                break;
            default: // Unknown state
                std::cout << "Warning: Unknown state encountered\n";
            }
        }
    }
    return path;
}


bool clockwise(const std::vector<Coord> &v) // Return true if the coordinates are in clockwise order, else false
{
    // Reference: https://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
    // Assumes +y axis is upwards
    int sum = 0;
    for (unsigned int i = 0; i < v.size(); ++i)
        sum += (v[(i + 1) % v.size()].x - v[i].x) * (v[(i + 1) % v.size()].y + v[i].y);
    return sum > 0;
}

struct Comp // Simple comparator for sorting in pathTo
{
    Coord point;
    
    Comp(Coord c)
    { point = c; }
    bool operator()(std::pair<unsigned int, Coord> it1, std::pair<unsigned int, Coord> it2)
    { return distance(point, it1.second) < distance(point, it2.second); }
    bool operator()(Coord c1, Coord c2)
    { return distance(point, c1) < distance(point, c2); }
};

void pathToHelp(std::list<Coord> &path, std::list<Coord>::iterator point1, std::list<Coord>::iterator point2, const Polygon &boundary) // Workhorse function for pathTo to recursively unfold intersections
{
    Edge line(*point1, *point2);
    Coord inter;
    std::vector<std::pair<unsigned int, Coord> > intersects;
    std::list<Coord> result;
    Comp comp(*point1);
    for (unsigned int i = 0; i < boundary.size(); ++i) // Check for intersections and store them
        if (intersection(line, boundary.edge(i), inter))
            intersects.push_back(std::make_pair(i, inter));
    if (intersects.size() == 0)
        return;
    else
    {
        std::sort(intersects.begin(), intersects.end(), comp); // Sort intersects by distance from point1
        for (std::vector<std::pair<unsigned int, Coord> >::iterator it = intersects.begin(); it != intersects.end(); ++it) // Offset each current intersection by RADIUS in the desired direction
        {
            it->second.x += RADIUS * cos(boundary.edge(it->first).theta() + (PI / 2.0));
            it->second.y += RADIUS * sin(boundary.edge(it->first).theta() + (PI / 2.0));
        }
        // OPTIONAL: Add check to see if the offset intersects still lie within the boundary. If not, remove them. If intersects.size() becomes 0, return.
        for (std::vector<std::pair<unsigned int, Coord> >::iterator it = intersects.begin(); it != intersects.end(); ++it) // Add the offset intersections to the path
            path.insert(point2, it->second);
        for (std::list<Coord>::iterator it = path.begin(); std::next(it) != path.end(); ++it) // Unfold intersections in each of the subpaths
            pathToHelp(path, it, std::next(it), boundary);
    }
}

std::list<Coord> pathTo(const Coord &point1, const Coord &point2, const Polygon &boundary) // Generate path from point1 to point2 that does not intersect boundary
{
    std::list<Coord> result;
    result.push_back(point1);
    result.push_back(point2);
    std::list<Coord>::iterator it1 = result.begin();
    std::list<Coord>::iterator it2 = std::next(it1);
    pathToHelp(result, it1, it2, boundary);
    result.pop_front(); // Remove the terminal points
    result.pop_back();
    return result;
}

void naiveTraverse(const Polygon &p, std::list<Edge> &waypoints) // Traverse the polygon using a simple East-West traversal
{
    // The logic for this is just a slight modification of traverse()
    assert(p.size() > 2);
    Edge sweepLine;
    int minY;
    // Find the vertex in the polygon with the minimum y value
    minY = p.v[0].y;
    for (unsigned int i = 1; i < p.size(); ++i)
        if (p.v[i].y < minY)
            minY = p.v[i].y;
    // Define infinite horizontal sweep line
    sweepLine.v1 = Coord(-INF, minY);
    sweepLine.v2 = Coord(INF, minY);
    Coord inter; // The intersections of the sweep line with the polygon
    unsigned int i = 0, j = 0;
    bool found = true; // Did we find at least one intersection
    struct
    {
        bool operator()(Coord &c1, Coord &c2)
        { return c1.x < c2.x; }
    } cmp; // Compare the x values of coordinates
    // Repeatedly find the intersections of the sweep line + OFFSET and store the intersections as waypoints until we do not find anymore intersections.
    // Initially offset the position of the sweep line by OFFSET / 2
    sweepLine.v1.y += (OFFSET / 2.0); sweepLine.v2.y += (OFFSET / 2.0);
    do
    {
        found = false;
        std::vector<Coord> intersections;
        for (unsigned int i = 0; i < p.size(); ++i)
        {
            if (intersection(p.edge(i), sweepLine, inter))
            {
                found = true; // Found at least one intersection
                intersections.push_back(inter);
            }
        }
        if (intersections.size() >= 2)
        {
            // There may be more than 2 intersections so we only want the ones with the min and max x-value
            // To make this more compact, we'll just do a sort and find these in O(nlogn)
            std::sort(intersections.begin(), intersections.end(), cmp);
            bool valid = true; // Is this pair of waypoints still valid after correction
            Coord inter1 = intersections.front();
            Coord inter2 = intersections.back();
            Edge beforeCorr(inter1, inter2); // The edge representing the path before correction
            // Shift waypoints inward to account for turn radius
            // Because we sorted, the waypoints will always be ordered left-right
            // Correct the x-coord
            inter2.x -= CORRECTION;
            inter1.x += CORRECTION;
            // Check if the waypoints have crossed each other after correction
            Edge afterCorr(inter1, inter2); // The path after correction
            if (afterCorr.v1.x >= afterCorr.v2.x)
                valid = false;
            // Add the waypoints to the list
            if (valid)
            {
                // If j is even, make pair (inter1, inter2), else (inter2, inter1)
                if ((j % 2) == 0)
                    waypoints.push_back(Edge(inter1, inter2)); 
                else
                    waypoints.push_back(Edge(inter2, inter1));
            }
        }
        // OFFSET the sweep line
        sweepLine.v1.y += (OFFSET / 2.0); sweepLine.v2.y += (OFFSET / 2.0);
        ++j;
    } while (found);
}

std::list<Coord> naivePath(const Polygon &p)
{
    std::list<Edge> waypoints;
    std::list<Coord> path;
    naiveTraverse(p, waypoints);
    for (std::list<Edge>::iterator e = waypoints.begin(); e != waypoints.end(); ++e)
    {
        path.push_back(e->v1);
        path.push_back(e->v2);
    }
    return path;
}

// int main(int argc, char **argv) // Test driver
// {
//     // Remember to change the value of OFFSET and CORRECTION in Config.h
//     Polygon p;
//     p.addVert(Coord(0, 0));
//     p.addVert(Coord(10, 0));
//     p.addVert(Coord(10, 5));
//     p.addVert(Coord(5, 2.5));
//     p.addVert(Coord(0, 10));
//     std::list<Coord> path = naivePath(p);
//     std::cout << path.size() << std::endl;
//     for (auto &c : path)
//         std::cout << "(" << c.x << "," << c.y << ")\n";
//     return 0;
// }
