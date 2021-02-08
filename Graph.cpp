/**
 * @file Graph.cpp
 * @brief Simple weighted directed graph implementation.
 * @author Harvey Lin
 */
#include <cstdlib>

/**
 * @brief A simple weighted directed graph.
 */
template <typename E, typename W>
struct Graph 
{
    /**
     * @brief The number of vertices stored in the graph.
     */
    const unsigned int numVerts;
    /**
     * @brief The list of vertices.
     */
    E *v;
    /**
     * @brief Adjacency matrix representing edges in the graph.
     */
    bool **e;
    /**
     * @brief Weight matrix for each directional edge.
     */
    W **w;

    /**
     * @brief Construct the graph with a specified number of vertices.
     * @param n number of vertices in the graph
     */
    Graph(const unsigned int n): numVerts(n)
    {
        v = new E[n];
        e = new bool*[n];
	w = new W*[n];
        for (int i = 0; i < n; ++i)
	{
	    e[i] = new bool[n]();
	    w[i] = new W[n]();
	}
    }
    /**
     * @brief Construct the graph with an existing list of vertices.
     * @param n number of vertices in the graph
     * @param verts list of vertices
     */
    Graph(const unsigned int n, E *verts): numVerts(n)
    {
	v = verts;
        e = new bool*[n];
	w = new W*[n];
        for (int i = 0; i < n; ++i)
	{
	    e[i] = new bool[n]();
	    w[i] = new W[n]();
	}
    }
    /**
     * @brief Destructor
     */
    ~Graph()
    {
        delete[] v;
        for (int i = 0; i < numVerts; ++i)
	{
	    delete[] e[i];
	    delete[] w[i];
	}
        delete[] e;
	delete[] w;
    }
    /**
     * @brief Get an arbitrary previous vertex.
     * @param vert index of vertex
     * @return returns pointer to an arbitrary previous vertex or NULL if there is none
     */
    E* prev(int vert)
    {
        for (int i = 0; i < numVerts; ++i)
            if (e[i][vert])
                return &v[i];
        return NULL;
    }
    /**
     * @brief Get an arbitrary next vertex.
     * @param vert index of vertex
     * @return returns pointer to an arbitrary next vertex or NULL if there is none
     */
    E* next(int vert)
    {
        for (int i = 0; i < numVerts; ++i)
            if (e[vert][i])
                return &v[i];
        return NULL;
    }
    /**
     * @brief Add an edge to the graph.
     * @param v1 index of first vertex of edge
     * @param v2 index of second vertex of edge
     */
    void setEdge(const unsigned int v1, const unsigned int v2)
    {
        assert(v1 < numVerts && v2 < numVerts);
        e[v1][v2] = true;
    }
    /**
     * @brief Remove an edge from the graph.
     * @param v1 index of first vertex of edge
     * @param v2 index of second vertex of edge
     */
    void removeEdge(const unsigned int v1, const unsigned int v2)
    {
        assert (v1 < numVerts && v2 < numVerts);
        e[v1][v2] = false;
    }
    /**
     * @brief Get the max number of vertices in the graph.
     * @return the number of vertices
     */
    unsigned int size() const
    { return numVerts; }
};
