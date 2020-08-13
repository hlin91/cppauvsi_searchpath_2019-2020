// Simple weighted directed graph
#include <cstdlib>

template <typename E, typename W>
struct Graph 
{
    const unsigned int numVerts;
    E *v; //List of vertices
    bool **e; // Adjacency matrix representing the edges
    W **w; // Weight matrix
    
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
    E* prev(int vert) // Returns pointer to an arbitrary previous vertex or NULL if there is none
    {
        for (int i = 0; i < numVerts; ++i)
            if (e[i][vert])
                return &v[i];
        return NULL;
    }
    E* next(int vert) // Returns pointer to an arbitrary next vertex or NULL if there is none
    {
        for (int i = 0; i < numVerts; ++i)
            if (e[vert][i])
                return &v[i];
        return NULL;
    }
    void setEdge(const unsigned int v1, const unsigned int v2)
    {
        assert(v1 < numVerts && v2 < numVerts);
        e[v1][v2] = true;
    }
    void removeEdge(const unsigned int v1, const unsigned int v2)
    {
        assert (v1 < numVerts && v2 < numVerts);
        e[v1][v2] = false;
    }
    unsigned int size() const
    { return numVerts; }
};
