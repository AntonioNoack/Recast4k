/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j Copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
package org.recast4j.recast;

import java.util.Arrays;

import static org.recast4j.recast.RecastConstants.*;

public class RecastMesh {

    private static final int MAX_MESH_VERTICES_POLY = 0xffff;
    static int VERTEX_BUCKET_COUNT = (1 << 12);

    private static class Edge {
        int[] vert = new int[2];
        int[] polyEdge = new int[2];
        int[] poly = new int[2];

    }

    private static void buildMeshAdjacency(int[] polys, int npolys, int nvertices, int verticesPerPoly) {
        // Based on code by Eric Lengyel from:
        // http://www.terathon.com/code/edges.php

        int maxEdgeCount = npolys * verticesPerPoly;
        int[] firstEdge = new int[nvertices + maxEdgeCount];
        int edgeCount = 0;

        Edge[] edges = new Edge[maxEdgeCount];

        for (int i = 0; i < nvertices; i++)
            firstEdge[i] = RC_MESH_NULL_IDX;

        for (int i = 0; i < npolys; ++i) {
            int t = i * verticesPerPoly * 2;
            for (int j = 0; j < verticesPerPoly; ++j) {
                if (polys[t + j] == RC_MESH_NULL_IDX)
                    break;
                int v0 = polys[t + j];
                int v1 = (j + 1 >= verticesPerPoly || polys[t + j + 1] == RC_MESH_NULL_IDX) ? polys[t]
                        : polys[t + j + 1];
                if (v0 < v1) {
                    Edge edge = new Edge();
                    edges[edgeCount] = edge;
                    edge.vert[0] = v0;
                    edge.vert[1] = v1;
                    edge.poly[0] = i;
                    edge.polyEdge[0] = j;
                    edge.poly[1] = i;
                    edge.polyEdge[1] = 0;
                    // Insert edge
                    firstEdge[nvertices + edgeCount] = firstEdge[v0];
                    firstEdge[v0] = edgeCount;
                    edgeCount++;
                }
            }
        }

        for (int i = 0; i < npolys; ++i) {
            int t = i * verticesPerPoly * 2;
            for (int j = 0; j < verticesPerPoly; ++j) {
                if (polys[t + j] == RC_MESH_NULL_IDX)
                    break;
                int v0 = polys[t + j];
                int v1 = (j + 1 >= verticesPerPoly || polys[t + j + 1] == RC_MESH_NULL_IDX) ? polys[t]
                        : polys[t + j + 1];
                if (v0 > v1) {
                    for (int e = firstEdge[v1]; e != RC_MESH_NULL_IDX; e = firstEdge[nvertices + e]) {
                        Edge edge = edges[e];
                        if (edge.vert[1] == v0 && edge.poly[0] == edge.poly[1]) {
                            edge.poly[1] = i;
                            edge.polyEdge[1] = j;
                            break;
                        }
                    }
                }
            }
        }

        // Store adjacency
        for (int i = 0; i < edgeCount; ++i) {
            Edge e = edges[i];
            if (e.poly[0] != e.poly[1]) {
                int p0 = e.poly[0] * verticesPerPoly * 2;
                int p1 = e.poly[1] * verticesPerPoly * 2;
                polys[p0 + verticesPerPoly + e.polyEdge[0]] = e.poly[1];
                polys[p1 + verticesPerPoly + e.polyEdge[1]] = e.poly[0];
            }
        }

    }

    private static int computeVertexHash(int x, int y, int z) {
        int h1 = 0x8da6b343; // Large multiplicative constants;
        int h2 = 0xd8163841; // here arbitrarily chosen primes
        int h3 = 0xcb1ab31f;
        int n = h1 * x + h2 * y + h3 * z;
        return n & (VERTEX_BUCKET_COUNT - 1);
    }

    private static int[] addVertex(int x, int y, int z, int[] vertices, int[] firstVert, int[] nextVert, int nv) {
        int bucket = computeVertexHash(x, 0, z);// todo why is y skipped?
        int i = firstVert[bucket];

        while (i != -1) {
            int v = i * 3;
            if (vertices[v] == x && (Math.abs(vertices[v + 1] - y) <= 2) && vertices[v + 2] == z)
                return new int[]{i, nv};
            i = nextVert[i]; // next
        }

        // Could not find, create new.
        i = nv;
        nv++;
        int v = i * 3;
        vertices[v] = x;
        vertices[v + 1] = y;
        vertices[v + 2] = z;
        nextVert[i] = firstVert[bucket];
        firstVert[bucket] = i;

        return new int[]{i, nv};
    }

    static int prev(int i, int n) {
        return i - 1 >= 0 ? i - 1 : n - 1;
    }

    static int next(int i, int n) {
        return i + 1 < n ? i + 1 : 0;
    }

    private static int area2(int[] vertices, int a, int b, int c) {
        return (vertices[b] - vertices[a]) * (vertices[c + 2] - vertices[a + 2])
                - (vertices[c] - vertices[a]) * (vertices[b + 2] - vertices[a + 2]);
    }

    // Returns true iff c is strictly to the left of the directed
    // line through a to b.
    static boolean left(int[] vertices, int a, int b, int c) {
        return area2(vertices, a, b, c) < 0;
    }

    static boolean leftOn(int[] vertices, int a, int b, int c) {
        return area2(vertices, a, b, c) <= 0;
    }

    private static boolean collinear(int[] vertices, int a, int b, int c) {
        return area2(vertices, a, b, c) == 0;
    }

    // Returns true iff ab properly intersects cd: they share
    // a point interior to both segments. The properness of the
    // intersection is ensured by using strict leftness.
    private static boolean intersectProp(int[] vertices, int a, int b, int c, int d) {
        // Eliminate improper cases.
        if (collinear(vertices, a, b, c) || collinear(vertices, a, b, d) || collinear(vertices, c, d, a)
                || collinear(vertices, c, d, b))
            return false;

        return (left(vertices, a, b, c) ^ left(vertices, a, b, d)) && (left(vertices, c, d, a) ^ left(vertices, c, d, b));
    }

    // Returns T iff (a,b,c) are collinear and point c lies
    // on the closed segement ab.
    private static boolean between(int[] vertices, int a, int b, int c) {
        if (!collinear(vertices, a, b, c))
            return false;
        // If ab not vertical, check betweenness on x; else on y.
        if (vertices[a] != vertices[b])
            return ((vertices[a] <= vertices[c]) && (vertices[c] <= vertices[b]))
                    || ((vertices[a] >= vertices[c]) && (vertices[c] >= vertices[b]));
        else
            return ((vertices[a + 2] <= vertices[c + 2]) && (vertices[c + 2] <= vertices[b + 2]))
                    || ((vertices[a + 2] >= vertices[c + 2]) && (vertices[c + 2] >= vertices[b + 2]));
    }

    // Returns true iff segments ab and cd intersect, properly or improperly.
    static boolean intersect(int[] vertices, int a, int b, int c, int d) {
        if (intersectProp(vertices, a, b, c, d))
            return true;
        else return between(vertices, a, b, c) || between(vertices, a, b, d) || between(vertices, c, d, a)
                || between(vertices, c, d, b);
    }

    static boolean vequal(int[] vertices, int a, int b) {
        return vertices[a] == vertices[b] && vertices[a + 2] == vertices[b + 2];
    }

    // Returns T iff (v_i, v_j) is a proper internal *or* external
    // diagonal of P, *ignoring edges incident to v_i and v_j*.
    private static boolean diagonalie(int i, int j, int n, int[] vertices, int[] indices) {
        int d0 = (indices[i] & 0x0fffffff) * 4;
        int d1 = (indices[j] & 0x0fffffff) * 4;

        // For each edge (k,k+1) of P
        for (int k = 0; k < n; k++) {
            int k1 = next(k, n);
            // Skip edges incident to i or j
            if (!((k == i) || (k1 == i) || (k == j) || (k1 == j))) {
                int p0 = (indices[k] & 0x0fffffff) * 4;
                int p1 = (indices[k1] & 0x0fffffff) * 4;

                if (vequal(vertices, d0, p0) || vequal(vertices, d1, p0) || vequal(vertices, d0, p1) || vequal(vertices, d1, p1))
                    continue;

                if (intersect(vertices, d0, d1, p0, p1))
                    return false;
            }
        }
        return true;
    }

    // Returns true iff the diagonal (i,j) is strictly internal to the
    // polygon P in the neighborhood of the i endpoint.
    private static boolean inCone(int i, int j, int n, int[] vertices, int[] indices) {
        int pi = (indices[i] & 0x0fffffff) * 4;
        int pj = (indices[j] & 0x0fffffff) * 4;
        int pi1 = (indices[next(i, n)] & 0x0fffffff) * 4;
        int pin1 = (indices[prev(i, n)] & 0x0fffffff) * 4;
        // If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
        if (leftOn(vertices, pin1, pi, pi1)) {
            return left(vertices, pi, pj, pin1) && left(vertices, pj, pi, pi1);
        }
        // Assume (i-1,i,i+1) not collinear.
        // else P[i] is reflex.
        return !(leftOn(vertices, pi, pj, pi1) && leftOn(vertices, pj, pi, pin1));
    }

    // Returns T iff (v_i, v_j) is a proper internal
    // diagonal of P.
    private static boolean diagonal(int i, int j, int n, int[] vertices, int[] indices) {
        return inCone(i, j, n, vertices, indices) && diagonalie(i, j, n, vertices, indices);
    }

    private static boolean diagonalieLoose(int i, int j, int n, int[] vertices, int[] indices) {
        int d0 = (indices[i] & 0x0fffffff) * 4;
        int d1 = (indices[j] & 0x0fffffff) * 4;

        // For each edge (k,k+1) of P
        for (int k = 0; k < n; k++) {
            int k1 = next(k, n);
            // Skip edges incident to i or j
            if (!((k == i) || (k1 == i) || (k == j) || (k1 == j))) {
                int p0 = (indices[k] & 0x0fffffff) * 4;
                int p1 = (indices[k1] & 0x0fffffff) * 4;

                if (vequal(vertices, d0, p0) || vequal(vertices, d1, p0) || vequal(vertices, d0, p1) || vequal(vertices, d1, p1))
                    continue;

                if (intersectProp(vertices, d0, d1, p0, p1))
                    return false;
            }
        }
        return true;
    }

    private static boolean inConeLoose(int i, int j, int n, int[] vertices, int[] indices) {
        int pi = (indices[i] & 0x0fffffff) * 4;
        int pj = (indices[j] & 0x0fffffff) * 4;
        int pi1 = (indices[next(i, n)] & 0x0fffffff) * 4;
        int pin1 = (indices[prev(i, n)] & 0x0fffffff) * 4;

        // If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
        if (leftOn(vertices, pin1, pi, pi1))
            return leftOn(vertices, pi, pj, pin1) && leftOn(vertices, pj, pi, pi1);
        // Assume (i-1,i,i+1) not collinear.
        // else P[i] is reflex.
        return !(leftOn(vertices, pi, pj, pi1) && leftOn(vertices, pj, pi, pin1));
    }

    private static boolean diagonalLoose(int i, int j, int n, int[] vertices, int[] indices) {
        return inConeLoose(i, j, n, vertices, indices) && diagonalieLoose(i, j, n, vertices, indices);
    }

    private static int triangulate(int n, int[] vertices, int[] indices, int[] tris) {
        int ntris = 0;

        // The last bit of the index is used to indicate if the vertex can be removed.
        for (int i = 0; i < n; i++) {
            int i1 = next(i, n);
            int i2 = next(i1, n);
            if (diagonal(i, i2, n, vertices, indices)) {
                indices[i1] |= 0x80000000;
            }
        }

        while (n > 3) {
            int minLen = -1;
            int mini = -1;
            for (int i = 0; i < n; i++) {
                int i1 = next(i, n);
                if ((indices[i1] & 0x80000000) != 0) {
                    int p0 = (indices[i] & 0x0fffffff) * 4;
                    int p2 = (indices[next(i1, n)] & 0x0fffffff) * 4;

                    int dx = vertices[p2] - vertices[p0];
                    int dy = vertices[p2 + 2] - vertices[p0 + 2];
                    int len = dx * dx + dy * dy;

                    if (minLen < 0 || len < minLen) {
                        minLen = len;
                        mini = i;
                    }
                }
            }

            if (mini == -1) {
                // We might get here because the contour has overlapping segments, like this:
                //
                // A o-o=====o---o B
                // / |C D| \
                // o o o o
                // : : : :
                // We'll try to recover by loosing up the inCone test a bit so that a diagonal
                // like A-B or C-D can be found and we can continue.
                for (int i = 0; i < n; i++) {
                    int i1 = next(i, n);
                    int i2 = next(i1, n);
                    if (diagonalLoose(i, i2, n, vertices, indices)) {
                        int p0 = (indices[i] & 0x0fffffff) * 4;
                        int p2 = (indices[next(i2, n)] & 0x0fffffff) * 4;
                        int dx = vertices[p2] - vertices[p0];
                        int dy = vertices[p2 + 2] - vertices[p0 + 2];
                        int len = dx * dx + dy * dy;

                        if (minLen < 0 || len < minLen) {
                            minLen = len;
                            mini = i;
                        }
                    }
                }
                if (mini == -1) {
                    // The contour is messed up. This sometimes happens
                    // if the contour simplification is too aggressive.
                    return -ntris;
                }
            }

            int i = mini;
            int i1 = next(i, n);
            int i2 = next(i1, n);

            tris[ntris * 3] = indices[i] & 0x0fffffff;
            tris[ntris * 3 + 1] = indices[i1] & 0x0fffffff;
            tris[ntris * 3 + 2] = indices[i2] & 0x0fffffff;
            ntris++;

            // Removes P[i1] by copying P[i+1]...P[n-1] left one index.
            n--;
            if (n - i1 >= 0)
                System.arraycopy(indices, i1 + 1, indices, i1, n - i1);

            if (i1 >= n)
                i1 = 0;
            i = prev(i1, n);
            // Update diagonal flags.
            if (diagonal(prev(i, n), i1, n, vertices, indices))
                indices[i] |= 0x80000000;
            else
                indices[i] &= 0x0fffffff;

            if (diagonal(i, next(i1, n), n, vertices, indices))
                indices[i1] |= 0x80000000;
            else
                indices[i1] &= 0x0fffffff;
        }

        // Append the remaining triangle.
        tris[ntris * 3] = indices[0] & 0x0fffffff;
        tris[ntris * 3 + 1] = indices[1] & 0x0fffffff;
        tris[ntris * 3 + 2] = indices[2] & 0x0fffffff;
        ntris++;

        return ntris;
    }

    private static int countPolyVertices(int[] p, int j, int nvp) {
        for (int i = 0; i < nvp; ++i)
            if (p[i + j] == RC_MESH_NULL_IDX)
                return i;
        return nvp;
    }

    private static boolean uleft(int[] vertices, int a, int b, int c) {
        return (vertices[b] - vertices[a]) * (vertices[c + 2] - vertices[a + 2])
                - (vertices[c] - vertices[a]) * (vertices[b + 2] - vertices[a + 2]) < 0;
    }

    private static int[] getPolyMergeValue(int[] polys, int pa, int pb, int[] vertices, int nvp) {
        int ea = -1;
        int eb = -1;
        int na = countPolyVertices(polys, pa, nvp);
        int nb = countPolyVertices(polys, pb, nvp);

        // If the merged polygon would be too big, do not merge.
        if (na + nb - 2 > nvp)
            return new int[]{-1, ea, eb};

        // Check if the polygons share an edge.

        for (int i = 0; i < na; ++i) {
            int va0 = polys[pa + i];
            int va1 = polys[pa + (i + 1) % na];
            if (va0 > va1) {
                int temp = va0;
                va0 = va1;
                va1 = temp;
            }
            for (int j = 0; j < nb; ++j) {
                int vb0 = polys[pb + j];
                int vb1 = polys[pb + (j + 1) % nb];
                if (vb0 > vb1) {
                    int temp = vb0;
                    vb0 = vb1;
                    vb1 = temp;
                }
                if (va0 == vb0 && va1 == vb1) {
                    ea = i;
                    eb = j;
                    break;
                }
            }
        }

        // No common edge, cannot merge.
        if (ea == -1 || eb == -1)
            return new int[]{-1, ea, eb};

        // Check to see if the merged polygon would be convex.
        int va, vb, vc;

        va = polys[pa + (ea + na - 1) % na];
        vb = polys[pa + ea];
        vc = polys[pb + (eb + 2) % nb];
        if (!uleft(vertices, va * 3, vb * 3, vc * 3))
            return new int[]{-1, ea, eb};

        va = polys[pb + (eb + nb - 1) % nb];
        vb = polys[pb + eb];
        vc = polys[pa + (ea + 2) % na];
        if (!uleft(vertices, va * 3, vb * 3, vc * 3))
            return new int[]{-1, ea, eb};

        va = polys[pa + ea];
        vb = polys[pa + (ea + 1) % na];

        int dx = vertices[va * 3] - vertices[vb * 3];
        int dy = vertices[va * 3 + 2] - vertices[vb * 3 + 2];

        return new int[]{dx * dx + dy * dy, ea, eb};
    }

    private static void mergePolyVertices(int[] polys, int pa, int pb, int ea, int eb, int tmp, int nvp) {
        int na = countPolyVertices(polys, pa, nvp);
        int nb = countPolyVertices(polys, pb, nvp);

        // Merge polygons.
        Arrays.fill(polys, tmp, tmp + nvp, RC_MESH_NULL_IDX);
        int n = 0;
        // Add pa
        for (int i = 0; i < na - 1; ++i) {
            polys[tmp + n] = polys[pa + (ea + 1 + i) % na];
            n++;
        }
        // Add pb
        for (int i = 0; i < nb - 1; ++i) {
            polys[tmp + n] = polys[pb + (eb + 1 + i) % nb];
            n++;
        }
        System.arraycopy(polys, tmp, polys, pa, nvp);
    }

    private static void pushFront(int v, int[] arr, int an) {
        if (an - 1 >= 0) System.arraycopy(arr, 0, arr, 1, an - 1);
        arr[0] = v;
    }

    private static void pushBack(int v, int[] arr, int an) {
        arr[an] = v;
    }

    private static boolean canRemoveVertex(PolyMesh mesh, int rem) {
        int nvp = mesh.maxVerticesPerPolygon;

        // Count number of polygons to remove.
        int numTouchedVertices = 0;
        int numRemainingEdges = 0;
        for (int i = 0; i < mesh.numPolygons; ++i) {
            int p = i * nvp * 2;
            int nv = countPolyVertices(mesh.polygons, p, nvp);
            int numRemoved = 0;
            int numVertices = 0;
            for (int j = 0; j < nv; ++j) {
                if (mesh.polygons[p + j] == rem) {
                    numTouchedVertices++;
                    numRemoved++;
                }
                numVertices++;
            }
            if (numRemoved != 0) {
                numRemainingEdges += numVertices - (numRemoved + 1);
            }
        }
        // There would be too few edges remaining to create a polygon.
        // This can happen for example when a tip of a triangle is marked
        // as deletion, but there are no other polys that share the vertex.
        // In this case, the vertex should not be removed.
        if (numRemainingEdges <= 2)
            return false;

        // Find edges which share the removed vertex.
        int maxEdges = numTouchedVertices * 2;
        int nedges = 0;
        int[] edges = new int[maxEdges * 3];

        for (int i = 0; i < mesh.numPolygons; ++i) {
            int p = i * nvp * 2;
            int nv = countPolyVertices(mesh.polygons, p, nvp);

            // Collect edges which touches the removed vertex.
            for (int j = 0, k = nv - 1; j < nv; k = j++) {
                if (mesh.polygons[p + j] == rem || mesh.polygons[p + k] == rem) {
                    // Arrange edge so that a=rem.
                    int a = mesh.polygons[p + j], b = mesh.polygons[p + k];
                    if (b == rem) {
                        int temp = a;
                        a = b;
                        b = temp;
                    }
                    // Check if the edge exists
                    boolean exists = false;
                    for (int m = 0; m < nedges; ++m) {
                        int e = m * 3;
                        if (edges[e + 1] == b) {
                            // Exists, increment vertex share count.
                            edges[e + 2]++;
                            exists = true;
                        }
                    }
                    // Add new edge.
                    if (!exists) {
                        int e = nedges * 3;
                        edges[e] = a;
                        edges[e + 1] = b;
                        edges[e + 2] = 1;
                        nedges++;
                    }
                }
            }
        }

        // There should be no more than 2 open edges.
        // This catches the case that two non-adjacent polygons
        // share the removed vertex. In that case, do not remove the vertex.
        int numOpenEdges = 0;
        for (int i = 0; i < nedges; ++i) {
            if (edges[i * 3 + 2] < 2)
                numOpenEdges++;
        }
        return numOpenEdges <= 2;
    }

    private static void removeVertex(Telemetry ctx, PolyMesh mesh, int rem, int maxTris) {
        int nvp = mesh.maxVerticesPerPolygon;

        // Count number of polygons to remove.
        int numRemovedVertices = 0;
        for (int i = 0; i < mesh.numPolygons; ++i) {
            int p = i * nvp * 2;
            int nv = countPolyVertices(mesh.polygons, p, nvp);
            for (int j = 0; j < nv; ++j) {
                if (mesh.polygons[p + j] == rem)
                    numRemovedVertices++;
            }
        }

        int nedges = 0;
        int[] edges = new int[numRemovedVertices * nvp * 4];

        int nhole = 0;
        int[] hole = new int[numRemovedVertices * nvp];

        int nhreg = 0;
        int[] hreg = new int[numRemovedVertices * nvp];

        int nharea = 0;
        int[] harea = new int[numRemovedVertices * nvp];

        for (int i = 0; i < mesh.numPolygons; ++i) {
            int p = i * nvp * 2;
            int nv = countPolyVertices(mesh.polygons, p, nvp);
            boolean hasRem = false;
            for (int j = 0; j < nv; ++j)
                if (mesh.polygons[p + j] == rem) {
                    hasRem = true;
                    break;
                }
            if (hasRem) {
                // Collect edges which does not touch the removed vertex.
                for (int j = 0, k = nv - 1; j < nv; k = j++) {
                    if (mesh.polygons[p + j] != rem && mesh.polygons[p + k] != rem) {
                        int e = nedges * 4;
                        edges[e] = mesh.polygons[p + k];
                        edges[e + 1] = mesh.polygons[p + j];
                        edges[e + 2] = mesh.regionIds[i];
                        edges[e + 3] = mesh.areaIds[i];
                        nedges++;
                    }
                }
                // Remove the polygon.
                int p2 = (mesh.numPolygons - 1) * nvp * 2;
                if (p != p2) {
                    System.arraycopy(mesh.polygons, p2, mesh.polygons, p, nvp);
                }
                Arrays.fill(mesh.polygons, p + nvp, p + nvp + nvp, RC_MESH_NULL_IDX);
                mesh.regionIds[i] = mesh.regionIds[mesh.numPolygons - 1];
                mesh.areaIds[i] = mesh.areaIds[mesh.numPolygons - 1];
                mesh.numPolygons--;
                --i;
            }
        }

        // Remove vertex.
        for (int i = rem; i < mesh.numVertices - 1; ++i) {
            mesh.vertices[i * 3] = mesh.vertices[(i + 1) * 3];
            mesh.vertices[i * 3 + 1] = mesh.vertices[(i + 1) * 3 + 1];
            mesh.vertices[i * 3 + 2] = mesh.vertices[(i + 1) * 3 + 2];
        }
        mesh.numVertices--;

        // Adjust indices to match the removed vertex layout.
        for (int i = 0; i < mesh.numPolygons; ++i) {
            int p = i * nvp * 2;
            int nv = countPolyVertices(mesh.polygons, p, nvp);
            for (int j = 0; j < nv; ++j)
                if (mesh.polygons[p + j] > rem)
                    mesh.polygons[p + j]--;
        }
        for (int i = 0; i < nedges; ++i) {
            if (edges[i * 4] > rem)
                edges[i * 4]--;
            if (edges[i * 4 + 1] > rem)
                edges[i * 4 + 1]--;
        }

        if (nedges == 0)
            return;

        // Start with one vertex, keep appending connected
        // segments to the start and end of the hole.
        hole[nhole++] = edges[0];
        hreg[nhreg++] = edges[2];
        harea[nharea++] = edges[3];

        while (nedges != 0) {
            boolean match = false;

            for (int i = 0; i < nedges; ++i) {
                int ea = edges[i * 4];
                int eb = edges[i * 4 + 1];
                int r = edges[i * 4 + 2];
                int a = edges[i * 4 + 3];
                boolean add = false;
                if (hole[0] == eb) {
                    // The segment matches the beginning of the hole boundary.
                    pushFront(ea, hole, ++nhole);
                    pushFront(r, hreg, ++nhreg);
                    pushFront(a, harea, ++nharea);
                    add = true;
                } else if (hole[nhole - 1] == ea) {
                    // The segment matches the end of the hole boundary.
                    pushBack(eb, hole, nhole++);
                    pushBack(r, hreg, nhreg++);
                    pushBack(a, harea, nharea++);
                    add = true;
                }
                if (add) {
                    // The edge segment was added, remove it.
                    edges[i * 4] = edges[(nedges - 1) * 4];
                    edges[i * 4 + 1] = edges[(nedges - 1) * 4 + 1];
                    edges[i * 4 + 2] = edges[(nedges - 1) * 4 + 2];
                    edges[i * 4 + 3] = edges[(nedges - 1) * 4 + 3];
                    --nedges;
                    match = true;
                    --i;
                }
            }

            if (!match)
                break;
        }

        int[] tris = new int[nhole * 3];
        int[] tvertices = new int[nhole * 4];
        int[] thole = new int[nhole];

        // Generate temp vertex array for triangulation.
        for (int i = 0; i < nhole; ++i) {
            int pi = hole[i];
            tvertices[i * 4] = mesh.vertices[pi * 3];
            tvertices[i * 4 + 1] = mesh.vertices[pi * 3 + 1];
            tvertices[i * 4 + 2] = mesh.vertices[pi * 3 + 2];
            tvertices[i * 4 + 3] = 0;
            thole[i] = i;
        }

        // Triangulate the hole.
        int ntris = triangulate(nhole, tvertices, thole, tris);
        if (ntris < 0) {
            ntris = -ntris;
            ctx.warn("removeVertex: triangulate() returned bad results.");
        }

        // Merge the hole triangles back to polygons.
        int[] polys = new int[(ntris + 1) * nvp];
        int[] pregs = new int[ntris];
        int[] pareas = new int[ntris];

        int tmpPoly = ntris * nvp;

        // Build initial polygons.
        int npolys = 0;
        Arrays.fill(polys, 0, ntris * nvp, RC_MESH_NULL_IDX);
        for (int j = 0; j < ntris; ++j) {
            int t = j * 3;
            if (tris[t] != tris[t + 1] && tris[t] != tris[t + 2] && tris[t + 1] != tris[t + 2]) {
                polys[npolys * nvp] = hole[tris[t]];
                polys[npolys * nvp + 1] = hole[tris[t + 1]];
                polys[npolys * nvp + 2] = hole[tris[t + 2]];

                // If this polygon covers multiple region types then
                // mark it as such
                if (hreg[tris[t]] != hreg[tris[t + 1]] || hreg[tris[t + 1]] != hreg[tris[t + 2]])
                    pregs[npolys] = RC_MULTIPLE_REGS;
                else
                    pregs[npolys] = hreg[tris[t]];

                pareas[npolys] = harea[tris[t]];
                npolys++;
            }
        }
        if (npolys == 0)
            return;

        // Merge polygons.
        if (nvp > 3) {
            for (; ; ) {
                // Find best polygons to merge.
                int bestMergeVal = 0;
                int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;

                for (int j = 0; j < npolys - 1; ++j) {
                    int pj = j * nvp;
                    for (int k = j + 1; k < npolys; ++k) {
                        int pk = k * nvp;
                        int[] veaeb = getPolyMergeValue(polys, pj, pk, mesh.vertices, nvp);
                        int v = veaeb[0];
                        int ea = veaeb[1];
                        int eb = veaeb[2];
                        if (v > bestMergeVal) {
                            bestMergeVal = v;
                            bestPa = j;
                            bestPb = k;
                            bestEa = ea;
                            bestEb = eb;
                        }
                    }
                }

                if (bestMergeVal > 0) {
                    // Found best, merge.
                    int pa = bestPa * nvp;
                    int pb = bestPb * nvp;
                    mergePolyVertices(polys, pa, pb, bestEa, bestEb, tmpPoly, nvp);
                    if (pregs[bestPa] != pregs[bestPb])
                        pregs[bestPa] = RC_MULTIPLE_REGS;
                    int last = (npolys - 1) * nvp;
                    if (pb != last) {
                        System.arraycopy(polys, last, polys, pb, nvp);
                    }
                    pregs[bestPb] = pregs[npolys - 1];
                    pareas[bestPb] = pareas[npolys - 1];
                    npolys--;
                } else {
                    // Could not merge any polygons, stop.
                    break;
                }
            }
        }

        // Store polygons.
        for (int i = 0; i < npolys; ++i) {
            if (mesh.numPolygons >= maxTris)
                break;
            int p = mesh.numPolygons * nvp * 2;
            Arrays.fill(mesh.polygons, p, p + nvp * 2, RC_MESH_NULL_IDX);
            if (nvp >= 0) System.arraycopy(polys, i * nvp, mesh.polygons, p, nvp);
            mesh.regionIds[mesh.numPolygons] = pregs[i];
            mesh.areaIds[mesh.numPolygons] = pareas[i];
            mesh.numPolygons++;
            if (mesh.numPolygons > maxTris) {
                throw new RuntimeException("removeVertex: Too many polygons " + mesh.numPolygons + " (max:" + maxTris + ".");
            }
        }

    }

    /// @par
    ///
    /// @note If the mesh data is to be used to construct a Detour navigation mesh, then the upper
    /// limit must be retricted to <= #DT_VERTICES_PER_POLYGON.
    ///
    /// @see rcAllocPolyMesh, rcContourSet, rcPolyMesh, rcConfig
    public static PolyMesh buildPolyMesh(Telemetry ctx, ContourSet cset, int nvp) {
        if (ctx != null) ctx.startTimer("POLYMESH");
        PolyMesh mesh = new PolyMesh();
        mesh.bmin.set(cset.bmin);
        mesh.bmax.set(cset.bmax);
        mesh.cellSize = cset.cellSize;
        mesh.cellHeight = cset.cellHeight;
        mesh.borderSize = cset.borderSize;
        mesh.maxEdgeError = cset.maxError;

        int maxVertices = 0;
        int maxTris = 0;
        int maxVerticesPerCont = 0;
        for (int i = 0; i < cset.contours.size(); ++i) {
            // Skip null contours.
            if (cset.contours.get(i).numVertices < 3)
                continue;
            maxVertices += cset.contours.get(i).numVertices;
            maxTris += cset.contours.get(i).numVertices - 2;
            maxVerticesPerCont = Math.max(maxVerticesPerCont, cset.contours.get(i).numVertices);
        }
        if (maxVertices >= 0xfffe) {
            throw new RuntimeException("rcBuildPolyMesh: Too many vertices " + maxVertices);
        }
        int[] vflags = new int[maxVertices];

        mesh.vertices = new int[maxVertices * 3];
        mesh.polygons = new int[maxTris * nvp * 2];
        Arrays.fill(mesh.polygons, RC_MESH_NULL_IDX);
        mesh.regionIds = new int[maxTris];
        mesh.areaIds = new int[maxTris];

        mesh.numVertices = 0;
        mesh.numPolygons = 0;
        mesh.maxVerticesPerPolygon = nvp;
        mesh.numAllocatedPolygons = maxTris;

        int[] nextVert = new int[maxVertices];

        int[] firstVert = new int[VERTEX_BUCKET_COUNT];
        for (int i = 0; i < VERTEX_BUCKET_COUNT; ++i)
            firstVert[i] = -1;

        int[] indices = new int[maxVerticesPerCont];
        int[] tris = new int[maxVerticesPerCont * 3];
        int[] polys = new int[(maxVerticesPerCont + 1) * nvp];

        int tmpPoly = maxVerticesPerCont * nvp;

        for (int i = 0; i < cset.contours.size(); ++i) {
            Contour cont = cset.contours.get(i);

            // Skip null contours.
            if (cont.numVertices < 3)
                continue;

            // Triangulate contour
            for (int j = 0; j < cont.numVertices; ++j)
                indices[j] = j;
            int ntris = triangulate(cont.numVertices, cont.vertices, indices, tris);
            if (ntris <= 0) {
                // Bad triangulation, should not happen.
                if (ctx != null) ctx.warn("buildPolyMesh: Bad triangulation Contour " + i + ".");
                ntris = -ntris;
            }

            // Add and merge vertices.
            for (int j = 0; j < cont.numVertices; ++j) {
                int v = j * 4;
                int[] inv = addVertex(cont.vertices[v], cont.vertices[v + 1], cont.vertices[v + 2], mesh.vertices, firstVert,
                        nextVert, mesh.numVertices);
                indices[j] = inv[0];
                mesh.numVertices = inv[1];
                if ((cont.vertices[v + 3] & RC_BORDER_VERTEX) != 0) {
                    // This vertex should be removed.
                    vflags[indices[j]] = 1;
                }
            }

            // Build initial polygons.
            int npolys = 0;
            Arrays.fill(polys, RC_MESH_NULL_IDX);
            for (int j = 0; j < ntris; ++j) {
                int t = j * 3;
                if (tris[t] != tris[t + 1] && tris[t] != tris[t + 2] && tris[t + 1] != tris[t + 2]) {
                    polys[npolys * nvp] = indices[tris[t]];
                    polys[npolys * nvp + 1] = indices[tris[t + 1]];
                    polys[npolys * nvp + 2] = indices[tris[t + 2]];
                    npolys++;
                }
            }
            if (npolys == 0)
                continue;

            // Merge polygons.
            if (nvp > 3) {
                for (; ; ) {
                    // Find best polygons to merge.
                    int bestMergeVal = 0;
                    int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;

                    for (int j = 0; j < npolys - 1; ++j) {
                        int pj = j * nvp;
                        for (int k = j + 1; k < npolys; ++k) {
                            int pk = k * nvp;
                            int[] veaeb = getPolyMergeValue(polys, pj, pk, mesh.vertices, nvp);
                            int v = veaeb[0];
                            int ea = veaeb[1];
                            int eb = veaeb[2];
                            if (v > bestMergeVal) {
                                bestMergeVal = v;
                                bestPa = j;
                                bestPb = k;
                                bestEa = ea;
                                bestEb = eb;
                            }
                        }
                    }

                    if (bestMergeVal > 0) {
                        // Found best, merge.
                        int pa = bestPa * nvp;
                        int pb = bestPb * nvp;
                        mergePolyVertices(polys, pa, pb, bestEa, bestEb, tmpPoly, nvp);
                        int lastPoly = (npolys - 1) * nvp;
                        if (pb != lastPoly) {
                            System.arraycopy(polys, lastPoly, polys, pb, nvp);
                        }
                        npolys--;
                    } else {
                        // Could not merge any polygons, stop.
                        break;
                    }
                }
            }

            // Store polygons.
            for (int j = 0; j < npolys; ++j) {
                int p = mesh.numPolygons * nvp * 2;
                int q = j * nvp;
                if (nvp >= 0) System.arraycopy(polys, q, mesh.polygons, p, nvp);
                mesh.regionIds[mesh.numPolygons] = cont.reg;
                mesh.areaIds[mesh.numPolygons] = cont.area;
                mesh.numPolygons++;
                if (mesh.numPolygons > maxTris) {
                    throw new RuntimeException(
                            "rcBuildPolyMesh: Too many polygons " + mesh.numPolygons + " (max:" + maxTris + ").");
                }
            }
        }

        // Remove edge vertices.
        for (int i = 0; i < mesh.numVertices; ++i) {
            if (vflags[i] != 0) {
                if (!canRemoveVertex(mesh, i))
                    continue;
                removeVertex(ctx, mesh, i, maxTris);
                // Remove vertex
                // Note: mesh.nvertices is already decremented inside removeVertex()!
                // Fixup vertex flags
                if (mesh.numVertices - i >= 0) System.arraycopy(vflags, i + 1, vflags, i, mesh.numVertices - i);
                --i;
            }
        }

        // Calculate adjacency.
        buildMeshAdjacency(mesh.polygons, mesh.numPolygons, mesh.numVertices, nvp);

        // Find portal edges
        if (mesh.borderSize > 0) {
            int w = cset.width;
            int h = cset.height;
            for (int i = 0; i < mesh.numPolygons; ++i) {
                int p = i * 2 * nvp;
                for (int j = 0; j < nvp; ++j) {
                    if (mesh.polygons[p + j] == RC_MESH_NULL_IDX)
                        break;
                    // Skip connected edges.
                    if (mesh.polygons[p + nvp + j] != RC_MESH_NULL_IDX)
                        continue;
                    int nj = j + 1;
                    if (nj >= nvp || mesh.polygons[p + nj] == RC_MESH_NULL_IDX)
                        nj = 0;
                    int va = mesh.polygons[p + j] * 3;
                    int vb = mesh.polygons[p + nj] * 3;

                    if (mesh.vertices[va] == 0 && mesh.vertices[vb] == 0)
                        mesh.polygons[p + nvp + j] = 0x8000;
                    else if (mesh.vertices[va + 2] == h && mesh.vertices[vb + 2] == h)
                        mesh.polygons[p + nvp + j] = 0x8001;
                    else if (mesh.vertices[va] == w && mesh.vertices[vb] == w)
                        mesh.polygons[p + nvp + j] = 0x8002;
                    else if (mesh.vertices[va + 2] == 0 && mesh.vertices[vb + 2] == 0)
                        mesh.polygons[p + nvp + j] = 0x8003;
                }
            }
        }

        // Just allocate the mesh flags array. The user is resposible to fill it.
        mesh.flags = new int[mesh.numPolygons];

        if (mesh.numVertices > MAX_MESH_VERTICES_POLY) {
            throw new RuntimeException("rcBuildPolyMesh: The resulting mesh has too many vertices " + mesh.numVertices
                    + " (max " + MAX_MESH_VERTICES_POLY + "). Data can be corrupted.");
        }
        if (mesh.numPolygons > MAX_MESH_VERTICES_POLY) {
            throw new RuntimeException("rcBuildPolyMesh: The resulting mesh has too many polygons " + mesh.numPolygons
                    + " (max " + MAX_MESH_VERTICES_POLY + "). Data can be corrupted.");
        }

        if (ctx != null) ctx.stopTimer("POLYMESH");
        return mesh;

    }

    /// @see rcAllocPolyMesh, rcPolyMesh
    @SuppressWarnings("unused")
    public static PolyMesh mergePolyMeshes(Telemetry ctx, PolyMesh[] meshes, int nmeshes) {

        if (nmeshes == 0 || meshes == null)
            return null;

        if (ctx != null) ctx.startTimer("MERGE_POLYMESH");
        PolyMesh mesh = new PolyMesh();
        mesh.maxVerticesPerPolygon = meshes[0].maxVerticesPerPolygon;
        mesh.cellSize = meshes[0].cellSize;
        mesh.cellHeight = meshes[0].cellHeight;
        mesh.bmin.set(meshes[0].bmin);
        mesh.bmax.set(meshes[0].bmax);

        int maxVertices = 0;
        int maxPolys = 0;
        int maxVerticesPerMesh = 0;
        for (int i = 0; i < nmeshes; ++i) {
            mesh.bmin.min(meshes[i].bmin);
            mesh.bmax.max(meshes[i].bmax);
            maxVerticesPerMesh = Math.max(maxVerticesPerMesh, meshes[i].numVertices);
            maxVertices += meshes[i].numVertices;
            maxPolys += meshes[i].numPolygons;
        }

        mesh.numVertices = 0;
        mesh.vertices = new int[maxVertices * 3];

        mesh.numPolygons = 0;
        mesh.polygons = new int[maxPolys * 2 * mesh.maxVerticesPerPolygon];
        Arrays.fill(mesh.polygons, 0, mesh.polygons.length, RC_MESH_NULL_IDX);
        mesh.regionIds = new int[maxPolys];
        mesh.areaIds = new int[maxPolys];
        mesh.flags = new int[maxPolys];

        int[] nextVert = new int[maxVertices];

        int[] firstVert = new int[VERTEX_BUCKET_COUNT];
        for (int i = 0; i < VERTEX_BUCKET_COUNT; ++i)
            firstVert[i] = -1;

        int[] vremap = new int[maxVerticesPerMesh];

        for (int i = 0; i < nmeshes; ++i) {
            PolyMesh pmesh = meshes[i];

            int ox = (int) Math.floor((pmesh.bmin.x - mesh.bmin.x) / mesh.cellSize + 0.5f);
            int oz = (int) Math.floor((pmesh.bmin.z - mesh.bmin.z) / mesh.cellSize + 0.5f);

            boolean isMinX = (ox == 0);
            boolean isMinZ = (oz == 0);
            boolean isMaxX = (Math.floor((mesh.bmax.x - pmesh.bmax.x) / mesh.cellSize + 0.5f)) == 0;
            boolean isMaxZ = (Math.floor((mesh.bmax.z - pmesh.bmax.z) / mesh.cellSize + 0.5f)) == 0;
            boolean isOnBorder = (isMinX || isMinZ || isMaxX || isMaxZ);

            for (int j = 0; j < pmesh.numVertices; ++j) {
                int v = j * 3;
                int[] inv = addVertex(pmesh.vertices[v] + ox, pmesh.vertices[v + 1], pmesh.vertices[v + 2] + oz, mesh.vertices,
                        firstVert, nextVert, mesh.numVertices);

                vremap[j] = inv[0];
                mesh.numVertices = inv[1];
            }

            for (int j = 0; j < pmesh.numPolygons; ++j) {
                int tgt = mesh.numPolygons * 2 * mesh.maxVerticesPerPolygon;
                int src = j * 2 * mesh.maxVerticesPerPolygon;
                mesh.regionIds[mesh.numPolygons] = pmesh.regionIds[j];
                mesh.areaIds[mesh.numPolygons] = pmesh.areaIds[j];
                mesh.flags[mesh.numPolygons] = pmesh.flags[j];
                mesh.numPolygons++;
                for (int k = 0; k < mesh.maxVerticesPerPolygon; ++k) {
                    if (pmesh.polygons[src + k] == RC_MESH_NULL_IDX)
                        break;
                    mesh.polygons[tgt + k] = vremap[pmesh.polygons[src + k]];
                }

                if (isOnBorder) {
                    for (int k = mesh.maxVerticesPerPolygon; k < mesh.maxVerticesPerPolygon * 2; ++k) {
                        if ((pmesh.polygons[src + k] & 0x8000) != 0 && pmesh.polygons[src + k] != 0xffff) {
                            int dir = pmesh.polygons[src + k] & 0xf;
                            switch (dir) {
                                case 0: // Portal x-
                                    if (isMinX)
                                        mesh.polygons[tgt + k] = pmesh.polygons[src + k];
                                    break;
                                case 1: // Portal z+
                                    if (isMaxZ)
                                        mesh.polygons[tgt + k] = pmesh.polygons[src + k];
                                    break;
                                case 2: // Portal x+
                                    if (isMaxX)
                                        mesh.polygons[tgt + k] = pmesh.polygons[src + k];
                                    break;
                                case 3: // Portal z-
                                    if (isMinZ)
                                        mesh.polygons[tgt + k] = pmesh.polygons[src + k];
                                    break;
                            }
                        }
                    }
                }
            }
        }

        // Calculate adjacency.
        buildMeshAdjacency(mesh.polygons, mesh.numPolygons, mesh.numVertices, mesh.maxVerticesPerPolygon);
        if (mesh.numVertices > MAX_MESH_VERTICES_POLY) {
            throw new RuntimeException("rcBuildPolyMesh: The resulting mesh has too many vertices " + mesh.numVertices
                    + " (max " + MAX_MESH_VERTICES_POLY + "). Data can be corrupted.");
        }
        if (mesh.numPolygons > MAX_MESH_VERTICES_POLY) {
            throw new RuntimeException("rcBuildPolyMesh: The resulting mesh has too many polygons " + mesh.numPolygons
                    + " (max " + MAX_MESH_VERTICES_POLY + "). Data can be corrupted.");
        }

        if (ctx != null) ctx.stopTimer("MERGE_POLYMESH");

        return mesh;
    }

    @SuppressWarnings("unused")
    public static PolyMesh copyPolyMesh(PolyMesh src) {
        PolyMesh dst = new PolyMesh();

        dst.numVertices = src.numVertices;
        dst.numPolygons = src.numPolygons;
        dst.numAllocatedPolygons = src.numPolygons;
        dst.maxVerticesPerPolygon = src.maxVerticesPerPolygon;
        dst.bmin.set(src.bmin);
        dst.bmax.set(src.bmax);
        dst.cellSize = src.cellSize;
        dst.cellHeight = src.cellHeight;
        dst.borderSize = src.borderSize;
        dst.maxEdgeError = src.maxEdgeError;

        dst.vertices = new int[src.numVertices * 3];
        System.arraycopy(src.vertices, 0, dst.vertices, 0, dst.vertices.length);
        dst.polygons = new int[src.numPolygons * 2 * src.maxVerticesPerPolygon];
        System.arraycopy(src.polygons, 0, dst.polygons, 0, dst.polygons.length);
        dst.regionIds = new int[src.numPolygons];
        System.arraycopy(src.regionIds, 0, dst.regionIds, 0, dst.regionIds.length);
        dst.areaIds = new int[src.numPolygons];
        System.arraycopy(src.areaIds, 0, dst.areaIds, 0, dst.areaIds.length);
        dst.flags = new int[src.numPolygons];
        System.arraycopy(src.flags, 0, dst.flags, 0, dst.flags.length);
        return dst;
    }
}
