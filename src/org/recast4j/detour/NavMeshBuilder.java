/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.detour;

import org.joml.Vector3f;

import java.util.Arrays;
import java.util.Comparator;

import static org.joml.Math.clamp;
import static org.recast4j.Vectors.*;

public class NavMeshBuilder {

    static final int MESH_NULL_IDX = 0xffff;

    private static class CompareItemX implements Comparator<BVNode> {
        @Override
        public int compare(BVNode a, BVNode b) {
            return Integer.compare(a.minX, b.minX);
        }
    }

    private static class CompareItemY implements Comparator<BVNode> {

        @Override
        public int compare(BVNode a, BVNode b) {
            return Integer.compare(a.minY, b.minY);
        }

    }

    private static class CompareItemZ implements Comparator<BVNode> {

        @Override
        public int compare(BVNode a, BVNode b) {
            return Integer.compare(a.minZ, b.minZ);
        }

    }

    private static void calcExtends(BVNode[] items, int imin, int imax, BVNode dst) {

        dst.minX = items[imin].minX;
        dst.minY = items[imin].minY;
        dst.minZ = items[imin].minZ;

        dst.maxX = items[imin].maxX;
        dst.maxY = items[imin].maxY;
        dst.maxZ = items[imin].maxZ;

        for (int i = imin + 1; i < imax; ++i) {
            BVNode it = items[i];
            dst.minX = Math.min(dst.minX, it.minX);
            dst.minY = Math.min(dst.minY, it.minY);
            dst.minZ = Math.min(dst.minZ, it.minZ);
            dst.maxX = Math.max(dst.maxX, it.maxX);
            dst.maxY = Math.max(dst.maxY, it.maxY);
            dst.maxZ = Math.max(dst.maxZ, it.maxZ);
        }
    }

    private static int longestAxis(int x, int y, int z) {
        int axis = 0;
        int maxVal = x;
        if (y > maxVal) {
            axis = 1;
            maxVal = y;
        }
        return z > maxVal ? 2 : axis;
    }

    public static int subdivide(BVNode[] items, int imin, int imax, int curBVNode, BVNode[] BVNodes) {
        int inum = imax - imin;
        int icur = curBVNode;

        BVNode n = new BVNode();
        BVNodes[curBVNode++] = n;

        if (inum == 1) {
            // Leaf
            BVNode i = items[imin];
            n.minX = i.minX;
            n.minY = i.minY;
            n.minZ = i.minZ;
            n.maxX = i.maxX;
            n.maxY = i.maxY;
            n.maxZ = i.maxZ;
            n.index = i.index;
        } else {

            // Split
            calcExtends(items, imin, imax, n);

            int axis = longestAxis(n.maxX - n.minX, n.maxY - n.minY, n.maxZ - n.minZ);

            if (axis == 0) {
                // Sort along x-axis
                Arrays.sort(items, imin, imin + inum, new CompareItemX());
            } else if (axis == 1) {
                // Sort along y-axis
                Arrays.sort(items, imin, imin + inum, new CompareItemY());
            } else {
                // Sort along z-axis
                Arrays.sort(items, imin, imin + inum, new CompareItemZ());
            }

            int isplit = imin + inum / 2;

            // Left
            curBVNode = subdivide(items, imin, isplit, curBVNode, BVNodes);
            // Right
            curBVNode = subdivide(items, isplit, imax, curBVNode, BVNodes);

            int iescape = curBVNode - icur;
            // Negative index means escape.
            n.index = -iescape;
        }
        return curBVNode;
    }

    private static int createBVTree(NavMeshDataCreateParams params, BVNode[] BVNodes) {
        // Build tree
        float quantFactor = 1 / params.cellSize;
        BVNode[] items = new BVNode[params.polyCount];
        for (int i = 0; i < params.polyCount; i++) {
            BVNode it = new BVNode();
            items[i] = it;
            it.index = i;
            // Calc polygon bounds. Use detail meshes if available.
            if (params.detailMeshes != null) {
                int vb = params.detailMeshes[i * 4];
                int ndv = params.detailMeshes[i * 4 + 1];
                Vector3f bmin = new Vector3f();
                Vector3f bmax = new Vector3f();
                int dv = vb * 3;
                copy(bmin, params.detailVertices, dv);
                copy(bmax, params.detailVertices, dv);
                for (int j = 1; j < ndv; j++) {
                    min(bmin, params.detailVertices, dv + j * 3);
                    max(bmax, params.detailVertices, dv + j * 3);
                }

                // BV-tree uses cs for all dimensions
                it.minX = clamp((int) ((bmin.x - params.bmin.x) * quantFactor), 0, 0x7fffffff);
                it.minY = clamp((int) ((bmin.y - params.bmin.y) * quantFactor), 0, 0x7fffffff);
                it.minZ = clamp((int) ((bmin.z - params.bmin.z) * quantFactor), 0, 0x7fffffff);

                it.maxX = clamp((int) ((bmax.x - params.bmin.x) * quantFactor), 0, 0x7fffffff);
                it.maxY = clamp((int) ((bmax.y - params.bmin.y) * quantFactor), 0, 0x7fffffff);
                it.maxZ = clamp((int) ((bmax.z - params.bmin.z) * quantFactor), 0, 0x7fffffff);
            } else {
                int p = i * params.maxVerticesPerPolygon * 2;
                it.minX = it.maxX = params.vertices[params.polys[p] * 3];
                it.minY = it.maxY = params.vertices[params.polys[p] * 3 + 1];
                it.minZ = it.maxZ = params.vertices[params.polys[p] * 3 + 2];

                for (int j = 1; j < params.maxVerticesPerPolygon; ++j) {
                    if (params.polys[p + j] == MESH_NULL_IDX)
                        break;
                    int x = params.vertices[params.polys[p + j] * 3];
                    int y = params.vertices[params.polys[p + j] * 3 + 1];
                    int z = params.vertices[params.polys[p + j] * 3 + 2];

                    if (x < it.minX)
                        it.minX = x;
                    if (y < it.minY)
                        it.minY = y;
                    if (z < it.minZ)
                        it.minZ = z;

                    if (x > it.maxX)
                        it.maxX = x;
                    if (y > it.maxY)
                        it.maxY = y;
                    if (z > it.maxZ)
                        it.maxZ = z;
                }
                // Remap y
                it.minY = (int) Math.floor(it.minY * params.cellHeight * quantFactor);
                it.maxY = (int) Math.ceil(it.maxY * params.cellHeight * quantFactor);
            }
        }

        return subdivide(items, 0, params.polyCount, 0, BVNodes);
    }

    static final int XP = 1;
    static final int ZP = 2;
    static final int XM = 4;
    static final int ZM = 8;

    public static int classifyOffMeshPoint(VectorPtr pt, Vector3f bmin, Vector3f bmax) {

        int outcode = 0;
        outcode |= (pt.get(0) >= bmax.x) ? XP : 0;
        outcode |= (pt.get(2) >= bmax.z) ? ZP : 0;
        outcode |= (pt.get(0) < bmin.x) ? XM : 0;
        outcode |= (pt.get(2) < bmin.z) ? ZM : 0;

        switch (outcode) {
            case XP:
                return 0;
            case XP | ZP:
                return 1;
            case ZP:
                return 2;
            case XM | ZP:
                return 3;
            case XM:
                return 4;
            case XM | ZM:
                return 5;
            case ZM:
                return 6;
            case XP | ZM:
                return 7;
        }

        return 0xff;
    }

    /**
     * Builds navigation mesh tile data from the provided tile creation data.
     *
     * @param params Tile creation data.
     * @return created tile data
     */
    public static MeshData createNavMeshData(NavMeshDataCreateParams params) {
        if (params.vertCount >= 0xffff)
            return null;
        if (params.vertCount == 0 || params.vertices == null)
            return null;
        if (params.polyCount == 0 || params.polys == null)
            return null;

        int nvp = params.maxVerticesPerPolygon;

        // Classify off-mesh connection points. We store only the connections
        // whose start point is inside the tile.
        int[] offMeshConClass = null;
        int storedOffMeshConCount = 0;
        int offMeshConLinkCount = 0;

        if (params.offMeshConCount > 0) {
            offMeshConClass = new int[params.offMeshConCount * 2];

            // Find tight heigh bounds, used for culling out off-mesh start
            // locations.
            float hmin = Float.MAX_VALUE;
            float hmax = -Float.MAX_VALUE;

            if (params.detailVertices != null && params.detailVerticesCount != 0) {
                for (int i = 0; i < params.detailVerticesCount; ++i) {
                    float h = params.detailVertices[i * 3 + 1];
                    hmin = Math.min(hmin, h);
                    hmax = Math.max(hmax, h);
                }
            } else {
                for (int i = 0; i < params.vertCount; ++i) {
                    int iv = i * 3;
                    float h = params.bmin.y + params.vertices[iv + 1] * params.cellHeight;
                    hmin = Math.min(hmin, h);
                    hmax = Math.max(hmax, h);
                }
            }
            hmin -= params.walkableClimb;
            hmax += params.walkableClimb;
            Vector3f bmin = new Vector3f();
            Vector3f bmax = new Vector3f();
            copy(bmin, params.bmin);
            copy(bmax, params.bmax);
            bmin.y = hmin;
            bmax.y = hmax;

            for (int i = 0; i < params.offMeshConCount; ++i) {
                VectorPtr p0 = new VectorPtr(params.offMeshConVertices, (i * 2) * 3);
                VectorPtr p1 = new VectorPtr(params.offMeshConVertices, (i * 2 + 1) * 3);

                offMeshConClass[i * 2] = classifyOffMeshPoint(p0, bmin, bmax);
                offMeshConClass[i * 2 + 1] = classifyOffMeshPoint(p1, bmin, bmax);

                // Zero out off-mesh start positions which are not even
                // potentially touching the mesh.
                if (offMeshConClass[i * 2] == 0xff) {
                    if (p0.get(1) < bmin.y || p0.get(1) > bmax.y)
                        offMeshConClass[i * 2] = 0;
                }

                // Count how many links should be allocated for off-mesh
                // connections.
                if (offMeshConClass[i * 2] == 0xff)
                    offMeshConLinkCount++;
                if (offMeshConClass[i * 2 + 1] == 0xff)
                    offMeshConLinkCount++;

                if (offMeshConClass[i * 2] == 0xff)
                    storedOffMeshConCount++;
            }
        }

        // Off-mesh connectionss are stored as polygons, adjust values.
        int totPolyCount = params.polyCount + storedOffMeshConCount;
        int totVertCount = params.vertCount + storedOffMeshConCount * 2;

        // Find portal edges which are at tile borders.
        int edgeCount = 0;
        int portalCount = 0;
        for (int i = 0; i < params.polyCount; ++i) {
            int p = i * 2 * nvp;
            for (int j = 0; j < nvp; ++j) {
                if (params.polys[p + j] == MESH_NULL_IDX)
                    break;
                edgeCount++;

                if ((params.polys[p + nvp + j] & 0x8000) != 0) {
                    int dir = params.polys[p + nvp + j] & 0xf;
                    if (dir != 0xf)
                        portalCount++;
                }
            }
        }

        int maxLinkCount = edgeCount + portalCount * 2 + offMeshConLinkCount * 2;

        // Find unique detail vertices.
        int uniqueDetailVertCount = 0;
        int detailTriCount = 0;
        if (params.detailMeshes != null) {
            // Has detail mesh, count unique detail vertex count and use input
            // detail tri count.
            detailTriCount = params.detailTriCount;
            for (int i = 0; i < params.polyCount; ++i) {
                int p = i * nvp * 2;
                int ndv = params.detailMeshes[i * 4 + 1];
                int nv = 0;
                for (int j = 0; j < nvp; ++j) {
                    if (params.polys[p + j] == MESH_NULL_IDX)
                        break;
                    nv++;
                }
                ndv -= nv;
                uniqueDetailVertCount += ndv;
            }
        } else {
            // No input detail mesh, build detail mesh from nav polys.
            // No extra detail vertices.
            for (int i = 0; i < params.polyCount; ++i) {
                int p = i * nvp * 2;
                int nv = 0;
                for (int j = 0; j < nvp; ++j) {
                    if (params.polys[p + j] == MESH_NULL_IDX)
                        break;
                    nv++;
                }
                detailTriCount += nv - 2;
            }
        }

        int bvTreeSize = params.buildBvTree ? params.polyCount * 2 : 0;
        MeshHeader header = new MeshHeader();
        float[] navVertices = new float[3 * totVertCount];
        Poly[] navPolys = new Poly[totPolyCount];
        PolyDetail[] navDMeshes = new PolyDetail[params.polyCount];
        float[] navDVertices = new float[3 * uniqueDetailVertCount];
        int[] navDTris = new int[4 * detailTriCount];
        BVNode[] navBvtree = new BVNode[bvTreeSize];
        OffMeshConnection[] offMeshCons = new OffMeshConnection[storedOffMeshConCount];

        // Store header
        header.magic = MeshHeader.DT_NAVMESH_MAGIC;
        header.version = MeshHeader.DT_NAVMESH_VERSION;
        header.x = params.tileX;
        header.y = params.tileZ;
        header.layer = params.tileLayer;
        header.userId = params.userId;
        header.polyCount = totPolyCount;
        header.vertCount = totVertCount;
        header.maxLinkCount = maxLinkCount;
        header.bmin.set(params.bmin);
        header.bmax.set(params.bmax);
        header.detailMeshCount = params.polyCount;
        header.detailVertCount = uniqueDetailVertCount;
        header.detailTriCount = detailTriCount;
        header.bvQuantizationFactor = 1f / params.cellSize;
        header.offMeshBase = params.polyCount;
        header.walkableHeight = params.walkableHeight;
        header.walkableRadius = params.walkableRadius;
        header.walkableClimb = params.walkableClimb;
        header.offMeshConCount = storedOffMeshConCount;
        header.bvNodeCount = bvTreeSize;

        int offMeshVerticesBase = params.vertCount;
        int offMeshPolyBase = params.polyCount;

        // Store vertices
        // Mesh vertices
        for (int i = 0; i < params.vertCount; ++i) {
            int iv = i * 3;
            int v = i * 3;
            navVertices[v] = params.bmin.x + params.vertices[iv] * params.cellSize;
            navVertices[v + 1] = params.bmin.y + params.vertices[iv + 1] * params.cellHeight;
            navVertices[v + 2] = params.bmin.z + params.vertices[iv + 2] * params.cellSize;
        }
        // Off-mesh link vertices.
        int n = 0;
        for (int i = 0; i < params.offMeshConCount; ++i) {
            // Only store connections which start from this tile.
            if (offMeshConClass[i * 2] == 0xff) {
                int linkv = i * 2 * 3;
                int v = (offMeshVerticesBase + n * 2) * 3;
                System.arraycopy(params.offMeshConVertices, linkv, navVertices, v, 6);
                n++;
            }
        }

        // Store polygons
        // Mesh polys
        int src = 0;
        for (int i = 0; i < params.polyCount; ++i) {
            Poly p = new Poly(i, nvp);
            navPolys[i] = p;
            p.vertCount = 0;
            p.flags = params.polyFlags[i];
            p.setArea(params.polyAreas[i]);
            p.setType(Poly.DT_POLYTYPE_GROUND);
            for (int j = 0; j < nvp; ++j) {
                if (params.polys[src + j] == MESH_NULL_IDX)
                    break;
                p.vertices[j] = params.polys[src + j];
                if ((params.polys[src + nvp + j] & 0x8000) != 0) {
                    // Border or portal edge.
                    int dir = params.polys[src + nvp + j] & 0xf;
                    if (dir == 0xf) // Border
                        p.neighborData[j] = 0;
                    else if (dir == 0) // Portal x-
                        p.neighborData[j] = NavMesh.DT_EXT_LINK | 4;
                    else if (dir == 1) // Portal z+
                        p.neighborData[j] = NavMesh.DT_EXT_LINK | 2;
                    else if (dir == 2) // Portal x+
                        p.neighborData[j] = NavMesh.DT_EXT_LINK;
                    else if (dir == 3) // Portal z-
                        p.neighborData[j] = NavMesh.DT_EXT_LINK | 6;
                } else {
                    // Normal connection
                    p.neighborData[j] = params.polys[src + nvp + j] + 1;
                }

                p.vertCount++;
            }
            src += nvp * 2;
        }
        // Off-mesh connection vertices.
        n = 0;
        for (int i = 0; i < params.offMeshConCount; ++i) {
            // Only store connections which start from this tile.
            if (offMeshConClass[i * 2] == 0xff) {
                Poly p = new Poly(offMeshPolyBase + n, nvp);
                navPolys[offMeshPolyBase + n] = p;
                p.vertCount = 2;
                p.vertices[0] = offMeshVerticesBase + n * 2;
                p.vertices[1] = offMeshVerticesBase + n * 2 + 1;
                p.flags = params.offMeshConFlags[i];
                p.setArea(params.offMeshConAreas[i]);
                p.setType(Poly.DT_POLYTYPE_OFFMESH_CONNECTION);
                n++;
            }
        }

        // Store detail meshes and vertices.
        // The nav polygon vertices are stored as the first vertices on each
        // mesh.
        // We compress the mesh data by skipping them and using the navmesh
        // coordinates.
        if (params.detailMeshes != null) {
            int vbase = 0;
            for (int i = 0; i < params.polyCount; ++i) {
                PolyDetail dtl = new PolyDetail();
                navDMeshes[i] = dtl;
                int vb = params.detailMeshes[i * 4];
                int ndv = params.detailMeshes[i * 4 + 1];
                int nv = navPolys[i].vertCount;
                dtl.vertBase = vbase;
                dtl.vertCount = (ndv - nv);
                dtl.triBase = params.detailMeshes[i * 4 + 2];
                dtl.triCount = params.detailMeshes[i * 4 + 3];
                // Copy vertices except the first 'nv' vertices which are equal to
                // nav poly vertices.
                if (ndv - nv != 0) {
                    System.arraycopy(params.detailVertices, (vb + nv) * 3, navDVertices, vbase * 3, 3 * (ndv - nv));
                    vbase += ndv - nv;
                }
            }
            // Store triangles.
            System.arraycopy(params.detailTris, 0, navDTris, 0, 4 * params.detailTriCount);
        } else {
            // Create dummy detail mesh by triangulating polys.
            int tbase = 0;
            for (int i = 0; i < params.polyCount; ++i) {
                PolyDetail dtl = new PolyDetail();
                navDMeshes[i] = dtl;
                int nv = navPolys[i].vertCount;
                dtl.vertBase = 0;
                dtl.vertCount = 0;
                dtl.triBase = tbase;
                dtl.triCount = (nv - 2);
                // Triangulate polygon (local indices).
                for (int j = 2; j < nv; ++j) {
                    int t = tbase * 4;
                    navDTris[t] = 0;
                    navDTris[t + 1] = (j - 1);
                    navDTris[t + 2] = j;
                    // Bit for each edge that belongs to poly boundary.
                    navDTris[t + 3] = (1 << 2);
                    if (j == 2)
                        navDTris[t + 3] |= (1);
                    if (j == nv - 1)
                        navDTris[t + 3] |= (1 << 4);
                    tbase++;
                }
            }
        }

        // Store and create BVtree.
        // TODO: take detail mesh into account! use byte per bbox extent?
        if (params.buildBvTree) {
            // Do not set header.bvBVNodeCount set to make it work look exactly the same as in original Detour
            header.bvNodeCount = createBVTree(params, navBvtree);
        }

        // Store Off-Mesh connections.
        n = 0;
        for (int i = 0; i < params.offMeshConCount; ++i) {
            // Only store connections, which start from this tile.
            if (offMeshConClass[i * 2] == 0xff) {
                OffMeshConnection con = new OffMeshConnection();
                offMeshCons[n] = con;
                con.poly = (offMeshPolyBase + n);
                // Copy connection end-points.
                int endPts = i * 2 * 3;
                copy(con.posA, params.offMeshConVertices, endPts);
                copy(con.posB, params.offMeshConVertices, endPts + 3);
                con.rad = params.offMeshConRad[i];
                con.flags = params.offMeshConDir[i] != 0 ? NavMesh.DT_OFFMESH_CON_BIDIR : 0;
                con.side = offMeshConClass[i * 2 + 1];
                if (params.offMeshConUserID != null)
                    con.userId = params.offMeshConUserID[i];
                n++;
            }
        }

        MeshData nmd = new MeshData();
        nmd.header = header;
        nmd.vertices = navVertices;
        nmd.polygons = navPolys;
        nmd.detailMeshes = navDMeshes;
        nmd.detailVertices = navDVertices;
        nmd.detailTriangles = navDTris;
        nmd.bvTree = navBvtree;
        nmd.offMeshCons = offMeshCons;
        return nmd;
    }

}
