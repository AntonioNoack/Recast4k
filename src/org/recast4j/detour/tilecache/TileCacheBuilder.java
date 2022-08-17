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
package org.recast4j.detour.tilecache;

import kotlin.Pair;
import org.joml.Vector3f;
import org.recast4j.Edge;
import org.recast4j.detour.tilecache.io.TileCacheLayerHeaderReader;
import org.recast4j.detour.tilecache.io.TileCacheLayerHeaderWriter;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.recast4j.Vectors.sqr;

public class TileCacheBuilder {

    static final int TILECACHE_NULL_AREA = 0;
    static final int TILECACHE_WALKABLE_AREA = 63;
    static final int TILECACHE_NULL_IDX = 0xffff;

    private static class LayerSweepSpan {
        int numSamples, regionId, neighborId;
    }

    private static class LayerMonotoneRegion {
        int area;
        int regId;
        int areaId;
        List<Integer> neighbors = new ArrayList<>(16);
    }

    private static class TempContour {
        List<Integer> vertices;
        int numVertices;
        List<Integer> poly;

        TempContour() {
            vertices = new ArrayList<>();
            numVertices = 0;
            poly = new ArrayList<>();
        }

        int npoly() {
            return poly.size();
        }

        public void clear() {
            numVertices = 0;
            vertices.clear();
        }
    }

    private final TileCacheLayerHeaderReader reader = new TileCacheLayerHeaderReader();

    void buildTileCacheRegions(TileCacheLayer layer, int walkableClimb) {

        int w = layer.header.width;
        int h = layer.header.height;

        Arrays.fill(layer.regs, (short) 0x00FF);
        LayerSweepSpan[] sweeps = new LayerSweepSpan[w];
        for (int i = 0; i < sweeps.length; i++) {
            sweeps[i] = new LayerSweepSpan();
        }
        // Partition walkable area into monotone regions.
        int[] prevCount = new int[256];
        int regId = 0;

        for (int y = 0; y < h; ++y) {
            if (regId > 0) {
                Arrays.fill(prevCount, 0, regId, 0);
            }
            // memset(prevCount,0,sizeof(char)*regId);
            int sweepId = 0;

            for (int x = 0; x < w; ++x) {
                int idx = x + y * w;
                if (layer.areas[idx] == TILECACHE_NULL_AREA)
                    continue;

                int sid = 0xff;

                // -x
                int xidx = (x - 1) + y * w;
                if (x > 0 && isConnected(layer, idx, xidx, walkableClimb)) {
                    if (layer.regs[xidx] != 0xff)
                        sid = layer.regs[xidx];
                }

                if (sid == 0xff) {
                    sid = sweepId++;
                    sweeps[sid].neighborId = 0xff;
                    sweeps[sid].numSamples = 0;
                }

                // -y
                int yidx = x + (y - 1) * w;
                if (y > 0 && isConnected(layer, idx, yidx, walkableClimb)) {
                    int nr = layer.regs[yidx];
                    if (nr != 0xff) {
                        // Set neighbour when first valid neighbour is
                        // encoutered.
                        if (sweeps[sid].numSamples == 0)
                            sweeps[sid].neighborId = nr;

                        if (sweeps[sid].neighborId == nr) {
                            // Update existing neighbour
                            sweeps[sid].numSamples++;
                            prevCount[nr]++;
                        } else {
                            // This is hit if there is nore than one neighbour.
                            // Invalidate the neighbour.
                            sweeps[sid].neighborId = 0xff;
                        }
                    }
                }

                layer.regs[idx] = (byte) sid;
            }

            // Create unique ID.
            for (int i = 0; i < sweepId; ++i) {
                // If the neighbour is set and there is only one continuous
                // connection to it,
                // the sweep will be merged with the previous one, else new
                // region is created.
                if (sweeps[i].neighborId != 0xff && prevCount[sweeps[i].neighborId] == sweeps[i].numSamples) {
                    sweeps[i].regionId = sweeps[i].neighborId;
                } else {
                    if (regId == 255) {
                        // Region ID's overflow.
                        throw new RuntimeException("Buffer too small");
                    }
                    sweeps[i].regionId = regId++;
                }
            }

            // Remap local sweep ids to region ids.
            for (int x = 0; x < w; ++x) {
                int idx = x + y * w;
                if (layer.regs[idx] != 0xff)
                    layer.regs[idx] = (short) sweeps[layer.regs[idx]].regionId;
            }
        }

        // Allocate and init layer regions.
        int nregs = regId;
        LayerMonotoneRegion[] regs = new LayerMonotoneRegion[nregs];

        for (int i = 0; i < nregs; ++i) {
            regs[i] = new LayerMonotoneRegion();
            regs[i].regId = 0xff;
        }

        // Find region neighbours.
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                int idx = x + y * w;
                int ri = layer.regs[idx];
                if (ri == 0xff)
                    continue;

                // Update area.
                regs[ri].area++;
                regs[ri].areaId = layer.areas[idx];

                // Update neighbours
                int ymi = x + (y - 1) * w;
                if (y > 0 && isConnected(layer, idx, ymi, walkableClimb)) {
                    int rai = layer.regs[ymi];
                    if (rai != 0xff && rai != ri) {
                        addUniqueLast(regs[ri].neighbors, rai);
                        addUniqueLast(regs[rai].neighbors, ri);
                    }
                }
            }
        }

        for (int i = 0; i < nregs; ++i)
            regs[i].regId = i;

        for (int i = 0; i < nregs; ++i) {
            LayerMonotoneRegion reg = regs[i];

            int merge = -1;
            int mergea = 0;
            for (int nei : reg.neighbors) {
                LayerMonotoneRegion regn = regs[nei];
                if (reg.regId == regn.regId)
                    continue;
                if (reg.areaId != regn.areaId)
                    continue;
                if (regn.area > mergea) {
                    if (canMerge(reg.regId, regn.regId, regs, nregs)) {
                        mergea = regn.area;
                        merge = nei;
                    }
                }
            }
            if (merge != -1) {
                int oldId = reg.regId;
                int newId = regs[merge].regId;
                for (int j = 0; j < nregs; ++j)
                    if (regs[j].regId == oldId)
                        regs[j].regId = newId;
            }
        }

        // Compact ids.
        int[] remap = new int[256];
        // Find number of unique regions.
        regId = 0;
        for (int i = 0; i < nregs; ++i)
            remap[regs[i].regId] = 1;
        for (int i = 0; i < 256; ++i)
            if (remap[i] != 0)
                remap[i] = regId++;
        // Remap ids.
        for (int i = 0; i < nregs; ++i)
            regs[i].regId = remap[regs[i].regId];

        layer.regCount = regId;

        for (int i = 0; i < w * h; ++i) {
            if (layer.regs[i] != 0xff)
                layer.regs[i] = (short) regs[layer.regs[i]].regId;
        }

    }

    void addUniqueLast(List<Integer> a, int v) {
        int n = a.size();
        if (n > 0 && a.get(n - 1) == v)
            return;
        a.add(v);
    }

    boolean isConnected(TileCacheLayer layer, int ia, int ib, int walkableClimb) {
        if (layer.areas[ia] != layer.areas[ib])
            return false;
        if (Math.abs(layer.heights[ia] - layer.heights[ib]) > walkableClimb)
            return false;
        return true;
    }

    boolean canMerge(int oldRegId, int newRegId, LayerMonotoneRegion[] regs, int nregs) {
        int count = 0;
        for (int i = 0; i < nregs; ++i) {
            LayerMonotoneRegion reg = regs[i];
            if (reg.regId != oldRegId)
                continue;
            for (int nei : reg.neighbors) {
                if (regs[nei].regId == newRegId)
                    count++;
            }
        }
        return count == 1;
    }

    private void appendVertex(TempContour cont, int x, int y, int z, int r) {
        // Try to merge with existing segments.
        if (cont.numVertices > 1) {
            int pa = (cont.numVertices - 2) * 4;
            int pb = (cont.numVertices - 1) * 4;
            if (cont.vertices.get(pb + 3) == r) {
                if (cont.vertices.get(pa).intValue() == cont.vertices.get(pb).intValue() && cont.vertices.get(pb) == x) {
                    // The vertices are aligned aling x-axis, update z.
                    cont.vertices.set(pb + 1, y);
                    cont.vertices.set(pb + 2, z);
                    return;
                } else if (cont.vertices.get(pa + 2).intValue() == cont.vertices.get(pb + 2).intValue()
                        && cont.vertices.get(pb + 2) == z) {
                    // The vertices are aligned aling z-axis, update x.
                    cont.vertices.set(pb, x);
                    cont.vertices.set(pb + 1, y);
                    return;
                }
            }
        }
        cont.vertices.add(x);
        cont.vertices.add(y);
        cont.vertices.add(z);
        cont.vertices.add(r);
        cont.numVertices++;
    }

    private int getNeighbourReg(TileCacheLayer layer, int ax, int ay, int dir) {
        int w = layer.header.width;
        int ia = ax + ay * w;

        int con = layer.cons[ia] & 0xf;
        int portal = layer.cons[ia] >> 4;
        int mask = 1 << dir;

        if ((con & mask) == 0) {
            // No connection, return portal or hard edge.
            if ((portal & mask) != 0)
                return 0xf8 + dir;
            return 0xff;
        }

        int bx = ax + getDirOffsetX(dir);
        int by = ay + getDirOffsetY(dir);
        int ib = bx + by * w;
        return layer.regs[ib];
    }

    private int getDirOffsetX(int dir) {
        int[] offset = new int[]{-1, 0, 1, 0,};
        return offset[dir & 0x03];
    }

    private int getDirOffsetY(int dir) {
        int[] offset = new int[]{0, 1, 0, -1};
        return offset[dir & 0x03];
    }

    private void walkContour(TileCacheLayer layer, int x, int y, TempContour cont) {
        int w = layer.header.width;
        int h = layer.header.height;

        cont.clear();

        int startX = x;
        int startY = y;
        int startDir = -1;

        for (int i = 0; i < 4; ++i) {
            int dir = (i + 3) & 3;
            int rn = getNeighbourReg(layer, x, y, dir);
            if (rn != layer.regs[x + y * w]) {
                startDir = dir;
                break;
            }
        }
        if (startDir == -1)
            return;

        int dir = startDir;
        int maxIter = w * h;
        int iter = 0;
        while (iter < maxIter) {
            int rn = getNeighbourReg(layer, x, y, dir);

            int nx = x;
            int ny = y;
            int ndir = dir;

            if (rn != layer.regs[x + y * w]) {
                // Solid edge.
                int px = x;
                int pz = y;
                switch (dir) {
                    case 0:
                        pz++;
                        break;
                    case 1:
                        px++;
                        pz++;
                        break;
                    case 2:
                        px++;
                        break;
                }

                // Try to merge with previous vertex.
                appendVertex(cont, px, layer.heights[x + y * w], pz, rn);
                ndir = (dir + 1) & 0x3; // Rotate CW
            } else {
                // Move to next.
                nx = x + getDirOffsetX(dir);
                ny = y + getDirOffsetY(dir);
                ndir = (dir + 3) & 0x3; // Rotate CCW
            }

            if (iter > 0 && x == startX && y == startY && dir == startDir)
                break;

            x = nx;
            y = ny;
            dir = ndir;

            iter++;
        }

        // Remove last vertex if it is duplicate of the first one.
        int pa = (cont.numVertices - 1) * 4;
        int pb = 0;
        if (cont.vertices.get(pa).intValue() == cont.vertices.get(pb).intValue()
                && cont.vertices.get(pa + 2).intValue() == cont.vertices.get(pb + 2).intValue())
            cont.numVertices--;

    }

    private float distancePtSeg(int x, int z, int px, int pz, int qx, int qz) {
        float pqx = qx - px;
        float pqz = qz - pz;
        float dx = x - px;
        float dz = z - pz;
        float d = pqx * pqx + pqz * pqz;
        float t = pqx * dx + pqz * dz;
        if (d > 0)
            t /= d;
        if (t < 0)
            t = 0;
        else if (t > 1)
            t = 1;

        dx = px + t * pqx - x;
        dz = pz + t * pqz - z;

        return dx * dx + dz * dz;
    }

    private void simplifyContour(TempContour cont, float maxError) {
        cont.poly.clear();

        for (int i = 0; i < cont.numVertices; ++i) {
            int j = (i + 1) % cont.numVertices;
            // Check for start of a wall segment.
            int ra = j * 4 + 3;
            int rb = i * 4 + 3;
            if (cont.vertices.get(ra).intValue() != cont.vertices.get(rb).intValue())
                cont.poly.add(i);
        }
        if (cont.npoly() < 2) {
            // If there is no transitions at all,
            // create some initial points for the simplification process.
            // Find lower-left and upper-right vertices of the contour.
            int llx = cont.vertices.get(0);
            int llz = cont.vertices.get(2);
            int lli = 0;
            int urx = cont.vertices.get(0);
            int urz = cont.vertices.get(2);
            int uri = 0;
            for (int i = 1; i < cont.numVertices; ++i) {
                int x = cont.vertices.get(i * 4);
                int z = cont.vertices.get(i * 4 + 2);
                if (x < llx || (x == llx && z < llz)) {
                    llx = x;
                    llz = z;
                    lli = i;
                }
                if (x > urx || (x == urx && z > urz)) {
                    urx = x;
                    urz = z;
                    uri = i;
                }
            }
            cont.poly.clear();
            cont.poly.add(lli);
            cont.poly.add(uri);
        }

        // Add points until all raw points are within
        // error tolerance to the simplified shape.
        for (int i = 0; i < cont.npoly(); ) {
            int ii = (i + 1) % cont.npoly();

            int ai = cont.poly.get(i);
            int ax = cont.vertices.get(ai * 4);
            int az = cont.vertices.get(ai * 4 + 2);

            int bi = cont.poly.get(ii);
            int bx = cont.vertices.get(bi * 4);
            int bz = cont.vertices.get(bi * 4 + 2);

            // Find maximum deviation from the segment.
            float maxd = 0;
            int maxi = -1;
            int ci, cinc, endi;

            // Traverse the segment in lexilogical order so that the
            // max deviation is calculated similarly when traversing
            // opposite segments.
            if (bx > ax || (bx == ax && bz > az)) {
                cinc = 1;
                ci = (ai + cinc) % cont.numVertices;
                endi = bi;
            } else {
                cinc = cont.numVertices - 1;
                ci = (bi + cinc) % cont.numVertices;
                endi = ai;
            }

            // Tessellate only outer edges or edges between areas.
            while (ci != endi) {
                float d = distancePtSeg(cont.vertices.get(ci * 4), cont.vertices.get(ci * 4 + 2), ax, az, bx, bz);
                if (d > maxd) {
                    maxd = d;
                    maxi = ci;
                }
                ci = (ci + cinc) % cont.numVertices;
            }

            // If the max deviation is larger than accepted error,
            // add new point, else continue to next segment.
            if (maxi != -1 && maxd > (maxError * maxError)) {
                cont.poly.add(i + 1, maxi);
            } else {
                ++i;
            }
        }

        // Remap vertices
        int start = 0;
        for (int i = 1; i < cont.npoly(); ++i)
            if (cont.poly.get(i) < cont.poly.get(start))
                start = i;

        cont.numVertices = 0;
        for (int i = 0; i < cont.npoly(); ++i) {
            int j = (start + i) % cont.npoly();
            int src = cont.poly.get(j) * 4;
            int dst = cont.numVertices * 4;
            cont.vertices.set(dst, cont.vertices.get(src));
            cont.vertices.set(dst + 1, cont.vertices.get(src + 1));
            cont.vertices.set(dst + 2, cont.vertices.get(src + 2));
            cont.vertices.set(dst + 3, cont.vertices.get(src + 3));
            cont.numVertices++;
        }
    }

    static Pair<Integer, Boolean> getCornerHeight(TileCacheLayer layer, int x, int y, int z, int walkableClimb) {
        int w = layer.header.width;
        int h = layer.header.height;

        int n = 0;

        int portal = 0xf;
        int height = 0;
        int preg = 0xff;
        boolean allSameReg = true;

        for (int dz = -1; dz <= 0; ++dz) {
            for (int dx = -1; dx <= 0; ++dx) {
                int px = x + dx;
                int pz = z + dz;
                if (px >= 0 && pz >= 0 && px < w && pz < h) {
                    int idx = px + pz * w;
                    int lh = layer.heights[idx];
                    if (Math.abs(lh - y) <= walkableClimb && layer.areas[idx] != TILECACHE_NULL_AREA) {
                        height = Math.max(height, (char) lh);
                        portal &= (layer.cons[idx] >> 4);
                        if (preg != 0xff && preg != layer.regs[idx])
                            allSameReg = false;
                        preg = layer.regs[idx];
                        n++;
                    }
                }
            }
        }

        int portalCount = 0;
        for (int dir = 0; dir < 4; ++dir)
            if ((portal & (1 << dir)) != 0)
                portalCount++;

        boolean shouldRemove = false;
        if (n > 1 && portalCount == 1 && allSameReg) {
            shouldRemove = true;
        }

        return new Pair<>(height, shouldRemove);
    }

    // TODO: move this somewhere else, once the layer meshing is done.
    TileCacheContourSet buildTileCacheContours(TileCacheLayer layer, int walkableClimb, float maxError) {
        int w = layer.header.width;
        int h = layer.header.height;

        TileCacheContourSet lcset = new TileCacheContourSet();
        lcset.nconts = layer.regCount;
        lcset.conts = new TileCacheContour[lcset.nconts];
        for (int i = 0; i < lcset.nconts; i++) {
            lcset.conts[i] = new TileCacheContour();
        }

        // Allocate temp buffer for contour tracing.
        TempContour temp = new TempContour();

        // Find contours.
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                int idx = x + y * w;
                int ri = layer.regs[idx];
                if (ri == 0xff)
                    continue;

                TileCacheContour cont = lcset.conts[ri];

                if (cont.nvertices > 0)
                    continue;

                cont.reg = ri;
                cont.area = layer.areas[idx];

                walkContour(layer, x, y, temp);

                simplifyContour(temp, maxError);

                // Store contour.
                cont.nvertices = temp.numVertices;
                if (cont.nvertices > 0) {
                    cont.vertices = new int[4 * temp.numVertices];

                    for (int i = 0, j = temp.numVertices - 1; i < temp.numVertices; j = i++) {
                        int dst = j * 4;
                        int v = j * 4;
                        int vn = i * 4;
                        int nei = temp.vertices.get(vn + 3); // The neighbour reg
                        // is
                        // stored at segment
                        // vertex of a
                        // segment.
                        Pair<Integer, Boolean> res = getCornerHeight(layer, temp.vertices.get(v), temp.vertices.get(v + 1),
                                temp.vertices.get(v + 2), walkableClimb);
                        int lh = res.getFirst();
                        boolean shouldRemove = res.getSecond();
                        cont.vertices[dst] = temp.vertices.get(v);
                        cont.vertices[dst + 1] = lh;
                        cont.vertices[dst + 2] = temp.vertices.get(v + 2);

                        // Store portal direction and remove status to the
                        // fourth component.
                        cont.vertices[dst + 3] = 0x0f;
                        if (nei != 0xff && nei >= 0xf8)
                            cont.vertices[dst + 3] = nei - 0xf8;
                        if (shouldRemove)
                            cont.vertices[dst + 3] |= 0x80;
                    }
                }
            }
        }
        return lcset;

    }

    static final int VERTEX_BUCKET_COUNT2 = (1 << 8);

    private int computeVertexHash2(int x, int y, int z) {
        int h1 = 0x8da6b343; // Large multiplicative constants;
        int h2 = 0xd8163841; // here arbitrarily chosen primes
        int h3 = 0xcb1ab31f;
        int n = h1 * x + h2 * y + h3 * z;
        return n & (VERTEX_BUCKET_COUNT2 - 1);
    }

    private int addVertex(int x, int y, int z, int[] vertices, int[] firstVert, int[] nextVert, int nv) {
        int bucket = computeVertexHash2(x, 0, z);
        int i = firstVert[bucket];
        while (i != TILECACHE_NULL_IDX) {
            int v = i * 3;
            if (vertices[v] == x && vertices[v + 2] == z && (Math.abs(vertices[v + 1] - y) <= 2))
                return i;
            i = nextVert[i]; // next
        }

        // Could not find, create new.
        i = nv;
        int v = i * 3;
        vertices[v] = x;
        vertices[v + 1] = y;
        vertices[v + 2] = z;
        nextVert[i] = firstVert[bucket];
        firstVert[bucket] = i;
        return i;
    }

    private void buildMeshAdjacency(int[] polys, int npolys, int[] vertices, int nvertices, TileCacheContourSet lcset,
                                    int maxVerticesPerPoly) {
        // Based on code by Eric Lengyel from:
        // http://www.terathon.com/code/edges.php

        int maxEdgeCount = npolys * maxVerticesPerPoly;

        int[] firstEdge = new int[nvertices + maxEdgeCount];
        int nextEdge = nvertices;
        int edgeCount = 0;

        Edge[] edges = new Edge[maxEdgeCount];
        for (int i = 0; i < maxEdgeCount; i++) {
            edges[i] = new Edge();
        }
        for (int i = 0; i < nvertices; i++)
            firstEdge[i] = TILECACHE_NULL_IDX;

        for (int i = 0; i < npolys; ++i) {
            int t = i * maxVerticesPerPoly * 2;
            for (int j = 0; j < maxVerticesPerPoly; ++j) {
                if (polys[t + j] == TILECACHE_NULL_IDX)
                    break;
                int v0 = polys[t + j];
                int v1 = (j + 1 >= maxVerticesPerPoly || polys[t + j + 1] == TILECACHE_NULL_IDX) ? polys[t]
                        : polys[t + j + 1];
                if (v0 < v1) {
                    Edge edge = edges[edgeCount];
                    edge.vert0 = v0;
                    edge.vert0 = v1;
                    edge.poly0 = i;
                    edge.polyEdge0 = j;
                    edge.poly1 = i;
                    edge.polyEdge1 = 0xff;
                    // Insert edge
                    firstEdge[nextEdge + edgeCount] = firstEdge[v0];
                    firstEdge[v0] = (short) edgeCount;
                    edgeCount++;
                }
            }
        }

        for (int i = 0; i < npolys; ++i) {
            int t = i * maxVerticesPerPoly * 2;
            for (int j = 0; j < maxVerticesPerPoly; ++j) {
                if (polys[t + j] == TILECACHE_NULL_IDX)
                    break;
                int v0 = polys[t + j];
                int v1 = (j + 1 >= maxVerticesPerPoly || polys[t + j + 1] == TILECACHE_NULL_IDX) ? polys[t]
                        : polys[t + j + 1];
                if (v0 > v1) {
                    boolean found = false;
                    for (int e = firstEdge[v1]; e != TILECACHE_NULL_IDX; e = firstEdge[nextEdge + e]) {
                        Edge edge = edges[e];
                        if (edge.vert1 == v0 && edge.poly0 == edge.poly1) {
                            edge.poly1 = i;
                            edge.polyEdge1 = j;
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        // Matching edge not found, it is an open edge, add it.
                        Edge edge = edges[edgeCount];
                        edge.vert0 = v1;
                        edge.vert1 = v0;
                        edge.poly0 = (short) i;
                        edge.polyEdge0 = (short) j;
                        edge.poly1 = (short) i;
                        edge.polyEdge1 = 0xff;
                        // Insert edge
                        firstEdge[nextEdge + edgeCount] = firstEdge[v1];
                        firstEdge[v1] = (short) edgeCount;
                        edgeCount++;
                    }
                }
            }
        }

        // Mark portal edges.
        for (int i = 0; i < lcset.nconts; ++i) {
            TileCacheContour cont = lcset.conts[i];
            if (cont.nvertices < 3)
                continue;

            for (int j = 0, k = cont.nvertices - 1; j < cont.nvertices; k = j++) {
                int va = k * 4;
                int vb = j * 4;
                int dir = cont.vertices[va + 3] & 0xf;
                if (dir == 0xf)
                    continue;

                if (dir == 0 || dir == 2) {
                    // Find matching vertical edge
                    int x = cont.vertices[va];
                    int zmin = cont.vertices[va + 2];
                    int zmax = cont.vertices[vb + 2];
                    if (zmin > zmax) {
                        int tmp = zmin;
                        zmin = zmax;
                        zmax = tmp;
                    }

                    for (int m = 0; m < edgeCount; ++m) {
                        Edge e = edges[m];
                        // Skip connected edges.
                        if (e.poly0 != e.poly1)
                            continue;
                        int eva = e.vert0 * 3;
                        int evb = e.vert1 * 3;
                        if (vertices[eva] == x && vertices[evb] == x) {
                            int ezmin = vertices[eva + 2];
                            int ezmax = vertices[evb + 2];
                            if (ezmin > ezmax) {
                                int tmp = ezmin;
                                ezmin = ezmax;
                                ezmax = tmp;
                            }
                            if (overlapRangeExl(zmin, zmax, ezmin, ezmax)) {
                                // Reuse the other polyedge to store dir.
                                e.polyEdge1 = dir;
                            }
                        }
                    }
                } else {
                    // Find matching vertical edge
                    int z = cont.vertices[va + 2];
                    int xmin = cont.vertices[va];
                    int xmax = cont.vertices[vb];
                    if (xmin > xmax) {
                        int tmp = xmin;
                        xmin = xmax;
                        xmax = tmp;
                    }
                    for (int m = 0; m < edgeCount; ++m) {
                        Edge e = edges[m];
                        // Skip connected edges.
                        if (e.poly0 != e.poly1)
                            continue;
                        int eva = e.vert0 * 3;
                        int evb = e.vert1 * 3;
                        if (vertices[eva + 2] == z && vertices[evb + 2] == z) {
                            int exmin = vertices[eva];
                            int exmax = vertices[evb];
                            if (exmin > exmax) {
                                int tmp = exmin;
                                exmin = exmax;
                                exmax = tmp;
                            }
                            if (overlapRangeExl(xmin, xmax, exmin, exmax)) {
                                // Reuse the other polyedge to store dir.
                                e.polyEdge1 = dir;
                            }
                        }
                    }
                }
            }
        }

        // Store adjacency
        for (int i = 0; i < edgeCount; ++i) {
            Edge e = edges[i];
            if (e.poly0 != e.poly1) {
                int p0 = e.poly0 * maxVerticesPerPoly * 2;
                int p1 = e.poly1 * maxVerticesPerPoly * 2;
                polys[p0 + maxVerticesPerPoly + e.polyEdge0] = e.poly1;
                polys[p1 + maxVerticesPerPoly + e.polyEdge1] = e.poly0;
            } else if (e.polyEdge1 != 0xff) {
                int p0 = e.poly0 * maxVerticesPerPoly * 2;
                polys[p0 + maxVerticesPerPoly + e.polyEdge0] = 0x8000 | (short) e.polyEdge1;
            }

        }
    }

    private boolean overlapRangeExl(int amin, int amax, int bmin, int bmax) {
        return amin < bmax && amax > bmin;
    }

    private int prev(int i, int n) {
        return i - 1 >= 0 ? i - 1 : n - 1;
    }

    private int next(int i, int n) {
        return i + 1 < n ? i + 1 : 0;
    }

    private int area2(int[] vertices, int a, int b, int c) {
        return (vertices[b] - vertices[a]) * (vertices[c + 2] - vertices[a + 2])
                - (vertices[c] - vertices[a]) * (vertices[b + 2] - vertices[a + 2]);
    }

    // Returns true iff c is strictly to the left of the directed
    // line through a to b.
    private boolean left(int[] vertices, int a, int b, int c) {
        return area2(vertices, a, b, c) < 0;
    }

    private boolean leftOn(int[] vertices, int a, int b, int c) {
        return area2(vertices, a, b, c) <= 0;
    }

    private boolean collinear(int[] vertices, int a, int b, int c) {
        return area2(vertices, a, b, c) == 0;
    }

    // Returns true iff ab properly intersects cd: they share
    // a point interior to both segments. The properness of the
    // intersection is ensured by using strict leftness.
    private boolean intersectProp(int[] vertices, int a, int b, int c, int d) {
        // Eliminate improper cases.
        if (collinear(vertices, a, b, c) || collinear(vertices, a, b, d) || collinear(vertices, c, d, a)
                || collinear(vertices, c, d, b))
            return false;

        return (left(vertices, a, b, c) ^ left(vertices, a, b, d)) && (left(vertices, c, d, a) ^ left(vertices, c, d, b));
    }

    // Returns T iff (a,b,c) are collinear and point c lies
    // on the closed segement ab.
    private boolean between(int[] vertices, int a, int b, int c) {
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
    private boolean intersect(int[] vertices, int a, int b, int c, int d) {
        if (intersectProp(vertices, a, b, c, d))
            return true;
        else if (between(vertices, a, b, c) || between(vertices, a, b, d) || between(vertices, c, d, a)
                || between(vertices, c, d, b))
            return true;
        else
            return false;
    }

    private boolean vequal(int[] vertices, int a, int b) {
        return vertices[a] == vertices[b] && vertices[a + 2] == vertices[b + 2];
    }

    // Returns T iff (v_i, v_j) is a proper internal *or* external
    // diagonal of P, *ignoring edges incident to v_i and v_j*.
    private boolean diagonalie(int i, int j, int n, int[] vertices, int[] indices) {
        int d0 = (indices[i] & 0x7fff) * 4;
        int d1 = (indices[j] & 0x7fff) * 4;

        // For each edge (k,k+1) of P
        for (int k = 0; k < n; k++) {
            int k1 = next(k, n);
            // Skip edges incident to i or j
            if (!((k == i) || (k1 == i) || (k == j) || (k1 == j))) {
                int p0 = (indices[k] & 0x7fff) * 4;
                int p1 = (indices[k1] & 0x7fff) * 4;

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
    private boolean inCone(int i, int j, int n, int[] vertices, int[] indices) {
        int pi = (indices[i] & 0x7fff) * 4;
        int pj = (indices[j] & 0x7fff) * 4;
        int pi1 = (indices[next(i, n)] & 0x7fff) * 4;
        int pin1 = (indices[prev(i, n)] & 0x7fff) * 4;

        // If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
        if (leftOn(vertices, pin1, pi, pi1))
            return left(vertices, pi, pj, pin1) && left(vertices, pj, pi, pi1);
        // Assume (i-1,i,i+1) not collinear.
        // else P[i] is reflex.
        return !(leftOn(vertices, pi, pj, pi1) && leftOn(vertices, pj, pi, pin1));
    }

    // Returns T iff (v_i, v_j) is a proper internal
    // diagonal of P.
    private boolean diagonal(int i, int j, int n, int[] vertices, int[] indices) {
        return inCone(i, j, n, vertices, indices) && diagonalie(i, j, n, vertices, indices);
    }

    private int triangulate(int n, int[] vertices, int[] indices, int[] tris) {
        int ntris = 0;
        int dst = 0;// tris;
        // The last bit of the index is used to indicate if the vertex can be
        // removed.
        for (int i = 0; i < n; i++) {
            int i1 = next(i, n);
            int i2 = next(i1, n);
            if (diagonal(i, i2, n, vertices, indices))
                indices[i1] |= 0x8000;
        }

        while (n > 3) {
            int minLen = -1;
            int mini = -1;
            for (int i = 0; i < n; i++) {
                int i1 = next(i, n);
                if ((indices[i1] & 0x8000) != 0) {
                    int p0 = (indices[i] & 0x7fff) * 4;
                    int p2 = (indices[next(i1, n)] & 0x7fff) * 4;

                    int dx = vertices[p2] - vertices[p0];
                    int dz = vertices[p2 + 2] - vertices[p0 + 2];
                    int len = dx * dx + dz * dz;
                    if (minLen < 0 || len < minLen) {
                        minLen = len;
                        mini = i;
                    }
                }
            }

            if (mini == -1) {
                // Should not happen.
                /*
                 * printf("mini == -1 ntris=%d n=%d\n", ntris, n); for (int i = 0; i < n; i++) { printf("%d ",
                 * indices[i] & 0x0fffffff); } printf("\n");
                 */
                return -ntris;
            }

            int i = mini;
            int i1 = next(i, n);
            int i2 = next(i1, n);

            tris[dst++] = indices[i] & 0x7fff;
            tris[dst++] = indices[i1] & 0x7fff;
            tris[dst++] = indices[i2] & 0x7fff;
            ntris++;

            // Removes P[i1] by copying P[i+1]...P[n-1] left one index.
            n--;
            for (int k = i1; k < n; k++)
                indices[k] = indices[k + 1];

            if (i1 >= n)
                i1 = 0;
            i = prev(i1, n);
            // Update diagonal flags.
            if (diagonal(prev(i, n), i1, n, vertices, indices))
                indices[i] |= 0x8000;
            else
                indices[i] &= 0x7fff;

            if (diagonal(i, next(i1, n), n, vertices, indices))
                indices[i1] |= 0x8000;
            else
                indices[i1] &= 0x7fff;
        }

        // Append the remaining triangle.
        tris[dst++] = indices[0] & 0x7fff;
        tris[dst++] = indices[1] & 0x7fff;
        tris[dst] = indices[2] & 0x7fff;
        ntris++;

        return ntris;
    }

    private int countPolyVertices(int[] polys, int p, int maxVerticesPerPoly) {
        for (int i = 0; i < maxVerticesPerPoly; ++i)
            if (polys[p + i] == TILECACHE_NULL_IDX)
                return i;
        return maxVerticesPerPoly;
    }

    private boolean uleft(int[] vertices, int a, int b, int c) {
        return (vertices[b] - vertices[a]) * (vertices[c + 2] - vertices[a + 2])
                - (vertices[c] - vertices[a]) * (vertices[b + 2] - vertices[a + 2]) < 0;
    }

    private int[] getPolyMergeValue(int[] polys, int pa, int pb, int[] vertices, int maxVerticesPerPoly) {
        int na = countPolyVertices(polys, pa, maxVerticesPerPoly);
        int nb = countPolyVertices(polys, pb, maxVerticesPerPoly);

        // If the merged polygon would be too big, do not merge.
        if (na + nb - 2 > maxVerticesPerPoly)
            return new int[]{-1, 0, 0};

        // Check if the polygons share an edge.
        int ea = -1;
        int eb = -1;

        for (int i = 0; i < na; ++i) {
            int va0 = polys[pa + i];
            int va1 = polys[pa + (i + 1) % na];
            if (va0 > va1) {
                int tmp = va0;
                va0 = va1;
                va1 = tmp;
            }
            for (int j = 0; j < nb; ++j) {
                int vb0 = polys[pb + j];
                int vb1 = polys[pb + (j + 1) % nb];
                if (vb0 > vb1) {
                    int tmp = vb0;
                    vb0 = vb1;
                    vb1 = tmp;
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

    private void mergePolys(int[] polys, int pa, int pb, int ea, int eb, int maxVerticesPerPoly) {
        int[] tmp = new int[maxVerticesPerPoly * 2];

        int na = countPolyVertices(polys, pa, maxVerticesPerPoly);
        int nb = countPolyVertices(polys, pb, maxVerticesPerPoly);

        // Merge polygons.
        Arrays.fill(tmp, TILECACHE_NULL_IDX);
        int n = 0;
        // Add pa
        for (int i = 0; i < na - 1; ++i)
            tmp[n++] = polys[pa + (ea + 1 + i) % na];
        // Add pb
        for (int i = 0; i < nb - 1; ++i)
            tmp[n++] = polys[pb + (eb + 1 + i) % nb];
        System.arraycopy(tmp, 0, polys, pa, maxVerticesPerPoly);
    }

    private int pushFront(int v, List<Integer> arr) {
        arr.add(0, v);
        return arr.size();
    }

    private int pushBack(int v, List<Integer> arr) {
        arr.add(v);
        return arr.size();
    }

    private boolean canRemoveVertex(TileCachePolyMesh mesh, int rem) {
        // Count number of polygons to remove.
        int maxVerticesPerPoly = mesh.nvp;
        int numRemainingEdges = 0;
        for (int i = 0; i < mesh.numPolygons; ++i) {
            int p = i * mesh.nvp * 2;
            int nv = countPolyVertices(mesh.polys, p, maxVerticesPerPoly);
            int numRemoved = 0;
            int numVertices = 0;
            for (int j = 0; j < nv; ++j) {
                if (mesh.polys[p + j] == rem) {
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
        List<Integer> edges = new ArrayList<>();
        int nedges = 0;

        for (int i = 0; i < mesh.numPolygons; ++i) {
            int p = i * mesh.nvp * 2;
            int nv = countPolyVertices(mesh.polys, p, maxVerticesPerPoly);

            // Collect edges which touches the removed vertex.
            for (int j = 0, k = nv - 1; j < nv; k = j++) {
                if (mesh.polys[p + j] == rem || mesh.polys[p + k] == rem) {
                    // Arrange edge so that a=rem.
                    int a = mesh.polys[p + j], b = mesh.polys[p + k];
                    if (b == rem) {
                        int tmp = a;
                        a = b;
                        b = tmp;
                    }

                    // Check if the edge exists
                    boolean exists = false;
                    for (int m = 0; m < nedges; ++m) {
                        int e = m * 3;
                        if (edges.get(e + 1) == b) {
                            // Exists, increment vertex share count.
                            edges.set(e + 2, edges.get(e + 2) + 1);
                            exists = true;
                        }
                    }
                    // Add new edge.
                    if (!exists) {
                        edges.add(a);
                        edges.add(b);
                        edges.add(1);
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
            if (edges.get(i * 3 + 2) < 2)
                numOpenEdges++;
        }
        return numOpenEdges <= 2;
    }

    private void removeVertex(TileCachePolyMesh mesh, int rem, int maxTris) {
        // Count number of polygons to remove.
        int maxVerticesPerPoly = mesh.nvp;

        int nedges = 0;
        List<Integer> edges = new ArrayList<>();
        int nhole;
        List<Integer> hole = new ArrayList<>();
        List<Integer> harea = new ArrayList<>();

        for (int i = 0; i < mesh.numPolygons; ++i) {
            int p = i * maxVerticesPerPoly * 2;
            int nv = countPolyVertices(mesh.polys, p, maxVerticesPerPoly);
            boolean hasRem = false;
            for (int j = 0; j < nv; ++j)
                if (mesh.polys[p + j] == rem) {
                    hasRem = true;
                    break;
                }
            if (hasRem) {
                // Collect edges which does not touch the removed vertex.
                for (int j = 0, k = nv - 1; j < nv; k = j++) {
                    if (mesh.polys[p + j] != rem && mesh.polys[p + k] != rem) {
                        edges.add(mesh.polys[p + k]);
                        edges.add(mesh.polys[p + j]);
                        edges.add(mesh.areas[i]);
                        nedges++;
                    }
                }
                // Remove the polygon.
                int p2 = (mesh.numPolygons - 1) * maxVerticesPerPoly * 2;
                System.arraycopy(mesh.polys, p2, mesh.polys, p, maxVerticesPerPoly);
                Arrays.fill(mesh.polys, p + maxVerticesPerPoly, p + 2 * maxVerticesPerPoly, TILECACHE_NULL_IDX);
                mesh.areas[i] = mesh.areas[mesh.numPolygons - 1];
                mesh.numPolygons--;
                --i;
            }
        }

        // Remove vertex.
        for (int i = rem; i < mesh.numVertices; ++i) {
            mesh.vertices[i * 3] = mesh.vertices[(i + 1) * 3];
            mesh.vertices[i * 3 + 1] = mesh.vertices[(i + 1) * 3 + 1];
            mesh.vertices[i * 3 + 2] = mesh.vertices[(i + 1) * 3 + 2];
        }
        mesh.numVertices--;

        // Adjust indices to match the removed vertex layout.
        for (int i = 0; i < mesh.numPolygons; ++i) {
            int p = i * maxVerticesPerPoly * 2;
            int nv = countPolyVertices(mesh.polys, p, maxVerticesPerPoly);
            for (int j = 0; j < nv; ++j)
                if (mesh.polys[p + j] > rem)
                    mesh.polys[p + j]--;
        }
        for (int i = 0; i < nedges; ++i) {
            if (edges.get(i * 3) > rem)
                edges.set(i * 3, edges.get(i * 3) - 1);
            if (edges.get(i * 3 + 1) > rem)
                edges.set(i * 3 + 1, edges.get(i * 3 + 1) - 1);
        }

        if (nedges == 0)
            return;

        // Start with one vertex, keep appending connected
        // segments to the start and end of the hole.
        nhole = pushBack(edges.get(0), hole);
        pushBack(edges.get(2), harea);

        while (nedges != 0) {
            boolean match = false;

            for (int i = 0; i < nedges; ++i) {
                int ea = edges.get(i * 3);
                int eb = edges.get(i * 3 + 1);
                int a = edges.get(i * 3 + 2);
                boolean add = false;
                if (hole.get(0) == eb) {
                    // The segment matches the beginning of the hole boundary.
                    nhole = pushFront(ea, hole);
                    pushFront(a, harea);
                    add = true;
                } else if (hole.get(nhole - 1) == ea) {
                    // The segment matches the end of the hole boundary.
                    nhole = pushBack(eb, hole);
                    pushBack(a, harea);
                    add = true;
                }
                if (add) {
                    // The edge segment was added, remove it.
                    edges.set(i * 3, edges.get((nedges - 1) * 3));
                    edges.set(i * 3 + 1, edges.get((nedges - 1) * 3) + 1);
                    edges.set(i * 3 + 2, edges.get((nedges - 1) * 3) + 2);
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
        int[] tpoly = new int[nhole];

        // Generate temp vertex array for triangulation.
        for (int i = 0; i < nhole; ++i) {
            int pi = hole.get(i);
            tvertices[i * 4] = mesh.vertices[pi * 3];
            tvertices[i * 4 + 1] = mesh.vertices[pi * 3 + 1];
            tvertices[i * 4 + 2] = mesh.vertices[pi * 3 + 2];
            tvertices[i * 4 + 3] = 0;
            tpoly[i] = i;
        }

        // Triangulate the hole.
        int ntris = triangulate(nhole, tvertices, tpoly, tris);
        if (ntris < 0) {
            // TODO: issue warning!
            ntris = -ntris;
        }

        int[] polys = new int[ntris * maxVerticesPerPoly];
        int[] pareas = new int[ntris];

        // Build initial polygons.
        int npolys = 0;
        Arrays.fill(polys, 0, ntris * maxVerticesPerPoly, TILECACHE_NULL_IDX);
        for (int j = 0; j < ntris; ++j) {
            int t = j * 3;
            if (tris[t] != tris[t + 1] && tris[t] != tris[t + 2] && tris[t + 1] != tris[t + 2]) {
                polys[npolys * maxVerticesPerPoly] = hole.get(tris[t]);
                polys[npolys * maxVerticesPerPoly + 1] = hole.get(tris[t + 1]);
                polys[npolys * maxVerticesPerPoly + 2] = hole.get(tris[t + 2]);
                pareas[npolys] = harea.get(tris[t]);
                npolys++;
            }
        }
        if (npolys == 0)
            return;

        // Merge polygons.
        if (maxVerticesPerPoly > 3) {
            for (; ; ) {
                // Find best polygons to merge.
                int bestMergeVal = 0;
                int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;

                for (int j = 0; j < npolys - 1; ++j) {
                    int pj = j * maxVerticesPerPoly;
                    for (int k = j + 1; k < npolys; ++k) {
                        int pk = k * maxVerticesPerPoly;
                        int[] pm = getPolyMergeValue(polys, pj, pk, mesh.vertices, maxVerticesPerPoly);
                        int v = pm[0];
                        int ea = pm[1];
                        int eb = pm[2];
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
                    int pa = bestPa * maxVerticesPerPoly;
                    int pb = bestPb * maxVerticesPerPoly;
                    mergePolys(polys, pa, pb, bestEa, bestEb, maxVerticesPerPoly);
                    System.arraycopy(polys, (npolys - 1) * maxVerticesPerPoly, polys, pb, maxVerticesPerPoly);
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
            int p = mesh.numPolygons * maxVerticesPerPoly * 2;
            Arrays.fill(mesh.polys, p, p + maxVerticesPerPoly * 2, TILECACHE_NULL_IDX);
            for (int j = 0; j < maxVerticesPerPoly; ++j)
                mesh.polys[p + j] = polys[i * maxVerticesPerPoly + j];
            mesh.areas[mesh.numPolygons] = pareas[i];
            mesh.numPolygons++;
            if (mesh.numPolygons > maxTris) {
                throw new RuntimeException("Buffer too small");
            }
        }

    }

    TileCachePolyMesh buildTileCachePolyMesh(TileCacheContourSet lcset, int maxVerticesPerPoly) {

        int maxVertices = 0;
        int maxTris = 0;
        int maxVerticesPerCont = 0;
        for (int i = 0; i < lcset.nconts; ++i) {
            // Skip null contours.
            if (lcset.conts[i].nvertices < 3)
                continue;
            maxVertices += lcset.conts[i].nvertices;
            maxTris += lcset.conts[i].nvertices - 2;
            maxVerticesPerCont = Math.max(maxVerticesPerCont, lcset.conts[i].nvertices);
        }

        // TODO: warn about too many vertices?

        TileCachePolyMesh mesh = new TileCachePolyMesh(maxVerticesPerPoly);

        int[] vflags = new int[maxVertices];

        mesh.vertices = new int[maxVertices * 3];
        mesh.polys = new int[maxTris * maxVerticesPerPoly * 2];
        mesh.areas = new int[maxTris];
        // Just allocate and clean the mesh flags array. The user is resposible
        // for filling it.
        mesh.flags = new int[maxTris];

        mesh.numVertices = 0;
        mesh.numPolygons = 0;

        Arrays.fill(mesh.polys, TILECACHE_NULL_IDX);

        int[] firstVert = new int[VERTEX_BUCKET_COUNT2];
        for (int i = 0; i < VERTEX_BUCKET_COUNT2; ++i)
            firstVert[i] = TILECACHE_NULL_IDX;

        int[] nextVert = new int[maxVertices];
        int[] indices = new int[maxVerticesPerCont];
        int[] tris = new int[maxVerticesPerCont * 3];
        int[] polys = new int[maxVerticesPerCont * maxVerticesPerPoly];

        for (int i = 0; i < lcset.nconts; ++i) {
            TileCacheContour cont = lcset.conts[i];

            // Skip null contours.
            if (cont.nvertices < 3)
                continue;

            // Triangulate contour
            for (int j = 0; j < cont.nvertices; ++j)
                indices[j] = j;

            int ntris = triangulate(cont.nvertices, cont.vertices, indices, tris);
            if (ntris <= 0) {
                // TODO: issue warning!
                ntris = -ntris;
            }

            // Add and merge vertices.
            for (int j = 0; j < cont.nvertices; ++j) {
                int v = j * 4;
                indices[j] = addVertex(cont.vertices[v], cont.vertices[v + 1], cont.vertices[v + 2], mesh.vertices, firstVert,
                        nextVert, mesh.numVertices);
                mesh.numVertices = Math.max(mesh.numVertices, indices[j] + 1);
                if ((cont.vertices[v + 3] & 0x80) != 0) {
                    // This vertex should be removed.
                    vflags[indices[j]] = 1;
                }
            }

            // Build initial polygons.
            int npolys = 0;
            Arrays.fill(polys, TILECACHE_NULL_IDX);
            for (int j = 0; j < ntris; ++j) {
                int t = j * 3;
                if (tris[t] != tris[t + 1] && tris[t] != tris[t + 2] && tris[t + 1] != tris[t + 2]) {
                    polys[npolys * maxVerticesPerPoly] = indices[tris[t]];
                    polys[npolys * maxVerticesPerPoly + 1] = indices[tris[t + 1]];
                    polys[npolys * maxVerticesPerPoly + 2] = indices[tris[t + 2]];
                    npolys++;
                }
            }
            if (npolys == 0)
                continue;

            // Merge polygons.
            if (maxVerticesPerPoly > 3) {
                for (; ; ) {
                    // Find best polygons to merge.
                    int bestMergeVal = 0;
                    int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;

                    for (int j = 0; j < npolys - 1; ++j) {
                        int pj = j * maxVerticesPerPoly;
                        for (int k = j + 1; k < npolys; ++k) {
                            int pk = k * maxVerticesPerPoly;
                            int[] pm = getPolyMergeValue(polys, pj, pk, mesh.vertices, maxVerticesPerPoly);
                            int v = pm[0];
                            int ea = pm[1];
                            int eb = pm[2];
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
                        int pa = bestPa * maxVerticesPerPoly;
                        int pb = bestPb * maxVerticesPerPoly;
                        mergePolys(polys, pa, pb, bestEa, bestEb, maxVerticesPerPoly);
                        System.arraycopy(polys, (npolys - 1) * maxVerticesPerPoly, polys, pb, maxVerticesPerPoly);
                        npolys--;
                    } else {
                        // Could not merge any polygons, stop.
                        break;
                    }
                }
            }

            // Store polygons.
            for (int j = 0; j < npolys; ++j) {
                int p = mesh.numPolygons * maxVerticesPerPoly * 2;
                int q = j * maxVerticesPerPoly;
                if (maxVerticesPerPoly >= 0) System.arraycopy(polys, q, mesh.polys, p, maxVerticesPerPoly);
                mesh.areas[mesh.numPolygons] = cont.area;
                mesh.numPolygons++;
                if (mesh.numPolygons > maxTris)
                    throw new RuntimeException("Buffer too small");
            }
        }

        // Remove edge vertices.
        for (int i = 0; i < mesh.numVertices; ++i) {
            if (vflags[i] != 0) {
                if (!canRemoveVertex(mesh, i))
                    continue;
                removeVertex(mesh, i, maxTris);
                // Remove vertex
                // Note: mesh.nvertices is already decremented inside
                // removeVertex()!
                if (mesh.numVertices - i >= 0)
                    System.arraycopy(vflags, i + 1, vflags, i, mesh.numVertices - i);
                --i;
            }
        }

        // Calculate adjacency.
        buildMeshAdjacency(mesh.polys, mesh.numPolygons, mesh.vertices, mesh.numVertices, lcset, maxVerticesPerPoly);

        return mesh;
    }

    public void markCylinderArea(TileCacheLayer layer, Vector3f orig, float cs, float ch, Vector3f pos, float radius,
                                 float height, int areaId) {
        Vector3f bmin = new Vector3f(pos);
        Vector3f bmax = new Vector3f(pos);
        bmin.x -= radius;
        bmin.z -= radius;
        bmax.x += radius;
        bmax.y += height;
        bmax.z += radius;
        float r2 = sqr(radius / cs + 0.5f);

        int w = layer.header.width;
        int h = layer.header.height;
        float ics = 1f / cs;
        float ich = 1f / ch;

        float px = (pos.x - orig.x) * ics;
        float pz = (pos.z - orig.z) * ics;

        int minx = (int) Math.floor((bmin.x - orig.x) * ics);
        int miny = (int) Math.floor((bmin.y - orig.y) * ich);
        int minz = (int) Math.floor((bmin.z - orig.z) * ics);
        int maxx = (int) Math.floor((bmax.x - orig.x) * ics);
        int maxy = (int) Math.floor((bmax.y - orig.y) * ich);
        int maxz = (int) Math.floor((bmax.z - orig.z) * ics);

        minx = Math.max(minx, 0);
        maxx = Math.min(maxx, w - 1);
        minz = Math.max(minz, 0);
        maxz = Math.min(maxz, h - 1);

        for (int z = minz; z <= maxz; ++z) {
            for (int x = minx; x <= maxx; ++x) {
                float dx = x + 0.5f - px;
                float dz = z + 0.5f - pz;
                if (dx * dx + dz * dz > r2)
                    continue;
                int y = layer.heights[x + z * w];
                if (y < miny || y > maxy)
                    continue;
                layer.areas[x + z * w] = (short) areaId;
            }
        }
    }

    public void markBoxArea(TileCacheLayer layer, Vector3f orig, float cs, float ch, Vector3f bmin, Vector3f bmax,
                            int areaId) {
        int w = layer.header.width;
        int h = layer.header.height;
        float ics = 1f / cs;
        float ich = 1f / ch;

        int minx = (int) Math.floor((bmin.x - orig.x) * ics);
        int miny = (int) Math.floor((bmin.y - orig.y) * ich);
        int minz = (int) Math.floor((bmin.z - orig.z) * ics);
        int maxx = (int) Math.floor((bmax.x - orig.x) * ics);
        int maxy = (int) Math.floor((bmax.y - orig.y) * ich);
        int maxz = (int) Math.floor((bmax.z - orig.z) * ics);

        minx = Math.max(minx, 0);
        maxx = Math.min(maxx, w - 1);
        minz = Math.max(minz, 0);
        maxz = Math.min(maxz, h - 1);

        for (int z = minz; z <= maxz; ++z) {
            for (int x = minx; x <= maxx; ++x) {
                int y = layer.heights[x + z * w];
                if (y < miny || y > maxy)
                    continue;
                layer.areas[x + z * w] = (short) areaId;
            }
        }

    }

    public byte[] compressTileCacheLayer(TileCacheLayer layer, ByteOrder order, boolean cCompatibility) {
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        TileCacheLayerHeaderWriter hw = new TileCacheLayerHeaderWriter();
        try {
            hw.write(baos, layer.header, order, cCompatibility);
            int gridSize = layer.header.width * layer.header.height;
            byte[] buffer = new byte[gridSize * 3];
            for (int i = 0; i < gridSize; i++) {
                buffer[i] = (byte) layer.heights[i];
                buffer[gridSize + i] = (byte) layer.areas[i];
                buffer[gridSize * 2 + i] = (byte) layer.cons[i];
            }
            baos.write(buffer);
            return baos.toByteArray();
        } catch (IOException e) {
            throw new RuntimeException(e.getMessage(), e);
        }
    }

    public byte[] compressTileCacheLayer(TileCacheLayerHeader header, int[] heights, int[] areas, int[] cons,
                                         ByteOrder order, boolean cCompatibility) {
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        TileCacheLayerHeaderWriter hw = new TileCacheLayerHeaderWriter();
        try {
            hw.write(baos, header, order, cCompatibility);
            int gridSize = header.width * header.height;
            byte[] buffer = new byte[gridSize * 3];
            for (int i = 0; i < gridSize; i++) {
                buffer[i] = (byte) heights[i];
                buffer[gridSize + i] = (byte) areas[i];
                buffer[gridSize * 2 + i] = (byte) cons[i];
            }
            baos.write(buffer);
            return baos.toByteArray();
        } catch (IOException e) {
            throw new RuntimeException(e.getMessage(), e);
        }
    }

    public TileCacheLayer decompressTileCacheLayer(byte[] compressed, ByteOrder order,
                                                   boolean cCompatibility) {
        ByteBuffer buf = ByteBuffer.wrap(compressed);
        buf.order(order);
        TileCacheLayer layer = new TileCacheLayer();
        try {
            layer.header = reader.read(buf, cCompatibility);
        } catch (IOException e) {
            throw new RuntimeException(e.getMessage(), e);
        }

        int gridSize = layer.header.width * layer.header.height;
        layer.heights = new short[gridSize];
        layer.areas = new short[gridSize];
        layer.cons = new short[gridSize];
        layer.regs = new short[gridSize];
        int go = buf.position();
        for (int i = 0; i < gridSize; i++) {
            layer.heights[i] = (short) (compressed[go + i] & 0xFF);
            layer.areas[i] = (short) (compressed[go + i + gridSize] & 0xFF);
            layer.cons[i] = (short) (compressed[go + i + gridSize * 2] & 0xFF);
        }
        return layer;

    }

    public void markBoxArea(TileCacheLayer layer, Vector3f orig, float cs, float ch, Vector3f center, Vector3f extents, float[] rotAux, int areaId) {
        int w = layer.header.width;
        int h = layer.header.height;
        float ics = 1f / cs;
        float ich = 1f / ch;

        float cx = (center.x - orig.x) * ics;
        float cz = (center.z - orig.z) * ics;

        float maxr = 1.41f * Math.max(extents.x, extents.z);
        int minx = (int) Math.floor(cx - maxr * ics);
        int maxx = (int) Math.floor(cx + maxr * ics);
        int minz = (int) Math.floor(cz - maxr * ics);
        int maxz = (int) Math.floor(cz + maxr * ics);
        int miny = (int) Math.floor((center.y - extents.y - orig.y) * ich);
        int maxy = (int) Math.floor((center.y + extents.y - orig.y) * ich);

        minx = Math.max(minx, 0);
        maxx = Math.min(maxx, w - 1);
        minz = Math.max(minz, 0);
        maxz = Math.min(maxz, h - 1);

        float xhalf = extents.x * ics + 0.5f;
        float zhalf = extents.z * ics + 0.5f;
        for (int z = minz; z <= maxz; ++z) {
            for (int x = minx; x <= maxx; ++x) {
                float x2 = 2f * (x - cx);
                float z2 = 2f * (z - cz);
                float xrot = rotAux[1] * x2 + rotAux[0] * z2;
                if (xrot > xhalf || xrot < -xhalf) continue;
                float zrot = rotAux[1] * z2 - rotAux[0] * x2;
                if (zrot > zhalf || zrot < -zhalf) continue;
                int y = layer.heights[x + z * w];
                if (y < miny || y > maxy) continue;
                layer.areas[x + z * w] = (short) areaId;
            }
        }

    }

}
