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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import static org.recast4j.recast.RecastConstants.*;

public class RecastContour {

    private static class ContourRegion {
        public Contour outline;
        public ContourHole[] holes;
        public int nholes;
    }

    private static class ContourHole {
        public int leftmost;
        public int minx;
        public int minz;
        public Contour contour;
    }

    private static class PotentialDiagonal {
        public int dist;
        public int vert;
    }

    private static int getCornerHeight(int x, int y, int i, int dir, CompactHeightfield chf, boolean[] isBorderVertex) {
        CompactSpan s = chf.spans[i];
        int ch = s.y;
        int dirp = (dir + 1) & 0x3;

        int[] regs = {0, 0, 0, 0};

        // Combine region and area codes in order to prevent
        // border vertices which are in between two areas to be removed.
        regs[0] = chf.spans[i].reg | (chf.areas[i] << 16);

        if (RecastCommon.getCon(s, dir) != RC_NOT_CONNECTED) {
            int ax = x + RecastCommon.getDirOffsetX(dir);
            int ay = y + RecastCommon.getDirOffsetY(dir);
            int ai = chf.cells[ax + ay * chf.width].index + RecastCommon.getCon(s, dir);
            CompactSpan as = chf.spans[ai];
            ch = Math.max(ch, as.y);
            regs[1] = chf.spans[ai].reg | (chf.areas[ai] << 16);
            if (RecastCommon.getCon(as, dirp) != RC_NOT_CONNECTED) {
                int ax2 = ax + RecastCommon.getDirOffsetX(dirp);
                int ay2 = ay + RecastCommon.getDirOffsetY(dirp);
                int ai2 = chf.cells[ax2 + ay2 * chf.width].index + RecastCommon.getCon(as, dirp);
                CompactSpan as2 = chf.spans[ai2];
                ch = Math.max(ch, as2.y);
                regs[2] = chf.spans[ai2].reg | (chf.areas[ai2] << 16);
            }
        }
        if (RecastCommon.getCon(s, dirp) != RC_NOT_CONNECTED) {
            int ax = x + RecastCommon.getDirOffsetX(dirp);
            int ay = y + RecastCommon.getDirOffsetY(dirp);
            int ai = chf.cells[ax + ay * chf.width].index + RecastCommon.getCon(s, dirp);
            CompactSpan as = chf.spans[ai];
            ch = Math.max(ch, as.y);
            regs[3] = chf.spans[ai].reg | (chf.areas[ai] << 16);
            if (RecastCommon.getCon(as, dir) != RC_NOT_CONNECTED) {
                int ax2 = ax + RecastCommon.getDirOffsetX(dir);
                int ay2 = ay + RecastCommon.getDirOffsetY(dir);
                int ai2 = chf.cells[ax2 + ay2 * chf.width].index + RecastCommon.getCon(as, dir);
                CompactSpan as2 = chf.spans[ai2];
                ch = Math.max(ch, as2.y);
                regs[2] = chf.spans[ai2].reg | (chf.areas[ai2] << 16);
            }
        }

        // Check if the vertex is special edge vertex, these vertices will be removed later.
        for (int a = 0; a < 4; a++) {
            int b = (a + 1) & 0x3;
            int c = (a + 2) & 0x3;
            int d = (a + 3) & 0x3;

            // The vertex is a border vertex there are two same exterior cells in a row,
            // followed by two interior cells and none of the regions are out of bounds.
            boolean twoSameExts = (regs[a] & regs[b] & RC_BORDER_REG) != 0 && regs[a] == regs[b];
            boolean twoInts = ((regs[c] | regs[d]) & RC_BORDER_REG) == 0;
            boolean intsSameArea = (regs[c] >> 16) == (regs[d] >> 16);
            boolean noZeros = regs[a] != 0 && regs[b] != 0 && regs[c] != 0 && regs[d] != 0;
            if (twoSameExts && twoInts && intsSameArea && noZeros) {
                isBorderVertex[0] = true;
                break;
            }
        }

        return ch;
    }

    private static void walkContour(int x, int y, int i, CompactHeightfield chf, int[] flags, List<Integer> points) {
        // Choose the first non-connected edge
        int dir = 0;
        while ((flags[i] & (1 << dir)) == 0)
            dir++;

        int startDir = dir;
        int starti = i;

        int area = chf.areas[i];

        int iter = 0;
        while (++iter < 40000) {
            if ((flags[i] & (1 << dir)) != 0) {
                // Choose the edge corner
                boolean[] isBorderVertex = {false};
                boolean isAreaBorder = false;
                int px = x;
                int py = getCornerHeight(x, y, i, dir, chf, isBorderVertex);
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
                int r = 0;
                CompactSpan s = chf.spans[i];
                if (RecastCommon.getCon(s, dir) != RC_NOT_CONNECTED) {
                    int ax = x + RecastCommon.getDirOffsetX(dir);
                    int ay = y + RecastCommon.getDirOffsetY(dir);
                    int ai = chf.cells[ax + ay * chf.width].index + RecastCommon.getCon(s, dir);
                    r = chf.spans[ai].reg;
                    if (area != chf.areas[ai])
                        isAreaBorder = true;
                }
                if (isBorderVertex[0])
                    r |= RC_BORDER_VERTEX;
                if (isAreaBorder)
                    r |= RC_AREA_BORDER;
                points.add(px);
                points.add(py);
                points.add(pz);
                points.add(r);

                flags[i] &= ~(1 << dir); // Remove visited edges
                dir = (dir + 1) & 0x3; // Rotate CW
            } else {
                int ni = -1;
                int nx = x + RecastCommon.getDirOffsetX(dir);
                int ny = y + RecastCommon.getDirOffsetY(dir);
                CompactSpan s = chf.spans[i];
                if (RecastCommon.getCon(s, dir) != RC_NOT_CONNECTED) {
                    CompactCell nc = chf.cells[nx + ny * chf.width];
                    ni = nc.index + RecastCommon.getCon(s, dir);
                }
                if (ni == -1) {
                    // Should not happen.
                    return;
                }
                x = nx;
                y = ny;
                i = ni;
                dir = (dir + 3) & 0x3; // Rotate CCW
            }

            if (starti == i && startDir == dir) {
                break;
            }
        }
    }

    private static float distancePtSeg(int x, int z, int px, int pz, int qx, int qz) {
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

    private static void simplifyContour(List<Integer> points, List<Integer> simplified, float maxError, int maxEdgeLen,
                                        int buildFlags) {
        // Add initial points.
        boolean hasConnections = false;
        for (int i = 0; i < points.size(); i += 4) {
            if ((points.get(i + 3) & RC_CONTOUR_REG_MASK) != 0) {
                hasConnections = true;
                break;
            }
        }

        if (hasConnections) {
            // The contour has some portals to other regions.
            // Add a new point to every location where the region changes.
            for (int i = 0, ni = points.size() / 4; i < ni; ++i) {
                int ii = (i + 1) % ni;
                boolean differentRegs = (points.get(i * 4 + 3) & RC_CONTOUR_REG_MASK) != (points.get(ii * 4 + 3)
                        & RC_CONTOUR_REG_MASK);
                boolean areaBorders = (points.get(i * 4 + 3) & RC_AREA_BORDER) != (points.get(ii * 4 + 3)
                        & RC_AREA_BORDER);
                if (differentRegs || areaBorders) {
                    simplified.add(points.get(i * 4));
                    simplified.add(points.get(i * 4 + 1));
                    simplified.add(points.get(i * 4 + 2));
                    simplified.add(i);
                }
            }
        }

        if (simplified.size() == 0) {
            // If there is no connections at all,
            // create some initial points for the simplification process.
            // Find lower-left and upper-right vertices of the contour.
            int llx = points.get(0);
            int lly = points.get(1);
            int llz = points.get(2);
            int lli = 0;
            int urx = points.get(0);
            int ury = points.get(1);
            int urz = points.get(2);
            int uri = 0;
            for (int i = 0; i < points.size(); i += 4) {
                int x = points.get(i);
                int y = points.get(i + 1);
                int z = points.get(i + 2);
                if (x < llx || (x == llx && z < llz)) {
                    llx = x;
                    lly = y;
                    llz = z;
                    lli = i / 4;
                }
                if (x > urx || (x == urx && z > urz)) {
                    urx = x;
                    ury = y;
                    urz = z;
                    uri = i / 4;
                }
            }
            simplified.add(llx);
            simplified.add(lly);
            simplified.add(llz);
            simplified.add(lli);

            simplified.add(urx);
            simplified.add(ury);
            simplified.add(urz);
            simplified.add(uri);
        }
        // Add points until all raw points are within
        // error tolerance to the simplified shape.
        int pn = points.size() / 4;
        for (int i = 0; i < simplified.size() / 4; ) {
            int ii = (i + 1) % (simplified.size() / 4);

            int ax = simplified.get(i * 4);
            int az = simplified.get(i * 4 + 2);
            int ai = simplified.get(i * 4 + 3);

            int bx = simplified.get(ii * 4);
            int bz = simplified.get(ii * 4 + 2);
            int bi = simplified.get(ii * 4 + 3);

            // Find maximum deviation from the segment.
            float maxd = 0;
            int maxi = -1;
            int ci, cinc, endi;

            // Traverse the segment in lexilogical order so that the
            // max deviation is calculated similarly when traversing
            // opposite segments.
            if (bx > ax || (bx == ax && bz > az)) {
                cinc = 1;
                ci = (ai + cinc) % pn;
                endi = bi;
            } else {
                cinc = pn - 1;
                ci = (bi + cinc) % pn;
                endi = ai;
                int temp = ax;
                ax = bx;
                bx = temp;
                temp = az;
                az = bz;
                bz = temp;
            }
            // Tessellate only outer edges or edges between areas.
            if ((points.get(ci * 4 + 3) & RC_CONTOUR_REG_MASK) == 0 || (points.get(ci * 4 + 3) & RC_AREA_BORDER) != 0) {
                while (ci != endi) {
                    float d = distancePtSeg(points.get(ci * 4), points.get(ci * 4 + 2), ax, az, bx, bz);
                    if (d > maxd) {
                        maxd = d;
                        maxi = ci;
                    }
                    ci = (ci + cinc) % pn;
                }
            }
            // If the max deviation is larger than accepted error,
            // add new point, else continue to next segment.
            if (maxi != -1 && maxd > (maxError * maxError)) {
                // Add the point.
                simplified.add((i + 1) * 4, points.get(maxi * 4));
                simplified.add((i + 1) * 4 + 1, points.get(maxi * 4 + 1));
                simplified.add((i + 1) * 4 + 2, points.get(maxi * 4 + 2));
                simplified.add((i + 1) * 4 + 3, maxi);
            } else {
                ++i;
            }
        }
        // Split too long edges.
        if (maxEdgeLen > 0 && (buildFlags & (RC_CONTOUR_TESS_WALL_EDGES | RC_CONTOUR_TESS_AREA_EDGES)) != 0) {
            for (int i = 0; i < simplified.size() / 4; ) {
                int ii = (i + 1) % (simplified.size() / 4);

                int ax = simplified.get(i * 4);
                int az = simplified.get(i * 4 + 2);
                int ai = simplified.get(i * 4 + 3);

                int bx = simplified.get(ii * 4);
                int bz = simplified.get(ii * 4 + 2);
                int bi = simplified.get(ii * 4 + 3);

                // Find maximum deviation from the segment.
                int maxi = -1;
                int ci = (ai + 1) % pn;

                // Tessellate only outer edges or edges between areas.
                boolean tess = (buildFlags & RC_CONTOUR_TESS_WALL_EDGES) != 0
                        && (points.get(ci * 4 + 3) & RC_CONTOUR_REG_MASK) == 0;
                // Wall edges.
                // Edges between areas.
                if ((buildFlags & RC_CONTOUR_TESS_AREA_EDGES) != 0 && (points.get(ci * 4 + 3) & RC_AREA_BORDER) != 0)
                    tess = true;

                if (tess) {
                    int dx = bx - ax;
                    int dz = bz - az;
                    if (dx * dx + dz * dz > maxEdgeLen * maxEdgeLen) {
                        // Round based on the segments in lexilogical order so that the
                        // max tesselation is consistent regardles in which direction
                        // segments are traversed.
                        int n = bi < ai ? (bi + pn - ai) : (bi - ai);
                        if (n > 1) {
                            if (bx > ax || (bx == ax && bz > az))
                                maxi = (ai + n / 2) % pn;
                            else
                                maxi = (ai + (n + 1) / 2) % pn;
                        }
                    }
                }

                // If the max deviation is larger than accepted error,
                // add new point, else continue to next segment.
                if (maxi != -1) {
                    // Add the point.
                    simplified.add((i + 1) * 4, points.get(maxi * 4));
                    simplified.add((i + 1) * 4 + 1, points.get(maxi * 4 + 1));
                    simplified.add((i + 1) * 4 + 2, points.get(maxi * 4 + 2));
                    simplified.add((i + 1) * 4 + 3, maxi);
                } else {
                    ++i;
                }
            }
        }
        for (int i = 0; i < simplified.size() / 4; ++i) {
            // The edge vertex flag is take from the current raw point,
            // and the neighbour region is take from the next raw point.
            int ai = (simplified.get(i * 4 + 3) + 1) % pn;
            int bi = simplified.get(i * 4 + 3);
            simplified.set(i * 4 + 3, (points.get(ai * 4 + 3) & (RC_CONTOUR_REG_MASK | RC_AREA_BORDER))
                    | (points.get(bi * 4 + 3) & RC_BORDER_VERTEX));
        }

    }

    private static int calcAreaOfPolygon2D(int[] vertices, int nvertices) {
        int area = 0;
        for (int i = 0, j = nvertices - 1; i < nvertices; j = i++) {
            int vi = i * 4;
            int vj = j * 4;
            area += vertices[vi] * vertices[vj + 2] - vertices[vj] * vertices[vi + 2];
        }
        return (area + 1) / 2;
    }

    private static boolean intersectSegCountour(int d0, int d1, int i, int n, int[] vertices, int[] d0vertices,
                                                int[] d1vertices) {
        // For each edge (k,k+1) of P
        int[] pvertices = new int[4 * 4];
        for (int g = 0; g < 4; g++) {
            pvertices[g] = d0vertices[d0 + g];
            pvertices[4 + g] = d1vertices[d1 + g];
        }
        d0 = 0;
        d1 = 4;
        for (int k = 0; k < n; k++) {
            int k1 = RecastMesh.next(k, n);
            // Skip edges incident to i.
            if (i == k || i == k1)
                continue;
            int p0 = k * 4;
            int p1 = k1 * 4;
            for (int g = 0; g < 4; g++) {
                pvertices[8 + g] = vertices[p0 + g];
                pvertices[12 + g] = vertices[p1 + g];
            }
            p0 = 8;
            p1 = 12;
            if (RecastMesh.vequal(pvertices, d0, p0) || RecastMesh.vequal(pvertices, d1, p0)
                    || RecastMesh.vequal(pvertices, d0, p1) || RecastMesh.vequal(pvertices, d1, p1))
                continue;

            if (RecastMesh.intersect(pvertices, d0, d1, p0, p1))
                return true;
        }
        return false;
    }

    private static boolean inCone(int i, int n, int[] vertices, int pj, int[] vertpj) {
        int pi = i * 4;
        int pi1 = RecastMesh.next(i, n) * 4;
        int pin1 = RecastMesh.prev(i, n) * 4;
        int[] pvertices = new int[4 * 4];
        for (int g = 0; g < 4; g++) {
            pvertices[g] = vertices[pi + g];
            pvertices[4 + g] = vertices[pi1 + g];
            pvertices[8 + g] = vertices[pin1 + g];
            pvertices[12 + g] = vertpj[pj + g];
        }
        pi = 0;
        pi1 = 4;
        pin1 = 8;
        pj = 12;
        // If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
        if (RecastMesh.leftOn(pvertices, pin1, pi, pi1))
            return RecastMesh.left(pvertices, pi, pj, pin1) && RecastMesh.left(pvertices, pj, pi, pi1);
        // Assume (i-1,i,i+1) not collinear.
        // else P[i] is reflex.
        return !(RecastMesh.leftOn(pvertices, pi, pj, pi1) && RecastMesh.leftOn(pvertices, pj, pi, pin1));
    }

    private static void removeDegenerateSegments(List<Integer> simplified) {
        // Remove adjacent vertices which are equal on xz-plane,
        // or else the triangulator will get confused.
        int npts = simplified.size() / 4;
        for (int i = 0; i < npts; ++i) {
            int ni = RecastMesh.next(i, npts);

            // if (vequal(&simplified[i*4], &simplified[ni*4]))
            if (simplified.get(i * 4) == simplified.get(ni * 4)
                    && simplified.get(i * 4 + 2) == simplified.get(ni * 4 + 2)) {
                // Degenerate segment, remove.
                simplified.remove(i * 4);
                simplified.remove(i * 4);
                simplified.remove(i * 4);
                simplified.remove(i * 4);
                npts--;
            }
        }
    }

    private static void mergeContours(Contour ca, Contour cb, int ia, int ib) {
        int maxVertices = ca.numVertices + cb.numVertices + 2;
        int[] vertices = new int[maxVertices * 4];

        int nv = 0;

        // Copy contour A.
        for (int i = 0; i <= ca.numVertices; ++i) {
            int dst = nv * 4;
            int src = ((ia + i) % ca.numVertices) * 4;
            vertices[dst] = ca.vertices[src];
            vertices[dst + 1] = ca.vertices[src + 1];
            vertices[dst + 2] = ca.vertices[src + 2];
            vertices[dst + 3] = ca.vertices[src + 3];
            nv++;
        }

        // Copy contour B
        for (int i = 0; i <= cb.numVertices; ++i) {
            int dst = nv * 4;
            int src = ((ib + i) % cb.numVertices) * 4;
            vertices[dst] = cb.vertices[src];
            vertices[dst + 1] = cb.vertices[src + 1];
            vertices[dst + 2] = cb.vertices[src + 2];
            vertices[dst + 3] = cb.vertices[src + 3];
            nv++;
        }

        ca.vertices = vertices;
        ca.numVertices = nv;

        cb.vertices = null;
        cb.numVertices = 0;

    }

    // Finds the lowest leftmost vertex of a contour.
    private static int[] findLeftMostVertex(Contour contour) {
        int minx = contour.vertices[0];
        int minz = contour.vertices[2];
        int leftmost = 0;
        for (int i = 1; i < contour.numVertices; i++) {
            int x = contour.vertices[i * 4];
            int z = contour.vertices[i * 4 + 2];
            if (x < minx || (x == minx && z < minz)) {
                minx = x;
                minz = z;
                leftmost = i;
            }
        }
        return new int[]{minx, minz, leftmost};
    }

    private static class CompareHoles implements Comparator<ContourHole> {
        @Override
        public int compare(ContourHole a, ContourHole b) {
            return Integer.compare(a.minx, b.minx) * 2 +
                    Integer.compare(a.minz, b.minz);
        }
    }

    private static class CompareDiagDist implements Comparator<PotentialDiagonal> {
        @Override
        public int compare(PotentialDiagonal va, PotentialDiagonal vb) {
            return Float.compare(va.dist, vb.dist);
        }
    }

    private static void mergeRegionHoles(Telemetry ctx, ContourRegion region) {
        // Sort holes from left to right.
        for (int i = 0; i < region.nholes; i++) {
            int[] minleft = findLeftMostVertex(region.holes[i].contour);
            region.holes[i].minx = minleft[0];
            region.holes[i].minz = minleft[1];
            region.holes[i].leftmost = minleft[2];
        }
        Arrays.sort(region.holes, new CompareHoles());

        int maxVertices = region.outline.numVertices;
        for (int i = 0; i < region.nholes; i++)
            maxVertices += region.holes[i].contour.numVertices;

        PotentialDiagonal[] diags = new PotentialDiagonal[maxVertices];
        for (int pd = 0; pd < maxVertices; pd++) {
            diags[pd] = new PotentialDiagonal();
        }
        Contour outline = region.outline;

        // Merge holes into the outline one by one.
        for (int i = 0; i < region.nholes; i++) {
            Contour hole = region.holes[i].contour;

            int index = -1;
            int bestVertex = region.holes[i].leftmost;
            for (int iter = 0; iter < hole.numVertices; iter++) {
                // Find potential diagonals.
                // The 'best' vertex must be in the cone described by 3 cosequtive vertices of the outline.
                // ..o j-1
                // |
                // | * best
                // |
                // j o-----o j+1
                // :
                int ndiags = 0;
                int corner = bestVertex * 4;
                for (int j = 0; j < outline.numVertices; j++) {
                    if (inCone(j, outline.numVertices, outline.vertices, corner, hole.vertices)) {
                        int dx = outline.vertices[j * 4] - hole.vertices[corner];
                        int dz = outline.vertices[j * 4 + 2] - hole.vertices[corner + 2];
                        diags[ndiags].vert = j;
                        diags[ndiags].dist = dx * dx + dz * dz;
                        ndiags++;
                    }
                }
                // Sort potential diagonals by distance, we want to make the connection as short as possible.
                Arrays.sort(diags, 0, ndiags, new CompareDiagDist());

                // Find a diagonal that is not intersecting the outline not the remaining holes.
                for (int j = 0; j < ndiags; j++) {
                    int pt = diags[j].vert * 4;
                    boolean intersect = intersectSegCountour(pt, corner, diags[j].vert, outline.numVertices, outline.vertices,
                            outline.vertices, hole.vertices);
                    for (int k = i; k < region.nholes && !intersect; k++)
                        intersect = intersectSegCountour(pt, corner, -1, region.holes[k].contour.numVertices,
                                region.holes[k].contour.vertices, outline.vertices, hole.vertices);
                    if (!intersect) {
                        index = diags[j].vert;
                        break;
                    }
                }
                // If found non-intersecting diagonal, stop looking.
                if (index != -1)
                    break;
                // All the potential diagonals for the current vertex were intersecting, try next vertex.
                bestVertex = (bestVertex + 1) % hole.numVertices;
            }

            if (index == -1) {
                ctx.warn("mergeHoles: Failed to find merge points for");
                continue;
            }
            mergeContours(region.outline, hole, index, bestVertex);
        }
    }

    /// @par
    ///
    /// The raw contours will match the region outlines exactly. The @p maxError and @p maxEdgeLen
    /// parameters control how closely the simplified contours will match the raw contours.
    ///
    /// Simplified contours are generated such that the vertices for portals between areas match up.
    /// (They are considered mandatory vertices.)
    ///
    /// Setting @p maxEdgeLength to zero will disabled the edge length feature.
    ///
    /// See the #rcConfig documentation for more information on the configuration parameters.
    ///
    /// @see rcAllocContourSet, rcCompactHeightfield, rcContourSet, rcConfig
    public static ContourSet buildContours(Telemetry ctx, CompactHeightfield chf, float maxError, int maxEdgeLen, int buildFlags) {

        int w = chf.width;
        int h = chf.height;
        int borderSize = chf.borderSize;
        ContourSet cset = new ContourSet();

        if (ctx != null) ctx.startTimer("CONTOURS");
        cset.bmin.set(chf.bmin);
        cset.bmax.set(chf.bmax);
        if (borderSize > 0) {
            // If the heightfield was build with bordersize, remove the offset.
            float pad = borderSize * chf.cellSize;
            cset.bmin.x += pad;
            cset.bmin.z += pad;
            cset.bmax.x -= pad;
            cset.bmax.z -= pad;
        }
        cset.cellSize = chf.cellSize;
        cset.cellHeight = chf.cellHeight;
        cset.width = chf.width - chf.borderSize * 2;
        cset.height = chf.height - chf.borderSize * 2;
        cset.borderSize = chf.borderSize;
        cset.maxError = maxError;

        int[] flags = new int[chf.spanCount];

        if (ctx != null) ctx.startTimer("CONTOURS_TRACE");

        // Mark boundaries.
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    int res = 0;
                    CompactSpan s = chf.spans[i];
                    if (chf.spans[i].reg == 0 || (chf.spans[i].reg & RC_BORDER_REG) != 0) {
                        flags[i] = 0;
                        continue;
                    }
                    for (int dir = 0; dir < 4; ++dir) {
                        int r = 0;
                        if (RecastCommon.getCon(s, dir) != RC_NOT_CONNECTED) {
                            int ax = x + RecastCommon.getDirOffsetX(dir);
                            int ay = y + RecastCommon.getDirOffsetY(dir);
                            int ai = chf.cells[ax + ay * w].index + RecastCommon.getCon(s, dir);
                            r = chf.spans[ai].reg;
                        }
                        if (r == chf.spans[i].reg)
                            res |= (1 << dir);
                    }
                    flags[i] = res ^ 0xf; // Inverse, mark non connected edges.
                }
            }
        }

        if (ctx != null) ctx.stopTimer("CONTOURS_TRACE");

        List<Integer> vertices = new ArrayList<>(256);
        List<Integer> simplified = new ArrayList<>(64);

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    if (flags[i] == 0 || flags[i] == 0xf) {
                        flags[i] = 0;
                        continue;
                    }
                    int reg = chf.spans[i].reg;
                    if (reg == 0 || (reg & RC_BORDER_REG) != 0)
                        continue;
                    int area = chf.areas[i];

                    vertices.clear();
                    simplified.clear();

                    if (ctx != null) ctx.startTimer("CONTOURS_WALK");
                    walkContour(x, y, i, chf, flags, vertices);
                    if (ctx != null) ctx.stopTimer("CONTOURS_WALK");

                    if (ctx != null) ctx.startTimer("CONTOURS_SIMPLIFY");
                    simplifyContour(vertices, simplified, maxError, maxEdgeLen, buildFlags);
                    removeDegenerateSegments(simplified);
                    if (ctx != null) ctx.stopTimer("CONTOURS_SIMPLIFY");

                    // Store region->contour remap info.
                    // Create contour.
                    if (simplified.size() / 4 >= 3) {

                        Contour cont = new Contour();
                        cset.contours.add(cont);

                        cont.numVertices = simplified.size() / 4;
                        cont.vertices = new int[simplified.size()];
                        for (int l = 0; l < cont.vertices.length; l++) {
                            cont.vertices[l] = simplified.get(l);
                        }

                        if (borderSize > 0) {
                            // If the heightfield was build with bordersize, remove the offset.
                            for (int j = 0; j < cont.numVertices; ++j) {
                                cont.vertices[j * 4] -= borderSize;
                                cont.vertices[j * 4 + 2] -= borderSize;
                            }
                        }

                        cont.numRawVertices = vertices.size() / 4;
                        cont.rawVertices = new int[vertices.size()];
                        for (int l = 0; l < cont.rawVertices.length; l++) {
                            cont.rawVertices[l] = vertices.get(l);
                        }
                        if (borderSize > 0) {
                            // If the heightfield was build with bordersize, remove the offset.
                            for (int j = 0; j < cont.numRawVertices; ++j) {
                                cont.rawVertices[j * 4] -= borderSize;
                                cont.rawVertices[j * 4 + 2] -= borderSize;
                            }
                        }

                        cont.reg = reg;
                        cont.area = area;
                    }
                }
            }
        }

        // Merge holes if needed.
        if (cset.contours.size() > 0) {
            // Calculate winding of all polygons.
            int[] winding = new int[cset.contours.size()];
            int nholes = 0;
            for (int i = 0; i < cset.contours.size(); ++i) {
                Contour cont = cset.contours.get(i);
                // If the contour is wound backwards, it is a hole.
                winding[i] = calcAreaOfPolygon2D(cont.vertices, cont.numVertices) < 0 ? -1 : 1;
                if (winding[i] < 0)
                    nholes++;
            }

            if (nholes > 0) {
                // Collect outline contour and holes contours per region.
                // We assume that there is one outline and multiple holes.
                int nregions = chf.maxRegions + 1;
                ContourRegion[] regions = new ContourRegion[nregions];
                for (int i = 0; i < nregions; i++) {
                    regions[i] = new ContourRegion();
                }

                for (int i = 0; i < cset.contours.size(); ++i) {
                    Contour cont = cset.contours.get(i);
                    // Positively would contours are outlines, negative holes.
                    if (winding[i] > 0) {
                        if (regions[cont.reg].outline != null) {
                            throw new RuntimeException(
                                    "rcBuildContours: Multiple outlines for region " + cont.reg + ".");
                        }
                        regions[cont.reg].outline = cont;
                    } else {
                        regions[cont.reg].nholes++;
                    }
                }
                for (int i = 0; i < nregions; i++) {
                    if (regions[i].nholes > 0) {
                        regions[i].holes = new ContourHole[regions[i].nholes];
                        for (int nh = 0; nh < regions[i].nholes; nh++) {
                            regions[i].holes[nh] = new ContourHole();
                        }
                        regions[i].nholes = 0;
                    }
                }
                for (int i = 0; i < cset.contours.size(); ++i) {
                    Contour cont = cset.contours.get(i);
                    ContourRegion reg = regions[cont.reg];
                    if (winding[i] < 0)
                        reg.holes[reg.nholes++].contour = cont;
                }

                // Finally merge each regions holes into the outline.
                for (int i = 0; i < nregions; i++) {
                    ContourRegion reg = regions[i];
                    if (reg.nholes == 0)
                        continue;

                    if (reg.outline != null) {
                        mergeRegionHoles(ctx, reg);
                    } else {
                        // The region does not have an outline.
                        // This can happen if the contour becomes self-overlapping because of
                        // too aggressive simplification settings.
                        throw new RuntimeException("rcBuildContours: Bad outline for region " + i
                                + ", contour simplification is likely too aggressive.");
                    }
                }
            }
        }
        if (ctx != null) ctx.stopTimer("CONTOURS");
        return cset;
    }
}
