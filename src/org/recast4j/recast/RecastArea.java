/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4J Copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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

import org.joml.Vector3f;
import org.recast4j.Vectors;

import java.util.Arrays;

import static org.recast4j.recast.RecastConstants.RC_NOT_CONNECTED;
import static org.recast4j.recast.RecastConstants.RC_NULL_AREA;

public class RecastArea {

    /**
     * Basically, any spans that are closer to a boundary or obstruction than the specified radius are marked as unwalkable.
     * This method is usually called immediately after the heightfield has been built.
     * */
    public static void erodeWalkableArea(Telemetry ctx, int radius, CompactHeightfield chf) {
        int w = chf.width;
        int h = chf.height;
        if (ctx != null) ctx.startTimer("ERODE_AREA");

        int[] dist = new int[chf.spanCount];
        Arrays.fill(dist, 255);
        // Mark boundary cells.
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    if (chf.areas[i] == RC_NULL_AREA) {
                        dist[i] = 0;
                    } else {
                        CompactSpan s = chf.spans[i];
                        int nc = 0;
                        for (int dir = 0; dir < 4; ++dir) {
                            if (RecastCommon.getCon(s, dir) != RC_NOT_CONNECTED) {
                                int nx = x + RecastCommon.getDirOffsetX(dir);
                                int ny = y + RecastCommon.getDirOffsetY(dir);
                                int nidx = chf.cells[nx + ny * w].index + RecastCommon.getCon(s, dir);
                                if (chf.areas[nidx] != RC_NULL_AREA) {
                                    nc++;
                                }
                            }
                        }
                        // At least one missing neighbour.
                        if (nc != 4)
                            dist[i] = 0;
                    }
                }
            }
        }

        int nd;

        // Pass 1
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];

                    if (RecastCommon.getCon(s, 0) != RC_NOT_CONNECTED) {
                        // (-1,0)
                        int ax = x + RecastCommon.getDirOffsetX(0);
                        int ay = y + RecastCommon.getDirOffsetY(0);
                        int ai = chf.cells[ax + ay * w].index + RecastCommon.getCon(s, 0);
                        CompactSpan as = chf.spans[ai];
                        nd = Math.min(dist[ai] + 2, 255);
                        if (nd < dist[i])
                            dist[i] = nd;

                        // (-1,-1)
                        if (RecastCommon.getCon(as, 3) != RC_NOT_CONNECTED) {
                            int aax = ax + RecastCommon.getDirOffsetX(3);
                            int aay = ay + RecastCommon.getDirOffsetY(3);
                            int aai = chf.cells[aax + aay * w].index + RecastCommon.getCon(as, 3);
                            nd = Math.min(dist[aai] + 3, 255);
                            if (nd < dist[i])
                                dist[i] = nd;
                        }
                    }
                    if (RecastCommon.getCon(s, 3) != RC_NOT_CONNECTED) {
                        // (0,-1)
                        int ax = x + RecastCommon.getDirOffsetX(3);
                        int ay = y + RecastCommon.getDirOffsetY(3);
                        int ai = chf.cells[ax + ay * w].index + RecastCommon.getCon(s, 3);
                        CompactSpan as = chf.spans[ai];
                        nd = Math.min(dist[ai] + 2, 255);
                        if (nd < dist[i])
                            dist[i] = nd;

                        // (1,-1)
                        if (RecastCommon.getCon(as, 2) != RC_NOT_CONNECTED) {
                            int aax = ax + RecastCommon.getDirOffsetX(2);
                            int aay = ay + RecastCommon.getDirOffsetY(2);
                            int aai = chf.cells[aax + aay * w].index + RecastCommon.getCon(as, 2);
                            nd = Math.min(dist[aai] + 3, 255);
                            if (nd < dist[i])
                                dist[i] = nd;
                        }
                    }
                }
            }
        }

        // Pass 2
        for (int y = h - 1; y >= 0; --y) {
            for (int x = w - 1; x >= 0; --x) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];

                    if (RecastCommon.getCon(s, 2) != RC_NOT_CONNECTED) {
                        // (1,0)
                        int ax = x + RecastCommon.getDirOffsetX(2);
                        int ay = y + RecastCommon.getDirOffsetY(2);
                        int ai = chf.cells[ax + ay * w].index + RecastCommon.getCon(s, 2);
                        CompactSpan as = chf.spans[ai];
                        nd = Math.min(dist[ai] + 2, 255);
                        if (nd < dist[i])
                            dist[i] = nd;

                        // (1,1)
                        if (RecastCommon.getCon(as, 1) != RC_NOT_CONNECTED) {
                            int aax = ax + RecastCommon.getDirOffsetX(1);
                            int aay = ay + RecastCommon.getDirOffsetY(1);
                            int aai = chf.cells[aax + aay * w].index + RecastCommon.getCon(as, 1);
                            nd = Math.min(dist[aai] + 3, 255);
                            if (nd < dist[i])
                                dist[i] = nd;
                        }
                    }
                    if (RecastCommon.getCon(s, 1) != RC_NOT_CONNECTED) {
                        // (0,1)
                        int ax = x + RecastCommon.getDirOffsetX(1);
                        int ay = y + RecastCommon.getDirOffsetY(1);
                        int ai = chf.cells[ax + ay * w].index + RecastCommon.getCon(s, 1);
                        CompactSpan as = chf.spans[ai];
                        nd = Math.min(dist[ai] + 2, 255);
                        if (nd < dist[i])
                            dist[i] = nd;

                        // (-1,1)
                        if (RecastCommon.getCon(as, 0) != RC_NOT_CONNECTED) {
                            int aax = ax + RecastCommon.getDirOffsetX(0);
                            int aay = ay + RecastCommon.getDirOffsetY(0);
                            int aai = chf.cells[aax + aay * w].index + RecastCommon.getCon(as, 0);
                            nd = Math.min(dist[aai] + 3, 255);
                            if (nd < dist[i])
                                dist[i] = nd;
                        }
                    }
                }
            }
        }

        int thr = radius * 2;
        for (int i = 0; i < chf.spanCount; ++i)
            if (dist[i] < thr)
                chf.areas[i] = RC_NULL_AREA;

        ctx.stopTimer("ERODE_AREA");
    }

    /// @par
    ///
    /// This filter is usually applied after applying area id's using functions
    /// such as #rcMarkBoxArea, #rcMarkConvexPolyArea, and #rcMarkCylinderArea.
    ///
    /// @see rcCompactHeightfield
    @SuppressWarnings("unused")
    public boolean medianFilterWalkableArea(Telemetry ctx, CompactHeightfield chf) {

        int w = chf.width;
        int h = chf.height;

        if (ctx != null) ctx.startTimer("MEDIAN_AREA");

        int[] areas = new int[chf.spanCount];

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];
                    if (chf.areas[i] == RC_NULL_AREA) {
                        areas[i] = chf.areas[i];
                        continue;
                    }

                    int[] nei = new int[9];
                    for (int j = 0; j < 9; ++j)
                        nei[j] = chf.areas[i];

                    for (int dir = 0; dir < 4; ++dir) {
                        if (RecastCommon.getCon(s, dir) != RC_NOT_CONNECTED) {
                            int ax = x + RecastCommon.getDirOffsetX(dir);
                            int ay = y + RecastCommon.getDirOffsetY(dir);
                            int ai = chf.cells[ax + ay * w].index + RecastCommon.getCon(s, dir);
                            if (chf.areas[ai] != RC_NULL_AREA)
                                nei[dir * 2] = chf.areas[ai];

                            CompactSpan as = chf.spans[ai];
                            int dir2 = (dir + 1) & 0x3;
                            if (RecastCommon.getCon(as, dir2) != RC_NOT_CONNECTED) {
                                int ax2 = ax + RecastCommon.getDirOffsetX(dir2);
                                int ay2 = ay + RecastCommon.getDirOffsetY(dir2);
                                int ai2 = chf.cells[ax2 + ay2 * w].index + RecastCommon.getCon(as, dir2);
                                if (chf.areas[ai2] != RC_NULL_AREA)
                                    nei[dir * 2 + 1] = chf.areas[ai2];
                            }
                        }
                    }
                    Arrays.sort(nei);
                    areas[i] = nei[4];
                }
            }
        }
        chf.areas = areas;

        ctx.stopTimer("MEDIAN_AREA");

        return true;
    }

    @SuppressWarnings("unused")
    public void markBoxArea(Telemetry ctx, Vector3f bmin, Vector3f bmax, AreaModification areaMod, CompactHeightfield chf) {
        // The value of spacial parameters are in world units.
        if (ctx != null) ctx.startTimer("MARK_BOX_AREA");

        int minx = (int) ((bmin.x - chf.bmin.x) / chf.cellSize);
        int miny = (int) ((bmin.y - chf.bmin.y) / chf.cellHeight);
        int minz = (int) ((bmin.z - chf.bmin.z) / chf.cellSize);
        int maxx = (int) ((bmax.x - chf.bmin.x) / chf.cellSize);
        int maxy = (int) ((bmax.y - chf.bmin.y) / chf.cellHeight);
        int maxz = (int) ((bmax.z - chf.bmin.z) / chf.cellSize);

        minx = Math.max(minx, 0);
        maxx = Math.min(maxx, chf.width - 1);
        minz = Math.max(minz, 0);
        maxz = Math.min(maxz, chf.height - 1);

        for (int z = minz; z <= maxz; ++z) {
            for (int x = minx; x <= maxx; ++x) {
                CompactCell c = chf.cells[x + z * chf.width];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];
                    if (s.y >= miny && s.y <= maxy) {
                        if (chf.areas[i] != RC_NULL_AREA)
                            chf.areas[i] = areaMod.apply(chf.areas[i]);
                    }
                }
            }
        }

        if (ctx != null) ctx.stopTimer("MARK_BOX_AREA");

    }

    static boolean pointInPoly(float[] vertices, Vector3f p) {
        boolean c = false;
        int i, j;
        for (i = 0, j = vertices.length - 3; i < vertices.length; j = i, i += 3) {
            if (((vertices[i + 2] > p.z) != (vertices[j + 2] > p.z)) && (p.x < (vertices[j] - vertices[i]) * (p.z - vertices[i + 2]) / (vertices[j + 2] - vertices[i + 2]) + vertices[i]))
                c = !c;
        }
        return c;
    }

    /// @par
    ///
    /// The value of spacial parameters are in world units.
    ///
    /// The y-values of the polygon vertices are ignored. So the polygon is effectively
    /// projected onto the xz-plane at @p hmin, then extruded to @p hmax.
    ///
    /// @see rcCompactHeightfield, rcMedianFilterWalkableArea
    public static void markConvexPolyArea(Telemetry ctx, float[] vertices, float hmin, float hmax, AreaModification areaMod,
                                          CompactHeightfield chf) {
        ctx.startTimer("MARK_CONVEXPOLY_AREA");

        Vector3f bmin = new Vector3f(), bmax = new Vector3f();
        Vectors.copy(bmin, vertices, 0);
        Vectors.copy(bmax, vertices, 0);
        for (int i = 3; i < vertices.length; i += 3) {
            Vectors.min(bmin, vertices, i);
            Vectors.max(bmax, vertices, i);
        }
        bmin.y = hmin;
        bmax.y = hmax;

        int minx = (int) ((bmin.x - chf.bmin.x) / chf.cellSize);
        int miny = (int) ((bmin.y - chf.bmin.y) / chf.cellHeight);
        int minz = (int) ((bmin.z - chf.bmin.z) / chf.cellSize);
        int maxx = (int) ((bmax.x - chf.bmin.x) / chf.cellSize);
        int maxy = (int) ((bmax.y - chf.bmin.y) / chf.cellHeight);
        int maxz = (int) ((bmax.z - chf.bmin.z) / chf.cellSize);

        minx = Math.max(minx, 0);
        maxx = Math.min(maxx, chf.width - 1);
        minz = Math.max(minz, 0);
        maxz = Math.min(maxz, chf.height - 1);

        for (int z = minz; z <= maxz; ++z) {
            for (int x = minx; x <= maxx; ++x) {
                CompactCell c = chf.cells[x + z * chf.width];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];
                    if (chf.areas[i] == RC_NULL_AREA)
                        continue;
                    if (s.y >= miny && s.y <= maxy) {
                        Vector3f p = new Vector3f();
                        p.x = chf.bmin.x + (x + 0.5f) * chf.cellSize;
                        p.y = 0;
                        p.z = chf.bmin.z + (z + 0.5f) * chf.cellSize;

                        if (pointInPoly(vertices, p)) {
                            chf.areas[i] = areaMod.apply(chf.areas[i]);
                        }
                    }
                }
            }
        }

        ctx.stopTimer("MARK_CONVEXPOLY_AREA");
    }

    /// @par
    ///
    /// The value of spacial parameters are in world units.
    ///
    /// @see rcCompactHeightfield, rcMedianFilterWalkableArea
    @SuppressWarnings("unused")
    public void markCylinderArea(Telemetry ctx, Vector3f pos, float r, float h, AreaModification areaMod,
                                 CompactHeightfield chf) {

        ctx.startTimer("MARK_CYLINDER_AREA");

        Vector3f bmin = new Vector3f(), bmax = new Vector3f();
        bmin.x = pos.x - r;
        bmin.y = pos.y;
        bmin.z = pos.z - r;
        bmax.x = pos.x + r;
        bmax.y = pos.y + h;
        bmax.z = pos.z + r;
        float r2 = r * r;

        int minx = (int) ((bmin.x - chf.bmin.x) / chf.cellSize);
        int miny = (int) ((bmin.y - chf.bmin.y) / chf.cellHeight);
        int minz = (int) ((bmin.z - chf.bmin.z) / chf.cellSize);
        int maxx = (int) ((bmax.x - chf.bmin.x) / chf.cellSize);
        int maxy = (int) ((bmax.y - chf.bmin.y) / chf.cellHeight);
        int maxz = (int) ((bmax.z - chf.bmin.z) / chf.cellSize);

        minx = Math.max(minx, 0);
        minz = Math.max(minz, 0);
        maxx = Math.min(maxx, chf.width - 1);
        maxy = Math.min(maxy, chf.height - 1);

        for (int z = minz; z <= maxz; ++z) {
            for (int x = minx; x <= maxx; ++x) {
                CompactCell c = chf.cells[x + z * chf.width];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];

                    if (chf.areas[i] == RC_NULL_AREA)
                        continue;

                    if (s.y >= miny && s.y <= maxy) {
                        float sx = chf.bmin.x + (x + 0.5f) * chf.cellSize;
                        float sz = chf.bmin.z + (z + 0.5f) * chf.cellSize;
                        float dx = sx - pos.x;
                        float dz = sz - pos.z;

                        if (dx * dx + dz * dz < r2) {
                            chf.areas[i] = areaMod.apply(chf.areas[i]);
                        }
                    }
                }
            }
        }
        ctx.stopTimer("MARK_CYLINDER_AREA");
    }

}
