package org.recast4j.recast;

import org.joml.Vector3f;
import org.recast4j.IntArrayList;

import java.util.Arrays;

import static org.recast4j.recast.RecastConstants.RC_NOT_CONNECTED;
import static org.recast4j.recast.RecastMeshDetail0.*;

public class RecastMeshDetail {

    static int MAX_VERTS = 127;
    static int MAX_TRIS = 255; // Max tris for delaunay is 2n-2-k (n=num vertices, k=num hull vertices).
    static int MAX_VERTS_PER_EDGE = 32;

    static int RC_UNSET_HEIGHT = RecastConstants.INSTANCE.getSPAN_MAX_HEIGHT();
    static int EV_UNDEF = -1;
    static int EV_HULL = -2;

    static int buildPolyDetail(
            float[] in, int nin, float sampleDist, float sampleMaxError,
            int heightSearchRadius, CompactHeightfield chf, RecastMeshDetail0.HeightPatch hp, float[] vertices, IntArrayList tris) {

        IntArrayList samples = new IntArrayList(512);

        int nverts;
        float[] edge = new float[(MAX_VERTS_PER_EDGE + 1) * 3];
        int[] hull = new int[MAX_VERTS];
        int nhull = 0;

        nverts = nin;

        for (int i = 0; i < nin; ++i) {
            copy(vertices, i * 3, in, i * 3);
        }
        tris.clear();

        float cs = chf.getCellSize();
        float ics = 1.0f / cs;

        // Calculate minimum extents of the polygon based on input data.
        float minExtent = polyMinExtent(vertices, nverts);

        // Tessellate outlines.
        // This is done in separate pass in order to ensure
        // seamless height values across the ply boundaries.
        if (sampleDist > 0) {
            for (int i = 0, j = nin - 1; i < nin; j = i++) {
                int vj = j * 3;
                int vi = i * 3;
                boolean swapped = false;
                // Make sure the segments are always handled in same order
                // using lexological sort or else there will be seams.
                if (Math.abs(in[vj] - in[vi]) < 1e-6f) {
                    if (in[vj + 2] > in[vi + 2]) {
                        int temp = vi;
                        vi = vj;
                        vj = temp;
                        swapped = true;
                    }
                } else {
                    if (in[vj] > in[vi]) {
                        int temp = vi;
                        vi = vj;
                        vj = temp;
                        swapped = true;
                    }
                }
                // Create samples along the edge.
                float dx = in[vi] - in[vj];
                float dy = in[vi + 1] - in[vj + 1];
                float dz = in[vi + 2] - in[vj + 2];
                float d = (float) Math.sqrt(dx * dx + dz * dz);
                int nn = 1 + (int) Math.floor(d / sampleDist);
                if (nn >= MAX_VERTS_PER_EDGE) {
                    nn = MAX_VERTS_PER_EDGE - 1;
                }
                if (nverts + nn >= MAX_VERTS) {
                    nn = MAX_VERTS - 1 - nverts;
                }

                for (int k = 0; k <= nn; ++k) {
                    float u = (float) k / (float) nn;
                    int pos = k * 3;
                    edge[pos] = in[vj] + dx * u;
                    edge[pos + 1] = in[vj + 1] + dy * u;
                    edge[pos + 2] = in[vj + 2] + dz * u;
                    edge[pos + 1] = getHeight(edge[pos], edge[pos + 1], edge[pos + 2], ics, chf.getCellHeight(),
                            heightSearchRadius, hp) * chf.getCellHeight();
                }
                // Simplify samples.
                int[] idx = new int[MAX_VERTS_PER_EDGE];
                idx[0] = 0;
                idx[1] = nn;
                int nidx = 2;
                for (int k = 0; k < nidx - 1; ) {
                    int a = idx[k];
                    int b = idx[k + 1];
                    int va = a * 3;
                    int vb = b * 3;
                    // Find maximum deviation along the segment.
                    float maxd = 0;
                    int maxi = -1;
                    for (int m = a + 1; m < b; ++m) {
                        float dev = distancePtSeg(edge, m * 3, va, vb);
                        if (dev > maxd) {
                            maxd = dev;
                            maxi = m;
                        }
                    }
                    // If the max deviation is larger than accepted error,
                    // add new point, else continue to next segment.
                    if (maxi != -1 && maxd > sampleMaxError * sampleMaxError) {
                        if (nidx - k >= 0) System.arraycopy(idx, k, idx, k + 1, nidx - k);
                        idx[k + 1] = maxi;
                        nidx++;
                    } else {
                        ++k;
                    }
                }

                hull[nhull++] = j;
                // Add new vertices.
                if (swapped) {
                    for (int k = nidx - 2; k > 0; --k) {
                        copy(vertices, nverts * 3, edge, idx[k] * 3);
                        hull[nhull++] = nverts;
                        nverts++;
                    }
                } else {
                    for (int k = 1; k < nidx - 1; ++k) {
                        copy(vertices, nverts * 3, edge, idx[k] * 3);
                        hull[nhull++] = nverts;
                        nverts++;
                    }
                }
            }
        }

        // If the polygon minimum extent is small (sliver or small triangle), do not try to add internal points.
        if (minExtent < sampleDist * 2) {
            triangulateHull(vertices, nhull, hull, nin, tris);
            return nverts;
        }

        // Tessellate the base mesh.
        // We're using the triangulateHull instead of delaunayHull as it tends to
        // create a bit better triangulation for long thin triangles when there
        // are no internal points.
        triangulateHull(vertices, nhull, hull, nin, tris);

        if (tris.getSize() == 0) {
            // Could not triangulate the poly, make sure there is some valid data there.
            throw new RuntimeException("buildPolyDetail: Could not triangulate polygon (" + nverts + ") vertices).");
        }

        if (sampleDist > 0) {
            // Create sample locations in a grid.
            Vector3f bmin = new Vector3f(in);
            Vector3f bmax = new Vector3f(in);
            for (int i = 1; i < nin; ++i) {
                min(bmin, in, i * 3);
                max(bmax, in, i * 3);
            }
            int x0 = (int) Math.floor(bmin.getX() / sampleDist);
            int x1 = (int) Math.ceil(bmax.getX() / sampleDist);
            int z0 = (int) Math.floor(bmin.getZ() / sampleDist);
            int z1 = (int) Math.ceil(bmax.getZ() / sampleDist);
            samples.clear();
            for (int z = z0; z < z1; ++z) {
                for (int x = x0; x < x1; ++x) {
                    Vector3f pt = new Vector3f(
                            x * sampleDist,
                            (bmax.getY() + bmin.getY()) * 0.5f,
                            z * sampleDist
                    );
                    // Make sure the samples are not too close to the edges.
                    if (distToPoly(nin, in, pt) > -sampleDist / 2) {
                        continue;
                    }
                    samples.add(x);
                    samples.add(getHeight(pt.getX(), pt.getY(), pt.getZ(), ics, chf.getCellHeight(), heightSearchRadius, hp));
                    samples.add(z);
                    samples.add(0); // Not added
                }
            }

            // Add the samples starting from the one that has the most
            // error. The procedure stops when all samples are added
            // or when the max error is within threshold.
            int nsamples = samples.getSize() >> 2;
            final Vector3f bestpt = new Vector3f();
            final Vector3f pt = new Vector3f();
            for (int iter = 0; iter < nsamples; ++iter) {
                if (nverts >= MAX_VERTS) {
                    break;
                }

                // Find sample with most error.
                float bestd = 0;
                int besti = -1;
                for (int i = 0; i < nsamples; ++i) {
                    int s = i * 4;
                    if (samples.get(s + 3) != 0) {
                        continue; // skip added.
                    }
                    // The sample location is jittered to get rid of some bad triangulations,
                    // which are caused by symmetrical data from the grid structure.
                    pt.set(
                            samples.get(s) * sampleDist + getJitterX(i) * cs * 0.1f,
                            samples.get(s + 1) * chf.getCellHeight(),
                            samples.get(s + 2) * sampleDist + getJitterY(i) * cs * 0.1f
                    );
                    float d = distToTriMesh(pt, vertices, tris, tris.getSize() >> 2);
                    if (d < 0) {
                        continue; // did not hit the mesh.
                    }
                    if (d > bestd) {
                        bestd = d;
                        besti = i;
                        bestpt.set(pt);
                    }
                }
                // If the max error is within accepted threshold, stop tessellating.
                if (bestd <= sampleMaxError || besti == -1) {
                    break;
                }
                // Mark sample as added.
                samples.set(besti * 4 + 3, 1);
                // Add the new sample point.
                copy(vertices, nverts * 3, bestpt);
                nverts++;

                // Create new triangulation.
                // TO DO: Incremental add instead of full rebuild.
                delaunayHull(nverts, vertices, nhull, hull, tris);
            }
        }

        int ntris = tris.getSize() >> 2;
        if (ntris > MAX_TRIS) {
            IntArrayList subList = tris.subList(0, MAX_TRIS * 4);
            tris.clear();
            tris.addAll(subList);
            throw new RuntimeException(
                    "rcBuildPolyMeshDetail: Shrinking triangle count from " + ntris + " to max " + MAX_TRIS);
        }
        return nverts;
    }

    private static final int[] offset = {0, 0, -1, -1, 0, -1, 1, -1, 1, 0, 1, 1, 0, 1, -1, 1, -1, 0,};

    static void seedArrayWithPolyCenter(Telemetry ctx, CompactHeightfield chf, int[] meshpoly, int poly, int npoly,
                                        int[] vertices, int bs, RecastMeshDetail0.HeightPatch hp, IntArrayList array) {
        // Note: Reads to the compact heightfield are offset by border size (bs)
        // since border size offset is already removed from the polymesh vertices.

        // Find cell closest to a poly vertex
        int startCellX = 0, startCellY = 0, startSpanIndex = -1;
        int dmin = RC_UNSET_HEIGHT;
        for (int j = 0; j < npoly && dmin > 0; ++j) {
            for (int k = 0; k < 9 && dmin > 0; ++k) {
                int ax = vertices[meshpoly[poly + j] * 3] + offset[k * 2];
                int ay = vertices[meshpoly[poly + j] * 3 + 1];
                int az = vertices[meshpoly[poly + j] * 3 + 2] + offset[k * 2 + 1];
                if (ax < hp.xmin || ax >= hp.xmin + hp.width || az < hp.ymin || az >= hp.ymin + hp.height) {
                    continue;
                }

                CompactCell c = chf.cells[(ax + bs) + (az + bs) * chf.getWidth()];
                for (int i = c.getIndex(), ni = c.getIndex() + c.getCount(); i < ni && dmin > 0; ++i) {
                    CompactSpan s = chf.spans[i];
                    int d = Math.abs(ay - s.getY());
                    if (d < dmin) {
                        startCellX = ax;
                        startCellY = az;
                        startSpanIndex = i;
                        dmin = d;
                    }
                }
            }
        }

        // Find center of the polygon
        int pcx = 0, pcy = 0;
        for (int j = 0; j < npoly; ++j) {
            pcx += vertices[meshpoly[poly + j] * 3];
            pcy += vertices[meshpoly[poly + j] * 3 + 2];
        }
        pcx /= npoly;
        pcy /= npoly;

        array.clear();
        array.add(startCellX);
        array.add(startCellY);
        array.add(startSpanIndex);
        int[] dirs = {0, 1, 2, 3};
        Arrays.fill(hp.data, 0, hp.width * hp.height, 0);
        // DFS to move to the center. Note that we need a DFS here and can not just move
        // directly towards the center without recording intermediate nodes, even though the polygons
        // are convex. In very rare we can get stuck due to contour simplification if we do not
        // record nodes.
        int cx = -1, cy = -1, ci = -1;
        while (true) {
            if (array.getSize() < 3) {
                ctx.warn("Walk towards polygon center failed to reach center");
                break;
            }
            ci = array.remove(array.getSize() - 1);
            cy = array.remove(array.getSize() - 1);
            cx = array.remove(array.getSize() - 1);

            // Check if close to center of the polygon.
            if (cx == pcx && cy == pcy) {
                break;
            }
            // If we are already at the correct X-position, prefer direction
            // directly towards the center in the Y-axis; otherwise prefer
            // direction in the X-axis
            int directDir;
            if (cx == pcx) {
                directDir = RecastCommon.INSTANCE.getDirForOffset(0, pcy > cy ? 1 : -1);
            } else {
                directDir = RecastCommon.INSTANCE.getDirForOffset(pcx > cx ? 1 : -1, 0);
            }

            // Push the direct dir last so we start with this on next iteration
            int tmp = dirs[3];
            dirs[3] = dirs[directDir];
            dirs[directDir] = tmp;

            CompactSpan cs = chf.spans[ci];

            for (int i = 0; i < 4; ++i) {
                int dir = dirs[i];
                if (RecastCommon.INSTANCE.getCon(cs, dir) == RC_NOT_CONNECTED) {
                    continue;
                }

                int newX = cx + RecastCommon.INSTANCE.getDirOffsetX(dir);
                int newY = cy + RecastCommon.INSTANCE.getDirOffsetY(dir);

                int hpx = newX - hp.xmin;
                int hpy = newY - hp.ymin;
                if (hpx < 0 || hpx >= hp.width || hpy < 0 || hpy >= hp.height) {
                    continue;
                }
                if (hp.data[hpx + hpy * hp.width] != 0) {
                    continue;
                }

                hp.data[hpx + hpy * hp.width] = 1;

                array.add(newX);
                array.add(newY);
                array.add(chf.cells[(newX + bs) + (newY + bs) * chf.getWidth()].getIndex() + RecastCommon.INSTANCE.getCon(cs, dir));
            }

            tmp = dirs[3];
            dirs[3] = dirs[directDir];
            dirs[directDir] = tmp;

        }

        array.clear();
        // getHeightData seeds are given in coordinates with borders
        array.add(cx + bs);
        array.add(cy + bs);
        array.add(ci);
        Arrays.fill(hp.data, 0, hp.width * hp.height, RC_UNSET_HEIGHT);
        CompactSpan cs = chf.spans[ci];
        hp.data[cx - hp.xmin + (cy - hp.ymin) * hp.width] = cs.getY();
    }

}