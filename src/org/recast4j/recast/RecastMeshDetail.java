package org.recast4j.recast;

import org.joml.Vector3f;
import org.recast4j.IntArrayList;
import org.recast4j.Vectors;

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

    static int getHeight(float fx, float fy, float fz, float ics, float ch, int radius, RecastMeshDetail0.HeightPatch hp) {
        int ix = (int) Math.floor(fx * ics + 0.01f);
        int iz = (int) Math.floor(fz * ics + 0.01f);
        ix = Vectors.INSTANCE.clamp(ix - hp.xmin, 0, hp.width - 1);
        iz = Vectors.INSTANCE.clamp(iz - hp.ymin, 0, hp.height - 1);
        int h = hp.data[ix + iz * hp.width];
        if (h == RC_UNSET_HEIGHT) {
            // Special case when data might be bad.
            // Walk adjacent cells in a spiral up to 'radius', and look
            // for a pixel, which has a valid height.
            int x = 1, z = 0, dx = 1, dz = 0;
            int maxSize = radius * 2 + 1;
            int maxIter = maxSize * maxSize - 1;

            int nextRingIterStart = 8;
            int nextRingIterations = 16;

            float dmin = Float.MAX_VALUE;
            for (int i = 0; i < maxIter; ++i) {
                int nx = ix + x;
                int nz = iz + z;

                if (nx >= 0 && nz >= 0 && nx < hp.width && nz < hp.height) {
                    int nh = hp.data[nx + nz * hp.width];
                    if (nh != RC_UNSET_HEIGHT) {
                        float d = Math.abs(nh * ch - fy);
                        if (d < dmin) {
                            h = nh;
                            dmin = d;
                        }
                    }
                }

                // We are searching in a grid, which looks approximately like this:
                // __________
                // |2 ______ 2|
                // | |1 __ 1| |
                // | | |__| | |
                // | |______| |
                // |__________|
                // We want to find the best height as close to the center cell as possible. This means that
                // if we find a height in one of the neighbor cells to the center, we don't want to
                // expand further out than the 8 neighbors - we want to limit our search to the closest
                // of these "rings", but the best height in the ring.
                // For example, the center is just 1 cell. We checked that at the entrance to the function.
                // The next "ring" contains 8 cells (marked 1 above). Those are all the neighbors to the center cell.
                // The next one again contains 16 cells (marked 2). In general each ring has 8 additional cells, which
                // can be thought of as adding 2 cells around the "center" of each side when we expand the ring.
                // Here we detect if we are about to enter the next ring, and if we are and we have found
                // a height, we abort the search.
                if (i + 1 == nextRingIterStart) {
                    if (h != RC_UNSET_HEIGHT) {
                        break;
                    }

                    nextRingIterStart += nextRingIterations;
                    nextRingIterations += 8;
                }

                if ((x == z) || ((x < 0) && (x == -z)) || ((x > 0) && (x == 1 - z))) {
                    int tmp = dx;
                    dx = -dz;
                    dz = tmp;
                }
                x += dx;
                z += dz;
            }
        }
        return h;
    }

    static int completeFacet(float[] pts, int npts, IntArrayList edges, int maxEdges, int nfaces, int e) {
        float EPS = 1e-5f;

        int edge = e * 4;

        // Cache s and t.
        int s, t;
        if (edges.get(edge + 2) == EV_UNDEF) {
            s = edges.get(edge);
            t = edges.get(edge + 1);
        } else if (edges.get(edge + 3) == EV_UNDEF) {
            s = edges.get(edge + 1);
            t = edges.get(edge);
        } else {
            // Edge already completed.
            return nfaces;
        }

        // Find best point on left of edge.
        int pt = npts;
        Vector3f c = new Vector3f();
        float r = -1f;
        for (int u = 0; u < npts; ++u) {
            if (u != s && u != t && vcross2(pts, s * 3, t * 3, u * 3) > EPS) {
                if (r < 0) {
                    // The circle is not updated yet, do it now.
                    pt = u;
                    r = circumcircle(pts, s * 3, t * 3, u * 3, c);
                } else {
                    float d = vdist2(c, pts, u * 3);
                    float tol = 0.001f;
                    if (!(d > r * (1 + tol))) {
                        if (d < r * (1 - tol)) {
                            // Inside safe circumcircle, update circle.
                            pt = u;
                            r = circumcircle(pts, s * 3, t * 3, u * 3, c);
                        } else {
                            // Inside epsilon circumcircle, do extra tests to make sure the edge is valid.
                            // s-u and t-u cannot overlap with s-pt nor t-pt if they exist.
                            if (doesNotOverlapEdges(pts, edges, s, u) && doesNotOverlapEdges(pts, edges, t, u)) {
                                // Edge is valid.
                                pt = u;
                                r = circumcircle(pts, s * 3, t * 3, u * 3, c);
                            }
                        }
                    } // else Outside current circumcircle, skip.
                }
            }
        }

        // Add new triangle or update edge info if s-t is on hull.
        if (pt < npts) {
            // Update face information of edge being completed.
            updateLeftFace(edges, e * 4, s, t, nfaces);

            // Add new edge or update face info of old edge.
            e = findEdge(edges, pt, s);
            if (e == EV_UNDEF) {
                addEdge(edges, maxEdges, pt, s, nfaces, EV_UNDEF);
            } else {
                updateLeftFace(edges, e * 4, pt, s, nfaces);
            }

            // Add new edge or update face info of old edge.
            e = findEdge(edges, t, pt);
            if (e == EV_UNDEF) {
                addEdge(edges, maxEdges, t, pt, nfaces, EV_UNDEF);
            } else {
                updateLeftFace(edges, e * 4, t, pt, nfaces);
            }

            nfaces++;
        } else {
            updateLeftFace(edges, e * 4, s, t, EV_HULL);
        }
        return nfaces;
    }

    static void delaunayHull(int npts, float[] pts, int nhull, int[] hull, IntArrayList tris) {
        int nfaces = 0;
        int maxEdges = npts * 10;
        IntArrayList edges = new IntArrayList(64);
        for (int i = 0, j = nhull - 1; i < nhull; j = i++) {
            addEdge(edges, maxEdges, hull[j], hull[i], EV_HULL, EV_UNDEF);
        }
        int currentEdge = 0;
        while (currentEdge < edges.getSize() >> 2) {
            if (edges.get(currentEdge * 4 + 2) == EV_UNDEF) {
                nfaces = completeFacet(pts, npts, edges, maxEdges, nfaces, currentEdge);
            }
            if (edges.get(currentEdge * 4 + 3) == EV_UNDEF) {
                nfaces = completeFacet(pts, npts, edges, maxEdges, nfaces, currentEdge);
            }
            currentEdge++;
        }
        // Create tris
        tris.clear();
        for (int i = 0; i < nfaces * 4; ++i) {
            tris.add(-1);
        }

        for (int e = 0, l = edges.getSize(); e < l; e += 4) {
            if (edges.get(e + 3) >= 0) {
                // Left face
                int t = edges.get(e + 3) * 4;
                if (tris.get(t) == -1) {
                    tris.set(t, edges.get(e));
                    tris.set(t + 1, edges.get(e + 1));
                } else if (tris.get(t) == edges.get(e + 1)) {
                    tris.set(t + 2, edges.get(e));
                } else if (tris.get(t + 1) == edges.get(e)) {
                    tris.set(t + 2, edges.get(e + 1));
                }
            }
            if (edges.get(e + 2) >= 0) {
                // Right
                int t = edges.get(e + 2) * 4;
                if (tris.get(t) == -1) {
                    tris.set(t, edges.get(e + 1));
                    tris.set(t + 1, edges.get(e));
                } else if (tris.get(t) == edges.get(e)) {
                    tris.set(t + 2, edges.get(e + 1));
                } else if (tris.get(t + 1) == edges.get(e + 1)) {
                    tris.set(t + 2, edges.get(e));
                }
            }
        }

        int size = tris.getSize();
        for (int t = 0; t < size; t += 4) {
            if (tris.get(t) == -1 || tris.get(t + 1) == -1 || tris.get(t + 2) == -1) {
                System.err.println("Dangling! " + tris.get(t) + " " + tris.get(t + 1) + "  " + tris.get(t + 2));
                tris.set(t, tris.get(size - 4));
                tris.set(t + 1, tris.get(size - 3));
                tris.set(t + 2, tris.get(size - 2));
                tris.set(t + 3, tris.get(size - 1));
                tris.setSize(size - 4);
                size -= 4;
                t -= 4;
            }
        }
    }

    static void triangulateHull(float[] vertices, int nhull, int[] hull, int nin, IntArrayList tris) {
        int start = 0, left = 1, right = nhull - 1;

        // Start from an ear with the shortest perimeter.
        // This tends to favor well-formed triangles as starting point.
        float dmin = Float.MAX_VALUE;
        for (int i = 0; i < nhull; i++) {
            if (hull[i] >= nin) {
                continue; // Ears are triangles with original vertices as middle vertex while others are actually line
            }
            // segments on edges
            int pi = RecastMesh.INSTANCE.prev(i, nhull);
            int ni = RecastMesh.INSTANCE.next(i, nhull);
            int pv = hull[pi] * 3;
            int cv = hull[i] * 3;
            int nv = hull[ni] * 3;
            float d = vdist2(vertices, pv, cv) + vdist2(vertices, cv, nv) + vdist2(vertices, nv, pv);
            if (d < dmin) {
                start = i;
                left = ni;
                right = pi;
                dmin = d;
            }
        }

        // Add first triangle
        tris.add(hull[start]);
        tris.add(hull[left]);
        tris.add(hull[right]);
        tris.add(0);

        // Triangulate the polygon by moving left or right,
        // depending on which triangle has shorter perimeter.
        // This heuristic was chose emprically, since it seems
        // handle tesselated straight edges well.
        while (RecastMesh.INSTANCE.next(left, nhull) != right) {
            // Check to see if se should advance left or right.
            int nleft = RecastMesh.INSTANCE.next(left, nhull);
            int nright = RecastMesh.INSTANCE.prev(right, nhull);

            int cvleft = hull[left] * 3;
            int nvleft = hull[nleft] * 3;
            int cvright = hull[right] * 3;
            int nvright = hull[nright] * 3;
            float dleft = vdist2(vertices, cvleft, nvleft) + vdist2(vertices, nvleft, cvright);
            float dright = vdist2(vertices, cvright, nvright) + vdist2(vertices, cvleft, nvright);

            tris.add(hull[left]);
            if (dleft < dright) {
                tris.add(hull[nleft]);
                tris.add(hull[right]);
                tris.add(0);
                left = nleft;
            } else {
                tris.add(hull[nright]);
                tris.add(hull[right]);
                tris.add(0);
                right = nright;
            }
        }
    }

    static int buildPolyDetail(float[] in, int nin, float sampleDist, float sampleMaxError,
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

    static final int RETRACT_SIZE = 256;

    static void push3(IntArrayList queue, int v1, int v2, int v3) {
        queue.add(v1);
        queue.add(v2);
        queue.add(v3);
    }

    static void getHeightData(Telemetry ctx, CompactHeightfield chf, int[] meshpolys, int poly, int npoly, int[] vertices,
                              int bs, RecastMeshDetail0.HeightPatch hp, int region) {
        // Note: Reads to the compact heightfield are offset by border size (bs)
        // since border size offset is already removed from the polymesh vertices.

        IntArrayList queue = new IntArrayList(512);
        Arrays.fill(hp.data, 0, hp.width * hp.height, RC_UNSET_HEIGHT);

        boolean empty = true;

        // We cannot sample from this poly if it was created from polys
        // of different regions. If it was then it could potentially be overlapping
        // with polys of that region and the heights sampled here could be wrong.
        if (region != RecastConstants.INSTANCE.getRC_MULTIPLE_REGS()) {
            // Copy the height from the same region, and mark region borders
            // as seed points to fill the rest.
            for (int hy = 0; hy < hp.height; hy++) {
                int y = hp.ymin + hy + bs;
                for (int hx = 0; hx < hp.width; hx++) {
                    int x = hp.xmin + hx + bs;
                    CompactCell c = chf.cells[x + y * chf.getWidth()];
                    for (int i = c.getIndex(), ni = c.getIndex() + c.getCount(); i < ni; ++i) {
                        CompactSpan s = chf.spans[i];
                        if (s.getReg() == region) {
                            // Store height
                            hp.data[hx + hy * hp.width] = s.getY();
                            empty = false;
                            // If any of the neighbours is not in same region,
                            // add the current location as flood fill start
                            boolean border = false;
                            for (int dir = 0; dir < 4; ++dir) {
                                if (RecastCommon.INSTANCE.getCon(s, dir) != RC_NOT_CONNECTED) {
                                    int ax = x + RecastCommon.INSTANCE.getDirOffsetX(dir);
                                    int ay = y + RecastCommon.INSTANCE.getDirOffsetY(dir);
                                    int ai = chf.cells[ax + ay * chf.getWidth()].getIndex() + RecastCommon.INSTANCE.getCon(s, dir);
                                    CompactSpan as = chf.spans[ai];
                                    if (as.getReg() != region) {
                                        border = true;
                                        break;
                                    }
                                }
                            }
                            if (border) {
                                push3(queue, x, y, i);
                            }
                            break;
                        }
                    }
                }
            }
        }

        // if the polygon does not contain any points from the current region (rare, but happens)
        // or if it could potentially be overlapping polygons of the same region,
        // then use the center as the seed point.
        if (empty) {
            seedArrayWithPolyCenter(ctx, chf, meshpolys, poly, npoly, vertices, bs, hp, queue);
        }

        int head = 0;

        // We assume the seed is centered in the polygon, so a BFS to collect
        // height data will ensure we do not move onto overlapping polygons and
        // sample wrong heights.
        while (head * 3 < queue.getSize()) {
            int cx = queue.get(head * 3);
            int cy = queue.get(head * 3 + 1);
            int ci = queue.get(head * 3 + 2);
            head++;
            if (head >= RETRACT_SIZE) {
                head = 0;
                queue = queue.subList(RETRACT_SIZE * 3, queue.getSize());
            }

            CompactSpan cs = chf.spans[ci];
            for (int dir = 0; dir < 4; ++dir) {
                if (RecastCommon.INSTANCE.getCon(cs, dir) == RC_NOT_CONNECTED) {
                    continue;
                }

                int ax = cx + RecastCommon.INSTANCE.getDirOffsetX(dir);
                int ay = cy + RecastCommon.INSTANCE.getDirOffsetY(dir);
                int hx = ax - hp.xmin - bs;
                int hy = ay - hp.ymin - bs;

                if (hx < 0 || hx >= hp.width || hy < 0 || hy >= hp.height) {
                    continue;
                }

                if (hp.data[hx + hy * hp.width] != RC_UNSET_HEIGHT) {
                    continue;
                }

                int ai = chf.cells[ax + ay * chf.getWidth()].getIndex() + RecastCommon.INSTANCE.getCon(cs, dir);
                CompactSpan as = chf.spans[ai];

                hp.data[hx + hy * hp.width] = as.getY();
                push3(queue, ax, ay, ai);
            }
        }
    }

    /// @par
    ///
    /// See the #rcConfig documentation for more information on the configuration parameters.
    ///
    /// @see rcAllocPolyMeshDetail, rcPolyMesh, rcCompactHeightfield, rcPolyMeshDetail, rcConfig
    public static PolyMeshDetail buildPolyMeshDetail(Telemetry ctx, PolyMesh mesh, CompactHeightfield chf,
                                                     float sampleDist, float sampleMaxError) {

        ctx.startTimer("POLYMESHDETAIL");
        if (mesh.getNumVertices() == 0 || mesh.getNumPolygons() == 0) {
            return null;
        }

        int nvp = mesh.getMaxVerticesPerPolygon();
        float cs = mesh.getCellSize();
        float ch = mesh.getCellHeight();
        Vector3f orig = mesh.getBmin();
        int borderSize = mesh.getBorderSize();
        int heightSearchRadius = (int) Math.max(1, Math.ceil(mesh.getMaxEdgeError()));

        IntArrayList tris = new IntArrayList(512);
        float[] vertices = new float[256 * 3];
        int nPolyVerts = 0;
        int maxhw = 0, maxhh = 0;

        int[] bounds = new int[mesh.getNumPolygons() * 4];
        float[] poly = new float[nvp * 3];

        // Find max size for a polygon area.
        for (int i = 0; i < mesh.getNumPolygons(); ++i) {
            int p = i * nvp * 2;
            bounds[i * 4] = chf.getWidth();
            bounds[i * 4 + 1] = 0;
            bounds[i * 4 + 2] = chf.getHeight();
            bounds[i * 4 + 3] = 0;
            int[] meshVerts = mesh.getVertices();
            int[] meshPolys = mesh.getPolygons();
            final int nullIdx = RecastConstants.INSTANCE.getRC_MESH_NULL_IDX();
            for (int j = 0; j < nvp; ++j) {
                if (meshPolys[p + j] == nullIdx) {
                    break;
                }
                int v = meshPolys[p + j] * 3;
                bounds[i * 4] = Math.min(bounds[i * 4], meshVerts[v]);
                bounds[i * 4 + 1] = Math.max(bounds[i * 4 + 1], meshVerts[v]);
                bounds[i * 4 + 2] = Math.min(bounds[i * 4 + 2], meshVerts[v + 2]);
                bounds[i * 4 + 3] = Math.max(bounds[i * 4 + 3], meshVerts[v + 2]);
                nPolyVerts++;
            }
            bounds[i * 4] = Math.max(0, bounds[i * 4] - 1);
            bounds[i * 4 + 1] = Math.min(chf.getWidth(), bounds[i * 4 + 1] + 1);
            bounds[i * 4 + 2] = Math.max(0, bounds[i * 4 + 2] - 1);
            bounds[i * 4 + 3] = Math.min(chf.getHeight(), bounds[i * 4 + 3] + 1);
            if (bounds[i * 4] >= bounds[i * 4 + 1] || bounds[i * 4 + 2] >= bounds[i * 4 + 3]) {
                continue;
            }
            maxhw = Math.max(maxhw, bounds[i * 4 + 1] - bounds[i * 4]);
            maxhh = Math.max(maxhh, bounds[i * 4 + 3] - bounds[i * 4 + 2]);
        }

        RecastMeshDetail0.HeightPatch hp = new RecastMeshDetail0.HeightPatch(new int[maxhw * maxhh]);

        int vcap = nPolyVerts + nPolyVerts / 2;
        int tcap = vcap * 2;
        PolyMeshDetail dmesh = new PolyMeshDetail(
                new int[mesh.getNumPolygons() * 4],
                new float[vcap * 3],
                new int[tcap * 4]
        );
        dmesh.setNumSubMeshes(mesh.getNumPolygons());

        int[] meshPolygons = mesh.getPolygons();
        for (int i = 0; i < mesh.getNumPolygons(); ++i) {
            int p = i * nvp * 2;

            // Store polygon vertices for processing.
            int npoly = 0;
            int[] meshVerts = mesh.getVertices();
            final int nullIdx = RecastConstants.INSTANCE.getRC_MESH_NULL_IDX();
            for (int j = 0; j < nvp && meshPolygons[p + j] != nullIdx; ++j) {
                int v = meshPolygons[p + j] * 3;
                poly[j * 3] = meshVerts[v] * cs;
                poly[j * 3 + 1] = meshVerts[v + 1] * ch;
                poly[j * 3 + 2] = meshVerts[v + 2] * cs;
                npoly++;
            }

            // Get the height data from the area of the polygon.
            hp.xmin = bounds[i * 4];
            hp.ymin = bounds[i * 4 + 2];
            hp.width = bounds[i * 4 + 1] - bounds[i * 4];
            hp.height = bounds[i * 4 + 3] - bounds[i * 4 + 2];
            getHeightData(ctx, chf, meshPolygons, p, npoly, mesh.getVertices(), borderSize, hp, mesh.getRegionIds()[i]);

            // Build detail mesh.
            int nverts = buildPolyDetail(poly, npoly, sampleDist, sampleMaxError, heightSearchRadius, chf, hp,
                    vertices, tris);

            // Move detail vertices to world space.
            for (int j = 0; j < nverts; ++j) {
                vertices[j * 3] += orig.getX();
                vertices[j * 3 + 1] += orig.getY() + chf.getCellHeight(); // Is this offset necessary? See
                // https://groups.google.com/d/msg/recastnavigation/UQFN6BGCcV0/-1Ny4koOBpkJ
                vertices[j * 3 + 2] += orig.getZ();
            }
            // Offset poly too, will be used to flag checking.
            for (int j = 0; j < npoly; ++j) {
                poly[j * 3] += orig.getX();
                poly[j * 3 + 1] += orig.getY();
                poly[j * 3 + 2] += orig.getZ();
            }

            // Store detail submesh.
            int ntris = tris.getSize() >> 2;

            int[] dmeshSubMeshes = dmesh.getSubMeshes();
            dmeshSubMeshes[i * 4] = dmesh.getNumVertices();
            dmeshSubMeshes[i * 4 + 1] = nverts;
            dmeshSubMeshes[i * 4 + 2] = dmesh.getNumTriangles();
            dmeshSubMeshes[i * 4 + 3] = ntris;

            int dMeshNverts = dmesh.getNumVertices();
            // Store vertices, allocate more memory if necessary.
            if (dMeshNverts + nverts > vcap) {
                vcap += Math.max((dMeshNverts + nverts - vcap + 255) >> 8, 0) << 8;
                float[] newv = new float[vcap * 3];
                if (dMeshNverts != 0) {
                    System.arraycopy(dmesh.getVertices(), 0, newv, 0, 3 * dMeshNverts);
                }
                dmesh.setVertices(newv);
            }
            float[] dMeshVerts = dmesh.getVertices();
            System.arraycopy(vertices, 0, dMeshVerts, dMeshNverts * 3, nverts * 3);
            dmesh.setNumVertices(dMeshNverts + nverts);

            // Store triangles, allocate more memory if necessary.
            if (dmesh.getNumTriangles() + ntris > tcap) {
                tcap += Math.max((dmesh.getNumTriangles() + ntris - tcap + 255) >> 8, 0) << 8;
                int[] newt = new int[tcap * 4];
                if (dmesh.getNumTriangles() != 0) {
                    System.arraycopy(dmesh.getTriangles(), 0, newt, 0, 4 * dmesh.getNumTriangles());
                }
                dmesh.setTriangles(newt);
            }
            int[] dmeshTris = dmesh.getTriangles();
            int l = dmesh.getNumTriangles() << 2;
            for (int j = 0; j < ntris; ++j) {
                int t = j * 4;
                dmeshTris[l++] = tris.get(t);
                dmeshTris[l++] = tris.get(t + 1);
                dmeshTris[l++] = tris.get(t + 2);
                dmeshTris[l++] = getTriFlags(vertices, tris.get(t) * 3, tris.get(t + 1) * 3, tris.get(t + 2) * 3, poly, npoly);
            }
            dmesh.setNumTriangles(l >> 2);
        }

        ctx.stopTimer("POLYMESHDETAIL");
        return dmesh;

    }

}