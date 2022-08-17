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

import kotlin.Pair;
import org.joml.Vector3f;
import org.joml.Vector3i;
import org.recast4j.FloatArrayList;
import org.recast4j.LongArrayList;

import java.util.*;

import static org.joml.Math.clamp;
import static org.recast4j.Vectors.*;
import static org.recast4j.detour.Node.CLOSED;
import static org.recast4j.detour.Node.OPEN;

public class NavMeshQuery {

    /**
     * Use raycasts during pathfind to "shortcut" (raycast still consider costs) Options for
     * NavMeshQuery::initSlicedFindPath and updateSlicedFindPath
     */
    public static final int DT_FINDPATH_ANY_ANGLE = 0x02;

    /**
     * Raycast should calculate movement cost along the ray and fill RaycastHit::cost
     */
    public static final int DT_RAYCAST_USE_COSTS = 0x01;

    /// Vertex flags returned by findStraightPath.
    /**
     * The vertex is the start position in the path.
     */
    public static final int DT_STRAIGHTPATH_START = 0x01;
    /**
     * The vertex is the end position in the path.
     */
    public static final int DT_STRAIGHTPATH_END = 0x02;
    /**
     * The vertex is the start of an off-mesh connection.
     */
    public static final int DT_STRAIGHTPATH_OFFMESH_CONNECTION = 0x04;

    /// Options for findStraightPath.
    public static final int DT_STRAIGHTPATH_AREA_CROSSINGS = 0x01; /// < Add a vertex at every polygon edge crossing
    /// where area changes.
    public static final int DT_STRAIGHTPATH_ALL_CROSSINGS = 0x02; /// < Add a vertex at every polygon edge crossing.

    protected final NavMesh nav;
    public final NodePool nodePool = new NodePool();
    protected final PriorityQueue<Node> openList = new PriorityQueue<>((n1, n2) -> Float.compare(n1.totalCost, n2.totalCost));
    protected QueryData queryData; /// < Sliced query state.

    public NavMeshQuery(NavMesh nav) {
        this.nav = nav;
    }

    /**
     * Returns random location on navmesh. Polygons are chosen weighted by area. The search runs in linear related to
     * number of polygon.
     *
     * @param filter The polygon filter to apply to the query.
     * @param random Function returning a random number [0..1).
     * @return Random location
     */
    @SuppressWarnings("unused")
    public Result<FindRandomPointResult> findRandomPoint(QueryFilter filter, Random random) {
        // Randomly pick one tile. Assume that all tiles cover roughly the same area.
        if (Objects.isNull(filter) || Objects.isNull(random)) {
            return Result.invalidParam();
        }
        MeshTile tile = null;
        float tsum = 0f;
        for (int i = 0; i < nav.getMaxTiles(); i++) {
            MeshTile t = nav.getTile(i);
            if (t == null || t.data == null || t.data.header == null) {
                continue;
            }

            // Choose random tile using reservoi sampling.
            float area = 1f; // Could be tile area too.
            tsum += area;
            float u = random.nextFloat();
            if (u * tsum <= area) {
                tile = t;
            }
        }
        if (tile == null) {
            return Result.invalidParam("Tile not found");
        }

        // Randomly pick one polygon weighted by polygon area.
        Poly poly = null;
        long polyRef = 0;
        long base = nav.getPolyRefBase(tile);

        float areaSum = 0f;
        for (int i = 0; i < tile.data.header.polyCount; ++i) {
            Poly p = tile.data.polygons[i];
            // Do not return off-mesh connection polygons.
            if (p.getType() != Poly.DT_POLYTYPE_GROUND) {
                continue;
            }
            // Must pass filter
            long ref = base | i;
            if (!filter.passFilter(ref, tile, p)) {
                continue;
            }

            // Calc area of the polygon.
            float polyArea = 0f;
            for (int j = 2; j < p.vertCount; ++j) {
                int va = p.vertices[0] * 3;
                int vb = p.vertices[j - 1] * 3;
                int vc = p.vertices[j] * 3;
                polyArea += triArea2D(tile.data.vertices, va, vb, vc);
            }

            // Choose random polygon weighted by area, using reservoi sampling.
            areaSum += polyArea;
            float u = random.nextFloat();
            if (u * areaSum <= polyArea) {
                poly = p;
                polyRef = ref;
            }
        }

        if (poly == null) {
            return Result.invalidParam("Poly not found");
        }

        // Randomly pick point on polygon.
        float[] vertices = new float[3 * nav.maxVerticesPerPoly];
        float[] areas = new float[nav.maxVerticesPerPoly];
        System.arraycopy(tile.data.vertices, poly.vertices[0] * 3, vertices, 0, 3);
        for (int j = 1; j < poly.vertCount; ++j) {
            System.arraycopy(tile.data.vertices, poly.vertices[j] * 3, vertices, j * 3, 3);
        }

        float s = random.nextFloat();
        float t = random.nextFloat();

        Vector3f pt = randomPointInConvexPoly(vertices, poly.vertCount, areas, s, t);
        FindRandomPointResult result = new FindRandomPointResult(polyRef, pt);
        float pheight = getPolyHeight(polyRef, pt);
        if (!Float.isFinite(pheight)) return Result.of(Status.FAILURE, result);
        pt.y = pheight;
        return Result.success(result);
    }

    /**
     * Returns random location on navmesh within the reach of specified location. Polygons are chosen weighted by area.
     * The search runs in linear related to number of polygon. The location is not exactly constrained by the circle,
     * but it limits the visited polygons.
     *
     * @param startRef  The reference id of the polygon where the search starts.
     * @param centerPos The center of the search circle. [(x, y, z)]
     * @param filter    The polygon filter to apply to the query.
     * @param random    Function returning a random number [0..1).
     * @return Random location
     */
    @SuppressWarnings("unused")
    public Result<FindRandomPointResult> findRandomPointAroundCircle(long startRef, Vector3f centerPos, float maxRadius, QueryFilter filter, Random random) {
        return findRandomPointAroundCircle(startRef, centerPos, maxRadius, filter, random, PolygonByCircleConstraint.noop());
    }

    /**
     * Returns random location on navmesh within the reach of specified location. Polygons are chosen weighted by area.
     * The search runs in linear related to number of polygon. The location is strictly constrained by the circle.
     *
     * @param startRef  The reference id of the polygon where the search starts.
     * @param centerPos The center of the search circle. [(x, y, z)]
     * @param filter    The polygon filter to apply to the query.
     * @param frand     Function returning a random number [0..1).
     * @return Random location
     */
    @SuppressWarnings("unused")
    public Result<FindRandomPointResult> findRandomPointWithinCircle(long startRef, Vector3f centerPos, float maxRadius, QueryFilter filter, Random frand) {
        return findRandomPointAroundCircle(startRef, centerPos, maxRadius, filter, frand, PolygonByCircleConstraint.strict());
    }

    public Result<FindRandomPointResult> findRandomPointAroundCircle(long startRef, Vector3f centerPos, float maxRadius,
                                                                     QueryFilter filter, Random frand, PolygonByCircleConstraint constraint) {

        // Validate input
        if (!nav.isValidPolyRef(startRef) || Objects.isNull(centerPos) || !isFinite(centerPos) || maxRadius < 0
                || !Float.isFinite(maxRadius) || Objects.isNull(filter) || Objects.isNull(frand)) {
            return Result.invalidParam();
        }

        Pair<MeshTile, Poly> tileAndPoly = nav.getTileAndPolyByRefUnsafe(startRef);
        MeshTile startTile = tileAndPoly.getFirst();
        Poly startPoly = tileAndPoly.getSecond();
        if (!filter.passFilter(startRef, startTile, startPoly)) {
            return Result.invalidParam("Invalid start ref");
        }

        nodePool.clear();
        openList.clear();

        Node startNode = nodePool.getNode(startRef);
        copy(startNode.pos, centerPos);
        startNode.parentIndex = 0;
        startNode.cost = 0;
        startNode.totalCost = 0;
        startNode.polygonRef = startRef;
        startNode.flags = OPEN;
        openList.offer(startNode);

        float radiusSqr = maxRadius * maxRadius;
        float areaSum = 0f;

        Poly randomPoly = null;
        long randomPolyRef = 0;
        float[] randomPolyVertices = null;

        while (!openList.isEmpty()) {
            Node bestNode = openList.poll();
            bestNode.flags &= ~OPEN;
            bestNode.flags |= CLOSED;
            // Get poly and tile.
            // The API input has been cheked already, skip checking internal data.
            long bestRef = bestNode.polygonRef;
            Pair<MeshTile, Poly> bestTilePoly = nav.getTileAndPolyByRefUnsafe(bestRef);
            MeshTile bestTile = bestTilePoly.getFirst();
            Poly bestPoly = bestTilePoly.getSecond();

            // Place random locations on on ground.
            if (bestPoly.getType() == Poly.DT_POLYTYPE_GROUND) {
                // Calc area of the polygon.
                float polyArea = 0f;
                float[] polyVertices = new float[bestPoly.vertCount * 3];
                for (int j = 0; j < bestPoly.vertCount; ++j) {
                    System.arraycopy(bestTile.data.vertices, bestPoly.vertices[j] * 3, polyVertices, j * 3, 3);
                }
                float[] constrainedVertices = constraint.apply(polyVertices, centerPos, maxRadius);
                if (constrainedVertices != null) {
                    int vertCount = constrainedVertices.length / 3;
                    for (int j = 2; j < vertCount; ++j) {
                        int va = 0;
                        int vb = (j - 1) * 3;
                        int vc = j * 3;
                        polyArea += triArea2D(constrainedVertices, va, vb, vc);
                    }
                    // Choose random polygon weighted by area, using reservoi sampling.
                    areaSum += polyArea;
                    float u = frand.nextFloat();
                    if (u * areaSum <= polyArea) {
                        randomPoly = bestPoly;
                        randomPolyRef = bestRef;
                        randomPolyVertices = constrainedVertices;
                    }
                }
            }

            // Get parent poly and tile.
            long parentRef = 0;
            if (bestNode.parentIndex != 0) {
                parentRef = nodePool.getNodeAtIdx(bestNode.parentIndex).polygonRef;
            }

            for (int i = bestTile.polyLinks[bestPoly.index]; i != NavMesh.DT_NULL_LINK; i = bestTile.links.get(i).indexOfNextLink) {
                Link link = bestTile.links.get(i);
                long neighbourRef = link.neighborRef;
                // Skip invalid neighbours and do not follow back to parent.
                if (neighbourRef == 0 || neighbourRef == parentRef) {
                    continue;
                }

                // Expand to neighbour
                Pair<MeshTile, Poly> neighbourTilePoly = nav.getTileAndPolyByRefUnsafe(neighbourRef);
                MeshTile neighbourTile = neighbourTilePoly.getFirst();
                Poly neighbourPoly = neighbourTilePoly.getSecond();

                // Do not advance if the polygon is excluded by the filter.
                if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    continue;
                }

                // Find edge and calc distance to the edge.
                Result<PortalResult> portalpoints = getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef,
                        neighbourPoly, neighbourTile, 0, 0);
                if (portalpoints.failed()) {
                    continue;
                }
                Vector3f va = portalpoints.result.left;
                Vector3f vb = portalpoints.result.right;

                // If the circle is not touching the next polygon, skip it.
                Pair<Float, Float> distseg = distancePtSegSqr2D(centerPos, va, vb);
                float distSqr = distseg.getFirst();
                if (distSqr > radiusSqr) {
                    continue;
                }

                Node neighbourNode = nodePool.getNode(neighbourRef);

                if ((neighbourNode.flags & Node.CLOSED) != 0) {
                    continue;
                }

                // Cost
                if (neighbourNode.flags == 0) {
                    neighbourNode.pos = lerp(va, vb, 0.5f);
                }

                float total = bestNode.totalCost + bestNode.pos.distance(neighbourNode.pos);

                // The node is already in open list, and the new result is worse, skip.
                if ((neighbourNode.flags & Node.OPEN) != 0 && total >= neighbourNode.totalCost) {
                    continue;
                }

                neighbourNode.polygonRef = neighbourRef;
                neighbourNode.flags = (neighbourNode.flags & ~Node.CLOSED);
                neighbourNode.parentIndex = nodePool.getNodeIdx(bestNode);
                neighbourNode.totalCost = total;

                if ((neighbourNode.flags & Node.OPEN) != 0) {
                    openList.remove(neighbourNode);
                    openList.offer(neighbourNode);
                } else {
                    neighbourNode.flags = Node.OPEN;
                    openList.offer(neighbourNode);
                }
            }
        }

        if (randomPoly == null) {
            return Result.failure();
        }

        // Randomly pick point on polygon.
        float s = frand.nextFloat();
        float t = frand.nextFloat();

        float[] areas = new float[randomPolyVertices.length / 3];
        Vector3f pt = randomPointInConvexPoly(randomPolyVertices, randomPolyVertices.length / 3, areas, s, t);
        FindRandomPointResult result = new FindRandomPointResult(randomPolyRef, pt);
        float pheight = getPolyHeight(randomPolyRef, pt);
        if (!Float.isFinite(pheight)) return Result.of(Status.FAILURE, result);
        pt.y = pheight;
        return Result.success(result);
    }

    /**
     * Uses the detail polygons to find the surface height. (Most accurate.)
     *
     * @param pos does not have to be within the bounds of the polygon or navigation mesh.
     *
     * See closestPointOnPolyBoundary() for a limited but faster option.
     * Finds the closest point on the specified polygon.
     * */
    public Result<ClosestPointOnPolyResult> closestPointOnPoly(long ref, Vector3f pos) {
        if (!nav.isValidPolyRef(ref) || Objects.isNull(pos) || !isFinite(pos)) {
            return Result.invalidParam();
        }
        return Result.success(nav.closestPointOnPoly(ref, pos));
    }

    /// @par
    ///
    /// Much faster than closestPointOnPoly().
    ///
    /// If the provided position lies within the polygon's xz-bounds (above or below),
    /// then @p pos and @p closest will be equal.
    ///
    /// The height of @p closest will be the polygon boundary. The height detail is not used.
    ///
    /// @p pos does not have to be within the bounds of the polygon or the navigation mesh.
    ///
    /// Returns a point on the boundary closest to the source point if the source point is outside the
    /// polygon's xz-bounds.
    /// @param[in] ref The reference id to the polygon.
    /// @param[in] pos The position to check.
    /// @param[out] closest The closest point.
    /// @returns The status flags for the query.
    public Result<Vector3f> closestPointOnPolyBoundary(long ref, Vector3f pos) {

        Result<Pair<MeshTile, Poly>> tileAndPoly = nav.getTileAndPolyByRef(ref);
        if (tileAndPoly.failed()) {
            return Result.of(tileAndPoly.status, tileAndPoly.message);
        }
        MeshTile tile = tileAndPoly.result.getFirst();
        Poly poly = tileAndPoly.result.getSecond();
        if (tile == null) {
            return Result.invalidParam("Invalid tile");
        }

        if (Objects.isNull(pos) || !isFinite(pos)) {
            return Result.invalidParam();
        }
        // Collect vertices.
        float[] vertices = new float[nav.maxVerticesPerPoly * 3];
        float[] edged = new float[nav.maxVerticesPerPoly];
        float[] edget = new float[nav.maxVerticesPerPoly];
        int nv = poly.vertCount;
        for (int i = 0; i < nv; ++i) {
            System.arraycopy(tile.data.vertices, poly.vertices[i] * 3, vertices, i * 3, 3);
        }

        Vector3f closest;
        if (distancePtPolyEdgesSqr(pos, vertices, nv, edged, edget)) {
            closest = copy(pos);
        } else {
            // Point is outside the polygon, dtClamp to nearest edge.
            float dmin = edged[0];
            int imin = 0;
            for (int i = 1; i < nv; ++i) {
                if (edged[i] < dmin) {
                    dmin = edged[i];
                    imin = i;
                }
            }
            int va = imin * 3;
            int vb = ((imin + 1) % nv) * 3;
            closest = lerp(vertices, va, vb, edget[imin]);
        }
        return Result.success(closest);
    }

    /**
     * Gets the height of the polygon at the provided position using the height detail. (Most accurate.)
     * Will return NaN/+/-Inf if the provided position is outside the xz-bounds of the polygon.
     * @param ref The reference id of the polygon.
     * @param pos A position within the xz-bounds of the polygon. [(x, y, z)]
     * @return The height at the surface of the polygon. or !finite for errors
     * */
    public float getPolyHeight(long ref, Vector3f pos) {

        Result<Pair<MeshTile, Poly>> tileAndPoly = nav.getTileAndPolyByRef(ref);
        if (tileAndPoly.failed()) {
            // return Result.of(tileAndPoly.status, tileAndPoly.message);
            return Float.NEGATIVE_INFINITY;
        }
        MeshTile tile = tileAndPoly.result.getFirst();
        Poly poly = tileAndPoly.result.getSecond();

        if (Objects.isNull(pos) || !isFinite2D(pos)) {
            return Float.POSITIVE_INFINITY;
        }

        // We used to return success for offmesh connections, but the
        // getPolyHeight in DetourNavMesh does not do this, so special
        // case it here.
        if (poly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
            Vector3f v0 = new Vector3f(), v1 = new Vector3f();
            copy(v0, tile.data.vertices, poly.vertices[0] * 3);
            copy(v1, tile.data.vertices, poly.vertices[1] * 3);
            Pair<Float, Float> dt = distancePtSegSqr2D(pos, v0, v1);
            return v0.y + (v1.y - v0.y) * dt.getSecond();
        }
        return nav.getPolyHeight(tile, poly, pos); // value / NaN
    }

    /**
     * Finds the polygon nearest to the specified center point. If center and nearestPt point to an equal position,
     * isOverPoly will be true; however there's also a special case of climb height inside the polygon
     *
     * @param center      The center of the search box. [(x, y, z)]
     * @param halfExtents The search distance along each axis. [(x, y, z)]
     * @param filter      The polygon filter to apply to the query.
     * @return FindNearestPolyResult containing nearestRef, nearestPt and overPoly
     */
    public Result<FindNearestPolyResult> findNearestPoly(Vector3f center, Vector3f halfExtents, QueryFilter filter) {

        // Get nearby polygons from proximity grid.
        FindNearestPolyQuery query = new FindNearestPolyQuery(this, center);
        Status status = queryPolygons(center, halfExtents, filter, query);
        if (status.isFailed()) {
            return Result.of(status, null);
        }

        return Result.success(query.result());
    }

    protected void queryPolygonsInTile(MeshTile tile, Vector3f qmin, Vector3f qmax, QueryFilter filter, PolyQuery query) {
        if (tile.data.bvTree != null) {
            int nodeIndex = 0;
            Vector3f tbmin = tile.data.header.bmin;
            Vector3f tbmax = tile.data.header.bmax;
            float qfac = tile.data.header.bvQuantizationFactor;
            // Calculate quantized box
            // dtClamp query box to world box.
            float minx = clamp(qmin.x, tbmin.x, tbmax.x) - tbmin.x;
            float miny = clamp(qmin.y, tbmin.y, tbmax.y) - tbmin.y;
            float minz = clamp(qmin.z, tbmin.z, tbmax.z) - tbmin.z;
            float maxx = clamp(qmax.x, tbmin.x, tbmax.x) - tbmin.x;
            float maxy = clamp(qmax.y, tbmin.y, tbmax.y) - tbmin.y;
            float maxz = clamp(qmax.z, tbmin.z, tbmax.z) - tbmin.z;
            // Quantize
            Vector3i bmin = new Vector3i(
                    (int) (qfac * minx) & 0x7ffffffe,
                    (int) (qfac * miny) & 0x7ffffffe,
                    (int) (qfac * minz) & 0x7ffffffe
            );
            Vector3i bmax = new Vector3i(
                    (int) (qfac * maxx + 1) | 1,
                    (int) (qfac * maxy + 1) | 1,
                    (int) (qfac * maxz + 1) | 1
            );
            // Traverse tree
            long base = nav.getPolyRefBase(tile);
            int end = tile.data.header.bvNodeCount;
            while (nodeIndex < end) {
                BVNode node = tile.data.bvTree[nodeIndex];
                boolean overlap = overlapQuantBounds(bmin, bmax, node);
                boolean isLeafNode = node.index >= 0;

                if (isLeafNode && overlap) {
                    long ref = base | node.index;
                    if (filter.passFilter(ref, tile, tile.data.polygons[node.index])) {
                        query.process(tile, tile.data.polygons[node.index], ref);
                    }
                }

                if (overlap || isLeafNode) {
                    nodeIndex++;
                } else {
                    int escapeIndex = -node.index;
                    nodeIndex += escapeIndex;
                }
            }
        } else {
            Vector3f bmin = new Vector3f();
            Vector3f bmax = new Vector3f();
            long base = nav.getPolyRefBase(tile);
            for (int i = 0; i < tile.data.header.polyCount; ++i) {
                Poly p = tile.data.polygons[i];
                // Do not return off-mesh connection polygons.
                if (p.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
                    continue;
                }
                long ref = base | i;
                if (!filter.passFilter(ref, tile, p)) {
                    continue;
                }
                // Calc polygon bounds.
                int v = p.vertices[0] * 3;
                copy(bmin, tile.data.vertices, v);
                copy(bmax, tile.data.vertices, v);
                for (int j = 1; j < p.vertCount; ++j) {
                    v = p.vertices[j] * 3;
                    min(bmin, tile.data.vertices, v);
                    max(bmax, tile.data.vertices, v);
                }
                if (overlapBounds(qmin, qmax, bmin, bmax)) {
                    query.process(tile, p, ref);
                }
            }
        }
    }

    /**
     * Finds polygons that overlap the search box.
     * <p>
     * If no polygons are found, the function will return with a polyCount of zero.
     *
     * @param center      The center of the search box. [(x, y, z)]
     * @param halfExtents The search distance along each axis. [(x, y, z)]
     * @param filter      The polygon filter to apply to the query.
     * @return The reference ids of the polygons that overlap the query box.
     */
    public Status queryPolygons(Vector3f center, Vector3f halfExtents, QueryFilter filter, PolyQuery query) {
        if (Objects.isNull(center) || !isFinite(center) || Objects.isNull(halfExtents) || !isFinite(halfExtents)
                || Objects.isNull(filter)) {
            return Status.FAILURE_INVALID_PARAM;
        }
        // Find tiles the query touches.
        Vector3f bmin = sub(center, halfExtents);
        Vector3f bmax = add(center, halfExtents);
        queryTiles(center, halfExtents).forEach(t -> queryPolygonsInTile(t, bmin, bmax, filter, query));
        return Status.SUCCESS;
    }

    /**
     * Finds tiles that overlap the search box.
     */
    public List<MeshTile> queryTiles(Vector3f center, Vector3f halfExtents) {
        if (Objects.isNull(center) || !isFinite(center) || Objects.isNull(halfExtents) || !isFinite(halfExtents)) {
            return Collections.emptyList();
        }
        Vector3f bmin = sub(center, halfExtents);
        Vector3f bmax = add(center, halfExtents);
        int minx = nav.calcTileLocX(bmin);
        int miny = nav.calcTileLocY(bmin);
        int maxx = nav.calcTileLocX(bmax);
        int maxy = nav.calcTileLocY(bmax);
        List<MeshTile> tiles = new ArrayList<>();
        for (int y = miny; y <= maxy; ++y) {
            for (int x = minx; x <= maxx; ++x) {
                tiles.addAll(nav.getTilesAt(x, y));
            }
        }
        return tiles;
    }

    /**
     * Finds a path from the start polygon to the end polygon.
     * <p>
     * If the end polygon cannot be reached through the navigation graph, the last polygon in the path will be the
     * nearest the end polygon.
     * <p>
     * The start and end positions are used to calculate traversal costs. (The y-values impact the result.)
     *
     * @param startRef The refrence id of the start polygon.
     * @param endRef   The reference id of the end polygon.
     * @param startPos A position within the start polygon. [(x, y, z)]
     * @param endPos   A position within the end polygon. [(x, y, z)]
     * @param filter   The polygon filter to apply to the query.
     * @return Found path
     */
    public Result<LongArrayList> findPath(long startRef, long endRef, Vector3f startPos, Vector3f endPos,
                                          QueryFilter filter) {
        return findPath(startRef, endRef, startPos, endPos, filter, new DefaultQueryHeuristic(), 0, 0);
    }

    @SuppressWarnings("unused")
    public Result<LongArrayList> findPath(long startRef, long endRef, Vector3f startPos, Vector3f endPos, QueryFilter filter,
                                          int options, float raycastLimit) {
        return findPath(startRef, endRef, startPos, endPos, filter, new DefaultQueryHeuristic(), options, raycastLimit);
    }

    public Result<LongArrayList> findPath(long startRef, long endRef, Vector3f startPos, Vector3f endPos, QueryFilter filter,
                                          QueryHeuristic heuristic, int options, float raycastLimit) {
        // Validate input
        if (!nav.isValidPolyRef(startRef) || !nav.isValidPolyRef(endRef) || Objects.isNull(startPos)
                || !isFinite(startPos) || Objects.isNull(endPos) || !isFinite(endPos) || Objects.isNull(filter)) {
            return Result.invalidParam();
        }

        float raycastLimitSqr = sqr(raycastLimit);

        // trade quality with performance?
        if ((options & DT_FINDPATH_ANY_ANGLE) != 0 && raycastLimit < 0f) {
            // limiting to several times the character radius yields nice results. It is not sensitive
            // so it is enough to compute it from the first tile.
            MeshTile tile = nav.getTileByRef(startRef);
            float agentRadius = tile.data.header.walkableRadius;
            raycastLimitSqr = sqr(agentRadius * NavMesh.DT_RAY_CAST_LIMIT_PROPORTIONS);
        }

        if (startRef == endRef) {
            LongArrayList path = new LongArrayList(1);
            path.add(startRef);
            return Result.success(path);
        }

        nodePool.clear();
        openList.clear();

        Node startNode = nodePool.getNode(startRef);
        startNode.pos.set(startPos);
        startNode.parentIndex = 0;
        startNode.cost = 0;
        startNode.totalCost = heuristic.getCost(startPos, endPos);
        startNode.polygonRef = startRef;
        startNode.flags = Node.OPEN;
        openList.offer(startNode);

        Node lastBestNode = startNode;
        float lastBestNodeCost = startNode.totalCost;

        Status status = Status.SUCCESS;

        while (!openList.isEmpty()) {
            // Remove node from open list and put it in closed list.
            Node bestNode = openList.poll();
            bestNode.flags &= ~Node.OPEN;
            bestNode.flags |= Node.CLOSED;

            // Reached the goal, stop searching.
            if (bestNode.polygonRef == endRef) {
                lastBestNode = bestNode;
                break;
            }

            // Get current poly and tile.
            // The API input has been cheked already, skip checking internal data.
            long bestRef = bestNode.polygonRef;
            Pair<MeshTile, Poly> tileAndPoly = nav.getTileAndPolyByRefUnsafe(bestRef);
            MeshTile bestTile = tileAndPoly.getFirst();
            Poly bestPoly = tileAndPoly.getSecond();

            // Get parent poly and tile.
            long parentRef = 0, grandpaRef = 0;
            MeshTile parentTile = null;
            Poly parentPoly = null;
            Node parentNode = null;
            if (bestNode.parentIndex != 0) {
                parentNode = nodePool.getNodeAtIdx(bestNode.parentIndex);
                parentRef = parentNode.polygonRef;
                if (parentNode.parentIndex != 0) {
                    grandpaRef = nodePool.getNodeAtIdx(parentNode.parentIndex).polygonRef;
                }
            }
            if (parentRef != 0) {
                tileAndPoly = nav.getTileAndPolyByRefUnsafe(parentRef);
                parentTile = tileAndPoly.getFirst();
                parentPoly = tileAndPoly.getSecond();
            }

            // decide whether to test raycast to previous nodes
            boolean tryLOS = false;
            if ((options & DT_FINDPATH_ANY_ANGLE) != 0) {
                if ((parentRef != 0) && (raycastLimitSqr >= Float.MAX_VALUE
                        || parentNode.pos.distanceSquared(bestNode.pos) < raycastLimitSqr)) {
                    tryLOS = true;
                }
            }

            for (int i = bestTile.polyLinks[bestPoly.index]; i != NavMesh.DT_NULL_LINK; i = bestTile.links.get(i).indexOfNextLink) {
                long neighbourRef = bestTile.links.get(i).neighborRef;

                // Skip invalid ids and do not expand back to where we came from.
                if (neighbourRef == 0 || neighbourRef == parentRef) {
                    continue;
                }

                // Get neighbour poly and tile.
                // The API input has been cheked already, skip checking internal data.
                tileAndPoly = nav.getTileAndPolyByRefUnsafe(neighbourRef);
                MeshTile neighbourTile = tileAndPoly.getFirst();
                Poly neighbourPoly = tileAndPoly.getSecond();

                if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    continue;
                }

                // get the node
                Node neighbourNode = nodePool.getNode(neighbourRef, 0);

                // do not expand to nodes that were already visited from the
                // same parent
                if (neighbourNode.parentIndex != 0 && neighbourNode.parentIndex == bestNode.parentIndex) {
                    continue;
                }

                // If the node is visited the first time, calculate node position.
                Vector3f neighbourPos = neighbourNode.pos;
                Result<Vector3f> midpod = neighbourRef == endRef
                        ? getEdgeIntersectionPoint(bestNode.pos, bestRef, bestPoly, bestTile, endPos, neighbourRef,
                        neighbourPoly, neighbourTile)
                        : getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile);
                if (!midpod.failed()) {
                    neighbourPos = midpod.result;
                }

                // Calculate cost and heuristic.
                float cost = 0;
                float heuristicCost = 0;

                // raycast parent
                boolean foundShortCut = false;
                LongArrayList shortcut = null;
                if (tryLOS) {
                    Result<RaycastHit> rayHit = raycast(parentRef, parentNode.pos, neighbourPos, filter,
                            DT_RAYCAST_USE_COSTS, grandpaRef);
                    if (rayHit.succeeded()) {
                        foundShortCut = rayHit.result.t >= 1f;
                        if (foundShortCut) {
                            shortcut = rayHit.result.path;
                            // shortcut found using raycast. Using shorter cost
                            // instead
                            cost = parentNode.cost + rayHit.result.pathCost;
                        }
                    }
                }

                // update move cost
                if (!foundShortCut) {
                    float curCost = filter.getCost(bestNode.pos, neighbourPos, parentRef, parentTile,
                            parentPoly, bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly);
                    cost = bestNode.cost + curCost;
                }

                // Special case for last node.
                if (neighbourRef == endRef) {
                    // Cost
                    float endCost = filter.getCost(neighbourPos, endPos, bestRef, bestTile, bestPoly, neighbourRef,
                            neighbourTile, neighbourPoly, 0L, null, null);
                    cost = cost + endCost;
                } else {
                    // Cost
                    heuristicCost = heuristic.getCost(neighbourPos, endPos);
                }

                float total = cost + heuristicCost;

                // The node is already in open list, and the new result is worse, skip.
                if ((neighbourNode.flags & Node.OPEN) != 0 && total >= neighbourNode.totalCost) {
                    continue;
                }
                // The node is already visited and process, and the new result is worse, skip.
                if ((neighbourNode.flags & Node.CLOSED) != 0 && total >= neighbourNode.totalCost) {
                    continue;
                }

                // Add or update the node.
                neighbourNode.parentIndex = foundShortCut ? bestNode.parentIndex : nodePool.getNodeIdx(bestNode);
                neighbourNode.polygonRef = neighbourRef;
                neighbourNode.flags = (neighbourNode.flags & ~Node.CLOSED);
                neighbourNode.cost = cost;
                neighbourNode.totalCost = total;
                neighbourNode.pos = neighbourPos;
                neighbourNode.shortcut = shortcut;

                if ((neighbourNode.flags & Node.OPEN) != 0) {
                    // Already in open, update node location.
                    openList.remove(neighbourNode);
                    openList.offer(neighbourNode);
                } else {
                    // Put the node in open list.
                    neighbourNode.flags |= Node.OPEN;
                    openList.offer(neighbourNode);
                }

                // Update nearest node to target so far.
                if (heuristicCost < lastBestNodeCost) {
                    lastBestNodeCost = heuristicCost;
                    lastBestNode = neighbourNode;
                }
            }
        }

        LongArrayList path = getPathToNode(lastBestNode);
        if (lastBestNode.polygonRef != endRef) {
            status = Status.PARTIAL_RESULT;
        }
        return Result.of(status, path);
    }

    /**
     * Intializes a sliced path query.
     * <p>
     * Common use case: -# Call initSlicedFindPath() to initialize the sliced path query. -# Call updateSlicedFindPath()
     * until it returns complete. -# Call finalizeSlicedFindPath() to get the path.
     *
     * @param startRef The reference id of the start polygon.
     * @param endRef   The reference id of the end polygon.
     * @param startPos A position within the start polygon. [(x, y, z)]
     * @param endPos   A position within the end polygon. [(x, y, z)]
     * @param filter   The polygon filter to apply to the query.
     * @param options  query options (see: #FindPathOptions)
     */
    public Status initSlicedFindPath(long startRef, long endRef, Vector3f startPos, Vector3f endPos, QueryFilter filter, int options) {
        return initSlicedFindPath(startRef, endRef, startPos, endPos, filter, options, new DefaultQueryHeuristic(), -1f);
    }

    @SuppressWarnings("unused")
    public Status initSlicedFindPath(long startRef, long endRef, Vector3f startPos, Vector3f endPos, QueryFilter filter, int options, float raycastLimit) {
        return initSlicedFindPath(startRef, endRef, startPos, endPos, filter, options, new DefaultQueryHeuristic(), raycastLimit);
    }

    public Status initSlicedFindPath(long startRef, long endRef, Vector3f startPos, Vector3f endPos, QueryFilter filter, int options, QueryHeuristic heuristic, float raycastLimit) {
        // Init path state.
        queryData = new QueryData();
        queryData.status = Status.FAILURE;
        queryData.startRef = startRef;
        queryData.endRef = endRef;
        queryData.startPos.set(startPos);
        queryData.endPos.set(endPos);
        queryData.filter = filter;
        queryData.options = options;
        queryData.heuristic = heuristic;
        queryData.raycastLimitSqr = sqr(raycastLimit);

        // Validate input
        if (!nav.isValidPolyRef(startRef) || !nav.isValidPolyRef(endRef) || Objects.isNull(startPos)
                || !isFinite(startPos) || Objects.isNull(endPos) || !isFinite(endPos) || Objects.isNull(filter)) {
            return Status.FAILURE_INVALID_PARAM;
        }

        // trade quality with performance?
        if ((options & DT_FINDPATH_ANY_ANGLE) != 0 && raycastLimit < 0f) {
            // limiting to several times the character radius yields nice results. It is not sensitive
            // so it is enough to compute it from the first tile.
            MeshTile tile = nav.getTileByRef(startRef);
            float agentRadius = tile.data.header.walkableRadius;
            queryData.raycastLimitSqr = sqr(agentRadius * NavMesh.DT_RAY_CAST_LIMIT_PROPORTIONS);
        }

        if (startRef == endRef) {
            queryData.status = Status.SUCCESS;
            return Status.SUCCESS;
        }

        nodePool.clear();
        openList.clear();

        Node startNode = nodePool.getNode(startRef);
        copy(startNode.pos, startPos);
        startNode.parentIndex = 0;
        startNode.cost = 0;
        startNode.totalCost = heuristic.getCost(startPos, endPos);
        startNode.polygonRef = startRef;
        startNode.flags = Node.OPEN;
        openList.offer(startNode);

        queryData.status = Status.IN_PROGRESS;
        queryData.lastBestNode = startNode;
        queryData.lastBestNodeCost = startNode.totalCost;

        return queryData.status;
    }

    /**
     * Updates an in-progress sliced path query.
     *
     * @param maxIter The maximum number of iterations to perform.
     * @return The status flags for the query.
     */
    public Result<Integer> updateSlicedFindPath(int maxIter) {
        if (!queryData.status.isInProgress()) {
            return Result.of(queryData.status, 0);
        }

        // Make sure the request is still valid.
        if (!nav.isValidPolyRef(queryData.startRef) || !nav.isValidPolyRef(queryData.endRef)) {
            queryData.status = Status.FAILURE;
            return Result.of(queryData.status, 0);
        }

        int iter = 0;
        while (iter < maxIter && !openList.isEmpty()) {
            iter++;

            // Remove node from open list and put it in closed list.
            Node bestNode = openList.poll();
            bestNode.flags &= ~Node.OPEN;
            bestNode.flags |= Node.CLOSED;

            // Reached the goal, stop searching.
            if (bestNode.polygonRef == queryData.endRef) {
                queryData.lastBestNode = bestNode;
                queryData.status = Status.SUCCESS;
                return Result.of(queryData.status, iter);
            }

            // Get current poly and tile.
            // The API input has been cheked already, skip checking internal
            // data.
            long bestRef = bestNode.polygonRef;
            Result<Pair<MeshTile, Poly>> tileAndPoly = nav.getTileAndPolyByRef(bestRef);
            if (tileAndPoly.failed()) {
                queryData.status = Status.FAILURE;
                // The polygon has disappeared during the sliced query, fail.
                return Result.of(queryData.status, iter);
            }
            MeshTile bestTile = tileAndPoly.result.getFirst();
            Poly bestPoly = tileAndPoly.result.getSecond();
            // Get parent and grand parent poly and tile.
            long parentRef = 0, grandpaRef = 0;
            MeshTile parentTile = null;
            Poly parentPoly = null;
            Node parentNode = null;
            if (bestNode.parentIndex != 0) {
                parentNode = nodePool.getNodeAtIdx(bestNode.parentIndex);
                parentRef = parentNode.polygonRef;
                if (parentNode.parentIndex != 0) {
                    grandpaRef = nodePool.getNodeAtIdx(parentNode.parentIndex).polygonRef;
                }
            }
            if (parentRef != 0) {
                tileAndPoly = nav.getTileAndPolyByRef(parentRef);
                boolean invalidParent = tileAndPoly.failed();
                if (invalidParent || (grandpaRef != 0 && !nav.isValidPolyRef(grandpaRef))) {
                    // The polygon has disappeared during the sliced query,
                    // fail.
                    queryData.status = Status.FAILURE;
                    return Result.of(queryData.status, iter);
                }
                parentTile = tileAndPoly.result.getFirst();
                parentPoly = tileAndPoly.result.getSecond();
            }

            // decide whether to test raycast to previous nodes
            boolean tryLOS = false;
            if ((queryData.options & DT_FINDPATH_ANY_ANGLE) != 0) {
                if ((parentRef != 0) && (queryData.raycastLimitSqr >= Float.MAX_VALUE
                        || parentNode.pos.distanceSquared(bestNode.pos) < queryData.raycastLimitSqr)) {
                    tryLOS = true;
                }
            }

            for (int i = bestTile.polyLinks[bestPoly.index]; i != NavMesh.DT_NULL_LINK; i = bestTile.links.get(i).indexOfNextLink) {
                long neighbourRef = bestTile.links.get(i).neighborRef;

                // Skip invalid ids and do not expand back to where we came
                // from.
                if (neighbourRef == 0 || neighbourRef == parentRef) {
                    continue;
                }

                // Get neighbour poly and tile.
                // The API input has been cheked already, skip checking internal
                // data.
                Pair<MeshTile, Poly> tileAndPolyUns = nav.getTileAndPolyByRefUnsafe(neighbourRef);
                MeshTile neighbourTile = tileAndPolyUns.getFirst();
                Poly neighbourPoly = tileAndPolyUns.getSecond();

                if (!queryData.filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    continue;
                }

                // get the neighbor node
                Node neighbourNode = nodePool.getNode(neighbourRef, 0);

                // do not expand to nodes that were already visited from the
                // same parent
                if (neighbourNode.parentIndex != 0 && neighbourNode.parentIndex == bestNode.parentIndex) {
                    continue;
                }

                // If the node is visited the first time, calculate node
                // position.
                Vector3f neighbourPos = neighbourNode.pos;
                Result<Vector3f> midpod = neighbourRef == queryData.endRef
                        ? getEdgeIntersectionPoint(bestNode.pos, bestRef, bestPoly, bestTile, queryData.endPos,
                        neighbourRef, neighbourPoly, neighbourTile)
                        : getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile);
                if (!midpod.failed()) {
                    neighbourPos = midpod.result;
                }

                // Calculate cost and heuristic.
                float cost = 0;
                float heuristic;

                // raycast parent
                boolean foundShortCut = false;
                LongArrayList shortcut = null;
                if (tryLOS) {
                    Result<RaycastHit> rayHit = raycast(parentRef, parentNode.pos, neighbourPos, queryData.filter,
                            DT_RAYCAST_USE_COSTS, grandpaRef);
                    if (rayHit.succeeded()) {
                        foundShortCut = rayHit.result.t >= 1f;
                        if (foundShortCut) {
                            shortcut = rayHit.result.path;
                            // shortcut found using raycast. Using shorter cost
                            // instead
                            cost = parentNode.cost + rayHit.result.pathCost;
                        }
                    }
                }

                // update move cost
                if (!foundShortCut) {
                    // No shortcut found.
                    float curCost = queryData.filter.getCost(bestNode.pos, neighbourPos, parentRef, parentTile,
                            parentPoly, bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly);
                    cost = bestNode.cost + curCost;
                }

                // Special case for last node.
                if (neighbourRef == queryData.endRef) {
                    float endCost = queryData.filter.getCost(neighbourPos, queryData.endPos, bestRef, bestTile,
                            bestPoly, neighbourRef, neighbourTile, neighbourPoly, 0, null, null);

                    cost = cost + endCost;
                    heuristic = 0;
                } else {
                    heuristic = queryData.heuristic.getCost(neighbourPos, queryData.endPos);
                }

                float total = cost + heuristic;

                // The node is already in open list and the new result is worse,
                // skip.
                if ((neighbourNode.flags & Node.OPEN) != 0 && total >= neighbourNode.totalCost) {
                    continue;
                }
                // The node is already visited and process, and the new result
                // is worse, skip.
                if ((neighbourNode.flags & Node.CLOSED) != 0 && total >= neighbourNode.totalCost) {
                    continue;
                }

                // Add or update the node.
                neighbourNode.parentIndex = foundShortCut ? bestNode.parentIndex : nodePool.getNodeIdx(bestNode);
                neighbourNode.polygonRef = neighbourRef;
                neighbourNode.flags = (neighbourNode.flags & ~Node.CLOSED);
                neighbourNode.cost = cost;
                neighbourNode.totalCost = total;
                neighbourNode.pos = neighbourPos;
                neighbourNode.shortcut = shortcut;

                if ((neighbourNode.flags & Node.OPEN) != 0) {
                    // Already in open, update node location.
                    openList.remove(neighbourNode);
                    openList.offer(neighbourNode);
                } else {
                    // Put the node in open list.
                    neighbourNode.flags |= Node.OPEN;
                    openList.offer(neighbourNode);
                }

                // Update nearest node to target so far.
                if (heuristic < queryData.lastBestNodeCost) {
                    queryData.lastBestNodeCost = heuristic;
                    queryData.lastBestNode = neighbourNode;
                }
            }
        }

        // Exhausted all nodes, but could not find path.
        if (openList.isEmpty()) {
            queryData.status = Status.PARTIAL_RESULT;
        }

        return Result.of(queryData.status, iter);
    }

    /// Finalizes and returns the results of a sliced path query.
    /// @param[out] path An ordered list of polygon references representing the path. (Start to end.)
    /// [(polyRef) * @p pathCount]
    /// @returns The status flags for the query.
    public Result<LongArrayList> finalizeSlicedFindPath() {

        LongArrayList path = new LongArrayList(64);
        if (queryData.status.isFailed()) {
            // Reset query.
            queryData = new QueryData();
            return Result.failure(path);
        }

        if (queryData.startRef == queryData.endRef) {
            // Special case: the search starts and ends at same poly.
            path.add(queryData.startRef);
        } else {
            // Reverse the path.
            if (queryData.lastBestNode.polygonRef != queryData.endRef) {
                queryData.status = Status.PARTIAL_RESULT;
            }
            path = getPathToNode(queryData.lastBestNode);
        }

        Status status = queryData.status;
        // Reset query.
        queryData = new QueryData();

        return Result.of(status, path);
    }

    /// Finalizes and returns the results of an incomplete sliced path query, returning the path to the furthest
    /// polygon on the existing path that was visited during the search.
    /// @param[in] existing An array of polygon references for the existing path.
    /// @param[in] existingSize The number of polygon in the @p existing array.
    /// @param[out] path An ordered list of polygon references representing the path. (Start to end.)
    /// [(polyRef) * @p pathCount]
    /// @returns The status flags for the query.
    public Result<LongArrayList> finalizeSlicedFindPathPartial(LongArrayList existing) {

        LongArrayList path = new LongArrayList(64);
        if (Objects.isNull(existing) || existing.isEmpty()) {
            return Result.failure(path);
        }
        if (queryData.status.isFailed()) {
            // Reset query.
            queryData = new QueryData();
            return Result.failure(path);
        }
        if (queryData.startRef == queryData.endRef) {
            // Special case: the search starts and ends at same poly.
            path.add(queryData.startRef);
        } else {
            // Find the furthest existing node that was visited.
            Node node = null;
            for (int i = existing.getSize() - 1; i >= 0; --i) {
                node = nodePool.findNode(existing.get(i));
                if (node != null) {
                    break;
                }
            }

            if (node == null) {
                queryData.status = Status.PARTIAL_RESULT;
                node = queryData.lastBestNode;
            }

            path = getPathToNode(node);

        }
        Status status = queryData.status;
        // Reset query.
        queryData = new QueryData();

        return Result.of(status, path);
    }

    protected Status appendVertex(Vector3f pos, int flags, long ref, List<StraightPathItem> straightPath, int maxStraightPath) {
        if (straightPath.size() > 0 && vEqual(straightPath.get(straightPath.size() - 1).pos, pos)) {
            // The vertices are equal, update flags and poly.
            straightPath.get(straightPath.size() - 1).flags = flags;
            straightPath.get(straightPath.size() - 1).ref = ref;
        } else {
            if (straightPath.size() < maxStraightPath) {
                // Append new vertex.
                straightPath.add(new StraightPathItem(pos, flags, ref));
            }
            // If reached end of path or there is no space to append more vertices, return.
            if (flags == DT_STRAIGHTPATH_END || straightPath.size() >= maxStraightPath) {
                return Status.SUCCESS;
            }
        }
        return Status.IN_PROGRESS;
    }

    protected Status appendPortals(int startIdx, int endIdx, Vector3f endPos, LongArrayList path,
                                   List<StraightPathItem> straightPath, int maxStraightPath, int options) {
        Vector3f startPos = straightPath.get(straightPath.size() - 1).pos;
        // Append or update last vertex
        Status stat;
        for (int i = startIdx; i < endIdx; i++) {
            // Calculate portal
            long from = path.get(i);
            Result<Pair<MeshTile, Poly>> tileAndPoly = nav.getTileAndPolyByRef(from);
            if (tileAndPoly.failed()) {
                return Status.FAILURE;
            }
            MeshTile fromTile = tileAndPoly.result.getFirst();
            Poly fromPoly = tileAndPoly.result.getSecond();

            long to = path.get(i + 1);
            tileAndPoly = nav.getTileAndPolyByRef(to);
            if (tileAndPoly.failed()) {
                return Status.FAILURE;
            }
            MeshTile toTile = tileAndPoly.result.getFirst();
            Poly toPoly = tileAndPoly.result.getSecond();

            Result<PortalResult> portals = getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, 0, 0);
            if (portals.failed()) {
                break;
            }
            Vector3f left = portals.result.left;
            Vector3f right = portals.result.right;

            if ((options & DT_STRAIGHTPATH_AREA_CROSSINGS) != 0) {
                // Skip intersection if only area crossings are requested.
                if (fromPoly.getArea() == toPoly.getArea()) {
                    continue;
                }
            }

            // Append intersection
            Optional<Pair<Float, Float>> interect = intersectSegSeg2D(startPos, endPos, left, right);
            if (interect.isPresent()) {
                float t = interect.get().getSecond();
                Vector3f pt = lerp(left, right, t);
                stat = appendVertex(pt, 0, path.get(i + 1), straightPath, maxStraightPath);
                if (!stat.isInProgress()) {
                    return stat;
                }
            }
        }
        return Status.IN_PROGRESS;
    }

    /// @par
    /// Finds the straight path from the start to the end position within the polygon corridor.
    ///
    /// This method peforms what is often called 'string pulling'.
    ///
    /// The start position is clamped to the first polygon in the path, and the
    /// end position is clamped to the last. So the start and end positions should
    /// normally be within or very near the first and last polygons respectively.
    ///
    /// The returned polygon references represent the reference id of the polygon
    /// that is entered at the associated path position. The reference id associated
    /// with the end point will always be zero. This allows, for example, matching
    /// off-mesh link points to their representative polygons.
    ///
    /// If the provided result buffers are too small for the entire result set,
    /// they will be filled as far as possible from the start toward the end
    /// position.
    ///
    /// @param[in] startPos Path start position. [(x, y, z)]
    /// @param[in] endPos Path end position. [(x, y, z)]
    /// @param[in] path An array of polygon references that represent the path corridor.
    /// @param[out] straightPath Points describing the straight path. [(x, y, z) * @p straightPathCount].
    /// @param[in] maxStraightPath The maximum number of points the straight path arrays can hold. [Limit: > 0]
    /// @param[in] options Query options. (see: #dtStraightPathOptions)
    /// @returns The status flags for the query.
    public Result<List<StraightPathItem>> findStraightPath(Vector3f startPos, Vector3f endPos, LongArrayList path,
                                                           int maxStraightPath, int options) {

        List<StraightPathItem> straightPath = new ArrayList<>();
        if (Objects.isNull(startPos) || !isFinite(startPos) || Objects.isNull(endPos) || !isFinite(endPos)
                || Objects.isNull(path) || path.isEmpty() || path.get(0) == 0 || maxStraightPath <= 0) {
            return Result.invalidParam();
        }
        // TODO: Should this be callers responsibility?
        Result<Vector3f> closestStartPosRes = closestPointOnPolyBoundary(path.get(0), startPos);
        if (closestStartPosRes.failed()) {
            return Result.invalidParam("Cannot find start position");
        }
        Vector3f closestStartPos = closestStartPosRes.result;
        Result<Vector3f> closestEndPosRes = closestPointOnPolyBoundary(path.get(path.getSize() - 1), endPos);
        if (closestEndPosRes.failed()) {
            return Result.invalidParam("Cannot find end position");
        }
        Vector3f closestEndPos = closestEndPosRes.result;
        // Add start point.
        Status stat = appendVertex(closestStartPos, DT_STRAIGHTPATH_START, path.get(0), straightPath, maxStraightPath);
        if (!stat.isInProgress()) {
            return Result.success(straightPath);
        }

        if (path.getSize() > 1) {
            Vector3f portalApex = copy(closestStartPos);
            Vector3f portalLeft = copy(portalApex);
            Vector3f portalRight = copy(portalApex);
            int apexIndex = 0;
            int leftIndex = 0;
            int rightIndex = 0;

            int leftPolyType = 0;
            int rightPolyType = 0;

            long leftPolyRef = path.get(0);
            long rightPolyRef = path.get(0);

            for (int i = 0; i < path.getSize(); ++i) {
                Vector3f left;
                Vector3f right;
                int toType;

                if (i + 1 < path.getSize()) {
                    // Next portal.
                    Result<PortalResult> portalPoints = getPortalPoints(path.get(i), path.get(i + 1));
                    if (portalPoints.failed()) {
                        closestEndPosRes = closestPointOnPolyBoundary(path.get(i), endPos);
                        if (closestEndPosRes.failed()) {
                            return Result.invalidParam();
                        }
                        closestEndPos = closestEndPosRes.result;
                        // Append portals along the current straight path segment.
                        if ((options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0) {
                            // Ignore status return value as we're just about to return anyway.
                            appendPortals(apexIndex, i, closestEndPos, path, straightPath, maxStraightPath, options);
                        }
                        // Ignore status return value as we're just about to return anyway.
                        appendVertex(closestEndPos, 0, path.get(i), straightPath, maxStraightPath);
                        return Result.success(straightPath);
                    }
                    left = portalPoints.result.left;
                    right = portalPoints.result.right;
                    toType = portalPoints.result.toType;

                    // If starting really close the portal, advance.
                    if (i == 0) {
                        Pair<Float, Float> dt = distancePtSegSqr2D(portalApex, left, right);
                        if (dt.getFirst() < sqr(0.001f)) {
                            continue;
                        }
                    }
                } else {
                    // End of the path.
                    left = copy(closestEndPos);
                    right = copy(closestEndPos);
                    toType = Poly.DT_POLYTYPE_GROUND;
                }

                // Right vertex.
                if (triArea2D(portalApex, portalRight, right) <= 0f) {
                    if (vEqual(portalApex, portalRight) || triArea2D(portalApex, portalLeft, right) > 0f) {
                        portalRight = copy(right);
                        rightPolyRef = (i + 1 < path.getSize()) ? path.get(i + 1) : 0;
                        rightPolyType = toType;
                        rightIndex = i;
                    } else {
                        // Append portals along the current straight path segment.
                        if ((options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0) {
                            stat = appendPortals(apexIndex, leftIndex, portalLeft, path, straightPath, maxStraightPath,
                                    options);
                            if (!stat.isInProgress()) {
                                return Result.success(straightPath);
                            }
                        }

                        portalApex = copy(portalLeft);
                        apexIndex = leftIndex;

                        int flags = 0;
                        if (leftPolyRef == 0) {
                            flags = DT_STRAIGHTPATH_END;
                        } else if (leftPolyType == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
                            flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
                        }

                        // Append or update vertex
                        stat = appendVertex(portalApex, flags, leftPolyRef, straightPath, maxStraightPath);
                        if (!stat.isInProgress()) {
                            return Result.success(straightPath);
                        }

                        portalLeft = copy(portalApex);
                        portalRight = copy(portalApex);
                        rightIndex = apexIndex;

                        // Restart
                        i = apexIndex;

                        continue;
                    }
                }

                // Left vertex.
                if (triArea2D(portalApex, portalLeft, left) >= 0f) {
                    if (vEqual(portalApex, portalLeft) || triArea2D(portalApex, portalRight, left) < 0f) {
                        portalLeft = copy(left);
                        leftPolyRef = (i + 1 < path.getSize()) ? path.get(i + 1) : 0;
                        leftPolyType = toType;
                        leftIndex = i;
                    } else {
                        // Append portals along the current straight path segment.
                        if ((options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0) {
                            stat = appendPortals(apexIndex, rightIndex, portalRight, path, straightPath,
                                    maxStraightPath, options);
                            if (!stat.isInProgress()) {
                                return Result.success(straightPath);
                            }
                        }

                        portalApex = copy(portalRight);
                        apexIndex = rightIndex;

                        int flags = 0;
                        if (rightPolyRef == 0) {
                            flags = DT_STRAIGHTPATH_END;
                        } else if (rightPolyType == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
                            flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
                        }

                        // Append or update vertex
                        stat = appendVertex(portalApex, flags, rightPolyRef, straightPath, maxStraightPath);
                        if (!stat.isInProgress()) {
                            return Result.success(straightPath);
                        }

                        portalLeft = copy(portalApex);
                        portalRight = copy(portalApex);
                        leftIndex = apexIndex;

                        // Restart
                        i = apexIndex;

                    }
                }
            }

            // Append portals along the current straight path segment.
            if ((options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0) {
                stat = appendPortals(apexIndex, path.getSize() - 1, closestEndPos, path, straightPath, maxStraightPath, options);
                if (!stat.isInProgress()) {
                    return Result.success(straightPath);
                }
            }
        }

        // Ignore status return value as we're just about to return anyway.
        appendVertex(closestEndPos, DT_STRAIGHTPATH_END, 0, straightPath, maxStraightPath);
        return Result.success(straightPath);
    }

    /// @par
    ///
    /// This method is optimized for small delta movement and a small number of
    /// polygons. If used for too great a distance, the result set will form an
    /// incomplete path.
    ///
    /// @p resultPos will equal the @p endPos if the end is reached.
    /// Otherwise the closest reachable position will be returned.
    ///
    /// @p resultPos is not projected onto the surface of the navigation
    /// mesh. Use #getPolyHeight if this is needed.
    ///
    /// This method treats the end position in the same manner as
    /// the #raycast method. (As a 2D point.) See that method's documentation
    /// for details.
    ///
    /// If the @p visited array is too small to hold the entire result set, it will
    /// be filled as far as possible from the start position toward the end
    /// position.
    ///
    /// Moves from the start to the end position constrained to the navigation mesh.
    /// @param[in] startRef The reference id of the start polygon.
    /// @param[in] startPos A position of the mover within the start polygon. [(x, y, x)]
    /// @param[in] endPos The desired end position of the mover. [(x, y, z)]
    /// @param[in] filter The polygon filter to apply to the query.
    /// @returns Path
    public Result<MoveAlongSurfaceResult> moveAlongSurface(long startRef, Vector3f startPos, Vector3f endPos,
                                                           QueryFilter filter) {

        // Validate input
        if (!nav.isValidPolyRef(startRef) || Objects.isNull(startPos) || !isFinite(startPos)
                || Objects.isNull(endPos) || !isFinite(endPos) || Objects.isNull(filter)) {
            return Result.invalidParam();
        }

        NodePool tinyNodePool = new NodePool();

        Node startNode = tinyNodePool.getNode(startRef);
        startNode.parentIndex = 0;
        startNode.cost = 0;
        startNode.totalCost = 0;
        startNode.polygonRef = startRef;
        startNode.flags = Node.CLOSED;
        LinkedList<Node> stack = new LinkedList<>();
        stack.add(startNode);

        Vector3f bestPos = new Vector3f();
        float bestDist = Float.MAX_VALUE;
        Node bestNode = null;
        copy(bestPos, startPos);

        // Search constraints
        Vector3f searchPos = lerp(startPos, endPos, 0.5f);
        float searchRadSqr = sqr(startPos.distance(endPos) * 0.5f + 0.001f);

        float[] vertices = new float[nav.maxVerticesPerPoly * 3];

        while (!stack.isEmpty()) {
            // Pop front.
            Node curNode = stack.pop();

            // Get poly and tile.
            // The API input has been cheked already, skip checking internal data.
            long curRef = curNode.polygonRef;
            Pair<MeshTile, Poly> tileAndPoly = nav.getTileAndPolyByRefUnsafe(curRef);
            MeshTile curTile = tileAndPoly.getFirst();
            Poly curPoly = tileAndPoly.getSecond();

            // Collect vertices.
            int nvertices = curPoly.vertCount;
            for (int i = 0; i < nvertices; ++i) {
                System.arraycopy(curTile.data.vertices, curPoly.vertices[i] * 3, vertices, i * 3, 3);
            }

            // If target is inside the poly, stop search.
            if (pointInPolygon(endPos, vertices, nvertices)) {
                bestNode = curNode;
                copy(bestPos, endPos);
                break;
            }

            // Find wall edges and find nearest point inside the walls.
            for (int i = 0, j = curPoly.vertCount - 1; i < curPoly.vertCount; j = i++) {
                // Find links to neighbours.
                int MAX_NEIS = 8;
                int nneis = 0;
                long[] neis = new long[MAX_NEIS];

                if ((curPoly.neighborData[j] & NavMesh.DT_EXT_LINK) != 0) {
                    // Tile border.
                    for (int k = curTile.polyLinks[curPoly.index]; k != NavMesh.DT_NULL_LINK; k = curTile.links.get(k).indexOfNextLink) {
                        Link link = curTile.links.get(k);
                        if (link.indexOfPolyEdge == j) {
                            if (link.neighborRef != 0) {
                                tileAndPoly = nav.getTileAndPolyByRefUnsafe(link.neighborRef);
                                MeshTile neiTile = tileAndPoly.getFirst();
                                Poly neiPoly = tileAndPoly.getSecond();
                                if (filter.passFilter(link.neighborRef, neiTile, neiPoly)) {
                                    if (nneis < MAX_NEIS) {
                                        neis[nneis++] = link.neighborRef;
                                    }
                                }
                            }
                        }
                    }
                } else if (curPoly.neighborData[j] != 0) {
                    int idx = curPoly.neighborData[j] - 1;
                    long ref = nav.getPolyRefBase(curTile) | idx;
                    if (filter.passFilter(ref, curTile, curTile.data.polygons[idx])) {
                        // Internal edge, encode id.
                        neis[nneis++] = ref;
                    }
                }

                if (nneis == 0) {
                    // Wall edge, calc distance.
                    int vj = j * 3;
                    int vi = i * 3;
                    Pair<Float, Float> distSeg = distancePtSegSqr2D(endPos, vertices, vj, vi);
                    float distSqr = distSeg.getFirst();
                    float tseg = distSeg.getSecond();
                    if (distSqr < bestDist) {
                        // Update nearest distance.
                        bestPos = lerp(vertices, vj, vi, tseg);
                        bestDist = distSqr;
                        bestNode = curNode;
                    }
                } else {
                    for (int k = 0; k < nneis; ++k) {
                        Node neighbourNode = tinyNodePool.getNode(neis[k]);
                        // Skip if already visited.
                        if ((neighbourNode.flags & Node.CLOSED) != 0) {
                            continue;
                        }

                        // Skip the link if it is too far from search constraint.
                        // TODO: Maybe should use getPortalPoints(), but this one is way faster.
                        int vj = j * 3;
                        int vi = i * 3;
                        Pair<Float, Float> distseg = distancePtSegSqr2D(searchPos, vertices, vj, vi);
                        float distSqr = distseg.getFirst();
                        if (distSqr > searchRadSqr) {
                            continue;
                        }

                        // Mark as the node as visited and push to queue.
                        neighbourNode.parentIndex = tinyNodePool.getNodeIdx(curNode);
                        neighbourNode.flags |= Node.CLOSED;
                        stack.add(neighbourNode);
                    }
                }
            }
        }

        LongArrayList visited = new LongArrayList();
        if (bestNode != null) {
            // Reverse the path.
            Node prev = null;
            Node node = bestNode;
            do {
                Node next = tinyNodePool.getNodeAtIdx(node.parentIndex);
                node.parentIndex = tinyNodePool.getNodeIdx(prev);
                prev = node;
                node = next;
            } while (node != null);

            // Store result
            node = prev;
            do {
                visited.add(node.polygonRef);
                node = tinyNodePool.getNodeAtIdx(node.parentIndex);
            } while (node != null);
        }
        return Result.success(new MoveAlongSurfaceResult(bestPos, visited));
    }

    static class PortalResult {
        final Vector3f left;
        final Vector3f right;
        final int fromType;
        final int toType;

        public PortalResult(Vector3f left, Vector3f right, int fromType, int toType) {
            this.left = left;
            this.right = right;
            this.fromType = fromType;
            this.toType = toType;
        }

    }

    protected Result<PortalResult> getPortalPoints(long from, long to) {
        Result<Pair<MeshTile, Poly>> tileAndPolyResult = nav.getTileAndPolyByRef(from);
        if (tileAndPolyResult.failed()) {
            return Result.of(tileAndPolyResult.status, tileAndPolyResult.message);
        }
        Pair<MeshTile, Poly> tileAndPoly = tileAndPolyResult.result;
        MeshTile fromTile = tileAndPoly.getFirst();
        Poly fromPoly = tileAndPoly.getSecond();
        int fromType = fromPoly.getType();

        tileAndPolyResult = nav.getTileAndPolyByRef(to);
        if (tileAndPolyResult.failed()) {
            return Result.of(tileAndPolyResult.status, tileAndPolyResult.message);
        }
        tileAndPoly = tileAndPolyResult.result;
        MeshTile toTile = tileAndPoly.getFirst();
        Poly toPoly = tileAndPoly.getSecond();
        int toType = toPoly.getType();

        return getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, fromType, toType);
    }

    // Returns portal points between two polygons.
    protected Result<PortalResult> getPortalPoints(long from, Poly fromPoly, MeshTile fromTile, long to, Poly toPoly,
                                                   MeshTile toTile, int fromType, int toType) {
        Vector3f left = new Vector3f();
        Vector3f right = new Vector3f();
        // Find the link that points to the 'to' polygon.
        Link link = null;
        for (int i = fromTile.polyLinks[fromPoly.index]; i != NavMesh.DT_NULL_LINK; i = fromTile.links.get(i).indexOfNextLink) {
            if (fromTile.links.get(i).neighborRef == to) {
                link = fromTile.links.get(i);
                break;
            }
        }
        if (link == null) {
            return Result.invalidParam("No link found");
        }

        // Handle off-mesh connections.
        if (fromPoly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
            // Find link that points to first vertex.
            for (int i = fromTile.polyLinks[fromPoly.index]; i != NavMesh.DT_NULL_LINK; i = fromTile.links.get(i).indexOfNextLink) {
                if (fromTile.links.get(i).neighborRef == to) {
                    int v = fromTile.links.get(i).indexOfPolyEdge;
                    copy(left, fromTile.data.vertices, fromPoly.vertices[v] * 3);
                    copy(right, fromTile.data.vertices, fromPoly.vertices[v] * 3);
                    return Result.success(new PortalResult(left, right, fromType, toType));
                }
            }
            return Result.invalidParam("Invalid offmesh from connection");
        }

        if (toPoly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
            for (int i = toTile.polyLinks[toPoly.index]; i != NavMesh.DT_NULL_LINK; i = toTile.links.get(i).indexOfNextLink) {
                if (toTile.links.get(i).neighborRef == from) {
                    int v = toTile.links.get(i).indexOfPolyEdge;
                    copy(left, toTile.data.vertices, toPoly.vertices[v] * 3);
                    copy(right, toTile.data.vertices, toPoly.vertices[v] * 3);
                    return Result.success(new PortalResult(left, right, fromType, toType));
                }
            }
            return Result.invalidParam("Invalid offmesh to connection");
        }

        // Find portal vertices.
        int v0 = fromPoly.vertices[link.indexOfPolyEdge];
        int v1 = fromPoly.vertices[(link.indexOfPolyEdge + 1) % fromPoly.vertCount];
        copy(left, fromTile.data.vertices, v0 * 3);
        copy(right, fromTile.data.vertices, v1 * 3);

        // If the link is at tile boundary, dtClamp the vertices to
        // the link width.
        if (link.side != 0xff) {
            // Unpack portal limits.
            if (link.bmin != 0 || link.bmax != 255) {
                float s = 1f / 255f;
                float tmin = link.bmin * s;
                float tmax = link.bmax * s;
                left = lerp(fromTile.data.vertices, v0 * 3, v1 * 3, tmin);
                right = lerp(fromTile.data.vertices, v0 * 3, v1 * 3, tmax);
            }
        }

        return Result.success(new PortalResult(left, right, fromType, toType));
    }

    protected Result<Vector3f> getEdgeMidPoint(long from, Poly fromPoly, MeshTile fromTile, long to, Poly toPoly, MeshTile toTile) {
        Result<PortalResult> ppoints = getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, 0, 0);
        if (ppoints.failed()) return Result.of(ppoints.status, ppoints.message);
        Vector3f left = ppoints.result.left;
        Vector3f right = ppoints.result.right;
        return Result.success(new Vector3f(left).add(right).mul(0.5f));
    }

    protected Result<Vector3f> getEdgeIntersectionPoint(Vector3f fromPos, long from, Poly fromPoly, MeshTile fromTile,
                                                        Vector3f toPos, long to, Poly toPoly, MeshTile toTile) {
        Result<PortalResult> ppoints = getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, 0, 0);
        if (ppoints.failed()) return Result.of(ppoints.status, ppoints.message);
        Vector3f left = ppoints.result.left;
        Vector3f right = ppoints.result.right;
        float t = 0.5f;
        Optional<Pair<Float, Float>> interect = intersectSegSeg2D(fromPos, toPos, left, right);
        if (interect.isPresent()) {
            t = clamp(interect.get().getSecond(), 0.1f, 0.9f);
        }
        Vector3f pt = lerp(left, right, t);
        return Result.success(pt);
    }

    /// @par
    ///
    /// This method is meant to be used for quick, short distance checks.
    ///
    /// If the path array is too small to hold the result, it will be filled as
    /// far as possible from the start position toward the end position.
    ///
    /// <b>Using the Hit Parameter t of RaycastHit</b>
    ///
    /// If the hit parameter is a very high value (FLT_MAX), then the ray has hit
    /// the end position. In this case the path represents a valid corridor to the
    /// end position, and the value of @p hitNormal is undefined.
    ///
    /// If the hit parameter is zero, then the start position is on the wall that
    /// was hit, and the value of @p hitNormal is undefined.
    ///
    /// If 0 < t < 1.0 then the following applies:
    ///
    /// @code
    /// distanceToHitBorder = distanceToEndPosition * t
    /// hitPoint = startPos + (endPos - startPos) * t
    /// @endcode
    ///
    /// <b>Use Case Restriction</b>
    ///
    /// The raycast ignores the y-value of the end position. (2D check.) This
    /// places significant limits on how it can be used. For example:
    ///
    /// Consider a scene where there is a main floor with a second floor balcony
    /// that hangs over the main floor. So the first floor mesh extends below the
    /// balcony mesh. The start position is somewhere on the first floor. The end
    /// position is on the balcony.
    ///
    /// The raycast will search toward the end position along the first floor mesh.
    /// If it reaches the end position's xz-coordinates it will indicate FLT_MAX
    /// (no wall hit), meaning it reached the end position. This is one example of why
    /// this method is meant for short distance checks.
    ///
    /// Casts a 'walkability' ray along the surface of the navigation mesh from
    /// the start position toward the end position.
    /// @note A wrapper around raycast(..., RaycastHit*). Retained for backward compatibility.
    /// @param[in] startRef The reference id of the start polygon.
    /// @param[in] startPos A position within the start polygon representing
    /// the start of the ray. [(x, y, z)]
    /// @param[in] endPos The position to cast the ray toward. [(x, y, z)]
    /// @param[out] t The hit parameter. (FLT_MAX if no wall hit.)
    /// @param[out] hitNormal The normal of the nearest wall hit. [(x, y, z)]
    /// @param[in] filter The polygon filter to apply to the query.
    /// @param[out] path The reference ids of the visited polygons. [opt]
    /// @param[out] pathCount The number of visited polygons. [opt]
    /// @param[in] maxPath The maximum number of polygons the @p path array can hold.
    /// @returns The status flags for the query.
    public Result<RaycastHit> raycast(long startRef, Vector3f startPos, Vector3f endPos, QueryFilter filter, int options,
                                      long prevRef) {
        // Validate input
        if (!nav.isValidPolyRef(startRef) || Objects.isNull(startPos) || !isFinite(startPos)
                || Objects.isNull(endPos) || !isFinite(endPos) || Objects.isNull(filter)
                || (prevRef != 0 && !nav.isValidPolyRef(prevRef))) {
            return Result.invalidParam();
        }

        RaycastHit hit = new RaycastHit();

        float[] vertices = new float[nav.maxVerticesPerPoly * 3 + 3];

        Vector3f curPos = new Vector3f(), lastPos = new Vector3f();

        copy(curPos, startPos);
        Vector3f dir = sub(endPos, startPos);

        MeshTile prevTile, tile, nextTile;
        Poly prevPoly, poly, nextPoly;

        // The API input has been checked already, skip checking internal data.
        long curRef = startRef;
        Pair<MeshTile, Poly> tileAndPolyUns = nav.getTileAndPolyByRefUnsafe(curRef);
        tile = tileAndPolyUns.getFirst();
        poly = tileAndPolyUns.getSecond();
        nextTile = prevTile = tile;
        nextPoly = prevPoly = poly;
        if (prevRef != 0) {
            tileAndPolyUns = nav.getTileAndPolyByRefUnsafe(prevRef);
            prevTile = tileAndPolyUns.getFirst();
            prevPoly = tileAndPolyUns.getSecond();
        }
        while (curRef != 0) {
            // Cast ray against current polygon.

            // Collect vertices.
            int nv = 0;
            for (int i = 0; i < poly.vertCount; ++i) {
                System.arraycopy(tile.data.vertices, poly.vertices[i] * 3, vertices, nv * 3, 3);
                nv++;
            }

            IntersectResult iresult = intersectSegmentPoly2D(startPos, endPos, vertices, nv);
            if (!iresult.intersects) {
                // Could not hit the polygon, keep the old t and report hit.
                return Result.success(hit);
            }

            hit.hitEdgeIndex = iresult.segMax;

            // Keep track of furthest t so far.
            if (iresult.tmax > hit.t) {
                hit.t = iresult.tmax;
            }

            // Store visited polygons.
            hit.path.add(curRef);

            // Ray end is completely inside the polygon.
            if (iresult.segMax == -1) {
                hit.t = Float.MAX_VALUE;

                // add the cost
                if ((options & DT_RAYCAST_USE_COSTS) != 0) {
                    hit.pathCost += filter.getCost(curPos, endPos, prevRef, prevTile, prevPoly, curRef, tile, poly,
                            curRef, tile, poly);
                }
                return Result.success(hit);
            }

            // Follow neighbours.
            long nextRef = 0;

            for (int i = tile.polyLinks[poly.index]; i != NavMesh.DT_NULL_LINK; i = tile.links.get(i).indexOfNextLink) {
                Link link = tile.links.get(i);

                // Find link which contains this edge.
                if (link.indexOfPolyEdge != iresult.segMax) {
                    continue;
                }

                // Get pointer to the next polygon.
                tileAndPolyUns = nav.getTileAndPolyByRefUnsafe(link.neighborRef);
                nextTile = tileAndPolyUns.getFirst();
                nextPoly = tileAndPolyUns.getSecond();
                // Skip off-mesh connections.
                if (nextPoly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
                    continue;
                }

                // Skip links based on filter.
                if (!filter.passFilter(link.neighborRef, nextTile, nextPoly)) {
                    continue;
                }

                // If the link is internal, just return the ref.
                if (link.side == 0xff) {
                    nextRef = link.neighborRef;
                    break;
                }

                // If the link is at tile boundary,

                // Check if the link spans the whole edge, and accept.
                if (link.bmin == 0 && link.bmax == 255) {
                    nextRef = link.neighborRef;
                    break;
                }

                // Check for partial edge links.
                int v0 = poly.vertices[link.indexOfPolyEdge];
                int v1 = poly.vertices[(link.indexOfPolyEdge + 1) % poly.vertCount];
                int left = v0 * 3;
                int right = v1 * 3;

                // Check that the intersection lies inside the link portal.
                float s = 1f / 255f;
                if (link.side == 0 || link.side == 4) {
                    // Calculate link size.
                    float lmin = tile.data.vertices[left + 2]
                            + (tile.data.vertices[right + 2] - tile.data.vertices[left + 2]) * (link.bmin * s);
                    float lmax = tile.data.vertices[left + 2]
                            + (tile.data.vertices[right + 2] - tile.data.vertices[left + 2]) * (link.bmax * s);
                    if (lmin > lmax) {
                        float temp = lmin;
                        lmin = lmax;
                        lmax = temp;
                    }

                    // Find Z intersection.
                    float z = startPos.z + (endPos.z - startPos.z) * iresult.tmax;
                    if (z >= lmin && z <= lmax) {
                        nextRef = link.neighborRef;
                        break;
                    }
                } else if (link.side == 2 || link.side == 6) {
                    // Calculate link size.
                    float lmin = tile.data.vertices[left]
                            + (tile.data.vertices[right] - tile.data.vertices[left]) * (link.bmin * s);
                    float lmax = tile.data.vertices[left]
                            + (tile.data.vertices[right] - tile.data.vertices[left]) * (link.bmax * s);
                    if (lmin > lmax) {
                        float temp = lmin;
                        lmin = lmax;
                        lmax = temp;
                    }

                    // Find X intersection.
                    float x = startPos.x + (endPos.x - startPos.x) * iresult.tmax;
                    if (x >= lmin && x <= lmax) {
                        nextRef = link.neighborRef;
                        break;
                    }
                }
            }

            // add the cost
            if ((options & DT_RAYCAST_USE_COSTS) != 0) {
                // compute the intersection point at the furthest end of the polygon
                // and correct the height (since the raycast moves in 2d)
                copy(lastPos, curPos);
                curPos = mad(startPos, dir, hit.t);
                VectorPtr e1 = new VectorPtr(vertices, iresult.segMax * 3);
                VectorPtr e2 = new VectorPtr(vertices, ((iresult.segMax + 1) % nv) * 3);
                Vector3f eDir = sub(e2, e1);
                Vector3f diff = sub(curPos, e1);
                float s = sqr(eDir.x) > sqr(eDir.z) ? diff.x / eDir.x : diff.z / eDir.z;
                curPos.y = e1.get(1) + eDir.y * s;

                hit.pathCost += filter.getCost(lastPos, curPos, prevRef, prevTile, prevPoly, curRef, tile, poly,
                        nextRef, nextTile, nextPoly);
            }

            if (nextRef == 0) {
                // No neighbour, we hit a wall.
                // Calculate hit normal.
                int a = iresult.segMax;
                int b = iresult.segMax + 1 < nv ? iresult.segMax + 1 : 0;
                int va = a * 3;
                int vb = b * 3;
                hit.hitNormal.x = vertices[vb + 2] - vertices[va + 2];
                hit.hitNormal.y = 0;
                hit.hitNormal.z = -(vertices[vb] - vertices[va]);
                hit.hitNormal.normalize();
                return Result.success(hit);
            }

            // No hit, advance to neighbour polygon.
            prevRef = curRef;
            curRef = nextRef;
            prevTile = tile;
            tile = nextTile;
            prevPoly = poly;
            poly = nextPoly;
        }

        return Result.success(hit);
    }

    /// @par
    ///
    /// At least one result array must be provided.
    ///
    /// The order of the result set is from least to highest cost to reach the polygon.
    ///
    /// A common use case for this method is to perform Dijkstra searches.
    /// Candidate polygons are found by searching the graph beginning at the start polygon.
    ///
    /// If a polygon is not found via the graph search, even if it intersects the
    /// search circle, it will not be included in the result set. For example:
    ///
    /// polyA is the start polygon.
    /// polyB shares an edge with polyA. (Is adjacent.)
    /// polyC shares an edge with polyB, but not with polyA
    /// Even if the search circle overlaps polyC, it will not be included in the
    /// result set unless polyB is also in the set.
    ///
    /// The value of the center point is used as the start position for cost
    /// calculations. It is not projected onto the surface of the mesh, so its
    /// y-value will effect the costs.
    ///
    /// Intersection tests occur in 2D. All polygons and the search circle are
    /// projected onto the xz-plane. So the y-value of the center point does not
    /// effect intersection tests.
    ///
    /// If the result arrays are to small to hold the entire result set, they will be
    /// filled to capacity.
    ///
    /// @}
    /// @name Dijkstra Search Functions
    /// @{

    /// Finds the polygons along the navigation graph that touch the specified circle.
    /// @param[in] startRef The reference id of the polygon where the search starts.
    /// @param[in] centerPos The center of the search circle. [(x, y, z)]
    /// @param[in] radius The radius of the search circle.
    /// @param[in] filter The polygon filter to apply to the query.
    /// @param[out] resultRef The reference ids of the polygons touched by the circle. [opt]
    /// @param[out] resultParent The reference ids of the parent polygons for each result.
    /// Zero if a result polygon has no parent. [opt]
    /// @param[out] resultCost The search cost from @p centerPos to the polygon. [opt]
    /// @param[out] resultCount The number of polygons found. [opt]
    /// @param[in] maxResult The maximum number of polygons the result arrays can hold.
    /// @returns The status flags for the query.
    @SuppressWarnings("unused")
    public Result<FindPolysAroundResult> findPolysAroundCircle(long startRef, Vector3f centerPos, float radius,
                                                               QueryFilter filter) {

        // Validate input

        if (!nav.isValidPolyRef(startRef) || Objects.isNull(centerPos) || !isFinite(centerPos) || radius < 0
                || !Float.isFinite(radius) || Objects.isNull(filter)) {
            return Result.invalidParam();
        }

        LongArrayList resultRef = new LongArrayList();
        LongArrayList resultParent = new LongArrayList();
        FloatArrayList resultCost = new FloatArrayList();

        nodePool.clear();
        openList.clear();

        Node startNode = nodePool.getNode(startRef);
        copy(startNode.pos, centerPos);
        startNode.parentIndex = 0;
        startNode.cost = 0;
        startNode.totalCost = 0;
        startNode.polygonRef = startRef;
        startNode.flags = Node.OPEN;
        openList.offer(startNode);

        float radiusSqr = sqr(radius);

        while (!openList.isEmpty()) {
            Node bestNode = openList.poll();
            bestNode.flags &= ~Node.OPEN;
            bestNode.flags |= Node.CLOSED;

            // Get poly and tile.
            // The API input has been cheked already, skip checking internal data.
            long bestRef = bestNode.polygonRef;
            Pair<MeshTile, Poly> tileAndPoly = nav.getTileAndPolyByRefUnsafe(bestRef);
            MeshTile bestTile = tileAndPoly.getFirst();
            Poly bestPoly = tileAndPoly.getSecond();

            // Get parent poly and tile.
            long parentRef = 0;
            MeshTile parentTile = null;
            Poly parentPoly = null;
            if (bestNode.parentIndex != 0) {
                parentRef = nodePool.getNodeAtIdx(bestNode.parentIndex).polygonRef;
            }
            if (parentRef != 0) {
                tileAndPoly = nav.getTileAndPolyByRefUnsafe(parentRef);
                parentTile = tileAndPoly.getFirst();
                parentPoly = tileAndPoly.getSecond();
            }

            resultRef.add(bestRef);
            resultParent.add(parentRef);
            resultCost.add(bestNode.totalCost);

            for (int i = bestTile.polyLinks[bestPoly.index]; i != NavMesh.DT_NULL_LINK; i = bestTile.links.get(i).indexOfNextLink) {
                Link link = bestTile.links.get(i);
                long neighbourRef = link.neighborRef;
                // Skip invalid neighbours and do not follow back to parent.
                if (neighbourRef == 0 || neighbourRef == parentRef) {
                    continue;
                }

                // Expand to neighbour
                tileAndPoly = nav.getTileAndPolyByRefUnsafe(neighbourRef);
                MeshTile neighbourTile = tileAndPoly.getFirst();
                Poly neighbourPoly = tileAndPoly.getSecond();

                // Do not advance if the polygon is excluded by the filter.
                if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    continue;
                }

                // Find edge and calc distance to the edge.
                Result<PortalResult> pp = getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly,
                        neighbourTile, 0, 0);
                if (pp.failed()) {
                    continue;
                }
                Vector3f va = pp.result.left;
                Vector3f vb = pp.result.right;

                // If the circle is not touching the next polygon, skip it.
                Pair<Float, Float> distseg = distancePtSegSqr2D(centerPos, va, vb);
                float distSqr = distseg.getFirst();
                if (distSqr > radiusSqr) {
                    continue;
                }

                Node neighbourNode = nodePool.getNode(neighbourRef);

                if ((neighbourNode.flags & Node.CLOSED) != 0) {
                    continue;
                }

                // Cost
                if (neighbourNode.flags == 0) {
                    neighbourNode.pos = lerp(va, vb, 0.5f);
                }

                float cost = filter.getCost(bestNode.pos, neighbourNode.pos, parentRef, parentTile, parentPoly, bestRef,
                        bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly);

                float total = bestNode.totalCost + cost;
                // The node is already in open list and the new result is worse, skip.
                if ((neighbourNode.flags & Node.OPEN) != 0 && total >= neighbourNode.totalCost) {
                    continue;
                }

                neighbourNode.polygonRef = neighbourRef;
                neighbourNode.parentIndex = nodePool.getNodeIdx(bestNode);
                neighbourNode.totalCost = total;

                if ((neighbourNode.flags & Node.OPEN) != 0) {
                    openList.remove(neighbourNode);
                    openList.offer(neighbourNode);
                } else {
                    neighbourNode.flags = Node.OPEN;
                    openList.offer(neighbourNode);
                }
            }
        }

        return Result.success(new FindPolysAroundResult(resultRef, resultParent, resultCost));
    }

    /// @par
    ///
    /// The order of the result set is from least to highest cost.
    ///
    /// At least one result array must be provided.
    ///
    /// A common use case for this method is to perform Dijkstra searches.
    /// Candidate polygons are found by searching the graph beginning at the start
    /// polygon.
    ///
    /// The same intersection test restrictions that apply to findPolysAroundCircle()
    /// method apply to this method.
    ///
    /// The 3D centroid of the search polygon is used as the start position for cost
    /// calculations.
    ///
    /// Intersection tests occur in 2D. All polygons are projected onto the
    /// xz-plane. So the y-values of the vertices do not effect intersection tests.
    ///
    /// If the result arrays are is too small to hold the entire result set, they will
    /// be filled to capacity.
    ///
    /// Finds the polygons along the naviation graph that touch the specified convex polygon.
    /// @param[in] startRef The reference id of the polygon where the search starts.
    /// @param[in] vertices The vertices describing the convex polygon. (CCW)
    /// [(x, y, z) * @p nvertices]
    /// @param[in] nvertices The number of vertices in the polygon.
    /// @param[in] filter The polygon filter to apply to the query.
    /// @param[out] resultRef The reference ids of the polygons touched by the search polygon. [opt]
    /// @param[out] resultParent The reference ids of the parent polygons for each result. Zero if a
    /// result polygon has no parent. [opt]
    /// @param[out] resultCost The search cost from the centroid point to the polygon. [opt]
    /// @param[out] resultCount The number of polygons found.
    /// @param[in] maxResult The maximum number of polygons the result arrays can hold.
    /// @returns The status flags for the query.
    @SuppressWarnings("unused")
    public Result<FindPolysAroundResult> findPolysAroundShape(long startRef, float[] vertices, QueryFilter filter) {
        // Validate input
        int nvertices = vertices.length / 3;
        if (!nav.isValidPolyRef(startRef) || nvertices < 3 || Objects.isNull(filter)) {
            return Result.invalidParam();
        }

        LongArrayList resultRef = new LongArrayList();
        LongArrayList resultParent = new LongArrayList();
        FloatArrayList resultCost = new FloatArrayList();

        nodePool.clear();
        openList.clear();

        Vector3f centerPos = new Vector3f();
        for (int i = 0; i < nvertices; ++i) {
            centerPos.x += vertices[i * 3];
            centerPos.y += vertices[i * 3 + 1];
            centerPos.z += vertices[i * 3 + 2];
        }
        centerPos.div(nvertices);

        Node startNode = nodePool.getNode(startRef);
        copy(startNode.pos, centerPos);
        startNode.parentIndex = 0;
        startNode.cost = 0;
        startNode.totalCost = 0;
        startNode.polygonRef = startRef;
        startNode.flags = Node.OPEN;
        openList.offer(startNode);

        while (!openList.isEmpty()) {
            Node bestNode = openList.poll();
            bestNode.flags &= ~Node.OPEN;
            bestNode.flags |= Node.CLOSED;

            // Get poly and tile.
            // The API input has been checked already, skip checking internal data.
            long bestRef = bestNode.polygonRef;
            Pair<MeshTile, Poly> tileAndPoly = nav.getTileAndPolyByRefUnsafe(bestRef);
            MeshTile bestTile = tileAndPoly.getFirst();
            Poly bestPoly = tileAndPoly.getSecond();

            // Get parent poly and tile.
            long parentRef = 0;
            MeshTile parentTile = null;
            Poly parentPoly = null;
            if (bestNode.parentIndex != 0) {
                parentRef = nodePool.getNodeAtIdx(bestNode.parentIndex).polygonRef;
            }
            if (parentRef != 0) {
                tileAndPoly = nav.getTileAndPolyByRefUnsafe(parentRef);
                parentTile = tileAndPoly.getFirst();
                parentPoly = tileAndPoly.getSecond();
            }

            resultRef.add(bestRef);
            resultParent.add(parentRef);
            resultCost.add(bestNode.totalCost);

            for (int i = bestTile.polyLinks[bestPoly.index]; i != NavMesh.DT_NULL_LINK; i = bestTile.links.get(i).indexOfNextLink) {
                Link link = bestTile.links.get(i);
                long neighbourRef = link.neighborRef;
                // Skip invalid neighbours and do not follow back to parent.
                if (neighbourRef == 0 || neighbourRef == parentRef) {
                    continue;
                }

                // Expand to neighbour
                tileAndPoly = nav.getTileAndPolyByRefUnsafe(neighbourRef);
                MeshTile neighbourTile = tileAndPoly.getFirst();
                Poly neighbourPoly = tileAndPoly.getSecond();

                // Do not advance if the polygon is excluded by the filter.
                if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    continue;
                }

                // Find edge and calc distance to the edge.
                Result<PortalResult> pp = getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly,
                        neighbourTile, 0, 0);
                if (pp.failed()) {
                    continue;
                }
                Vector3f va = pp.result.left;
                Vector3f vb = pp.result.right;

                // If the poly is not touching the edge to the next polygon, skip the connection it.
                IntersectResult ir = intersectSegmentPoly2D(va, vb, vertices, nvertices);
                if (!ir.intersects) {
                    continue;
                }
                if (ir.tmin > 1f || ir.tmax < 0f) {
                    continue;
                }

                Node neighbourNode = nodePool.getNode(neighbourRef);

                if ((neighbourNode.flags & Node.CLOSED) != 0) {
                    continue;
                }

                // Cost
                if (neighbourNode.flags == 0) {
                    neighbourNode.pos = lerp(va, vb, 0.5f);
                }

                float cost = filter.getCost(bestNode.pos, neighbourNode.pos, parentRef, parentTile, parentPoly, bestRef,
                        bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly);

                float total = bestNode.totalCost + cost;

                // The node is already in open list and the new result is worse, skip.
                if ((neighbourNode.flags & Node.OPEN) != 0 && total >= neighbourNode.totalCost) {
                    continue;
                }

                neighbourNode.polygonRef = neighbourRef;
                neighbourNode.parentIndex = nodePool.getNodeIdx(bestNode);
                neighbourNode.totalCost = total;

                if ((neighbourNode.flags & Node.OPEN) != 0) {
                    openList.remove(neighbourNode);
                    openList.offer(neighbourNode);
                } else {
                    neighbourNode.flags = Node.OPEN;
                    openList.offer(neighbourNode);
                }

            }
        }

        return Result.success(new FindPolysAroundResult(resultRef, resultParent, resultCost));
    }

    /// @par
    ///
    /// This method is optimized for a small search radius and small number of result
    /// polygons.
    ///
    /// Candidate polygons are found by searching the navigation graph beginning at
    /// the start polygon.
    ///
    /// The same intersection test restrictions that apply to the findPolysAroundCircle
    /// mehtod applies to this method.
    ///
    /// The value of the center point is used as the start point for cost calculations.
    /// It is not projected onto the surface of the mesh, so its y-value will effect
    /// the costs.
    ///
    /// Intersection tests occur in 2D. All polygons and the search circle are
    /// projected onto the xz-plane. So the y-value of the center point does not
    /// effect intersection tests.
    ///
    /// If the result arrays are is too small to hold the entire result set, they will
    /// be filled to capacity.
    ///
    /// Finds the non-overlapping navigation polygons in the local neighbourhood around the center position.
    /// @param[in] startRef The reference id of the polygon where the search starts.
    /// @param[in] centerPos The center of the query circle. [(x, y, z)]
    /// @param[in] radius The radius of the query circle.
    /// @param[in] filter The polygon filter to apply to the query.
    /// @param[out] resultRef The reference ids of the polygons touched by the circle.
    /// @param[out] resultParent The reference ids of the parent polygons for each result.
    /// Zero if a result polygon has no parent. [opt]
    /// @param[out] resultCount The number of polygons found.
    /// @param[in] maxResult The maximum number of polygons the result arrays can hold.
    /// @returns The status flags for the query.
    public Result<FindLocalNeighbourhoodResult> findLocalNeighbourhood(long startRef, Vector3f centerPos, float radius,
                                                                       QueryFilter filter) {

        // Validate input
        if (!nav.isValidPolyRef(startRef) || Objects.isNull(centerPos) || !isFinite(centerPos) || radius < 0
                || !Float.isFinite(radius) || Objects.isNull(filter)) {
            return Result.invalidParam();
        }

        LongArrayList resultRef = new LongArrayList();
        LongArrayList resultParent = new LongArrayList();

        NodePool tinyNodePool = new NodePool();

        Node startNode = tinyNodePool.getNode(startRef);
        startNode.parentIndex = 0;
        startNode.polygonRef = startRef;
        startNode.flags = Node.CLOSED;
        LinkedList<Node> stack = new LinkedList<>();
        stack.add(startNode);

        resultRef.add(startNode.polygonRef);
        resultParent.add(0L);

        float radiusSqr = sqr(radius);

        float[] pa = new float[nav.maxVerticesPerPoly * 3];
        float[] pb = new float[nav.maxVerticesPerPoly * 3];

        while (!stack.isEmpty()) {
            // Pop front.
            Node curNode = stack.pop();

            // Get poly and tile.
            // The API input has been cheked already, skip checking internal data.
            long curRef = curNode.polygonRef;
            Pair<MeshTile, Poly> tileAndPoly = nav.getTileAndPolyByRefUnsafe(curRef);
            MeshTile curTile = tileAndPoly.getFirst();
            Poly curPoly = tileAndPoly.getSecond();

            for (int i = curTile.polyLinks[curPoly.index]; i != NavMesh.DT_NULL_LINK; i = curTile.links.get(i).indexOfNextLink) {
                Link link = curTile.links.get(i);
                long neighbourRef = link.neighborRef;
                // Skip invalid neighbours.
                if (neighbourRef == 0) {
                    continue;
                }

                Node neighbourNode = tinyNodePool.getNode(neighbourRef);
                // Skip visited.
                if ((neighbourNode.flags & Node.CLOSED) != 0) {
                    continue;
                }

                // Expand to neighbour
                tileAndPoly = nav.getTileAndPolyByRefUnsafe(neighbourRef);
                MeshTile neighbourTile = tileAndPoly.getFirst();
                Poly neighbourPoly = tileAndPoly.getSecond();

                // Skip off-mesh connections.
                if (neighbourPoly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
                    continue;
                }

                // Do not advance if the polygon is excluded by the filter.
                if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    continue;
                }

                // Find edge and calc distance to the edge.
                Result<PortalResult> pp = getPortalPoints(curRef, curPoly, curTile, neighbourRef, neighbourPoly,
                        neighbourTile, 0, 0);
                if (pp.failed()) {
                    continue;
                }
                Vector3f va = pp.result.left;
                Vector3f vb = pp.result.right;

                // If the circle is not touching the next polygon, skip it.
                Pair<Float, Float> distseg = distancePtSegSqr2D(centerPos, va, vb);
                float distSqr = distseg.getFirst();
                if (distSqr > radiusSqr) {
                    continue;
                }

                // Mark node visited, this is done before the overlap test so that
                // we will not visit the poly again if the test fails.
                neighbourNode.flags |= Node.CLOSED;
                neighbourNode.parentIndex = tinyNodePool.getNodeIdx(curNode);

                // Check, that the polygon does not collide with existing polygons.

                // Collect vertices of the neighbour poly.
                int npa = neighbourPoly.vertCount;
                for (int k = 0; k < npa; ++k) {
                    System.arraycopy(neighbourTile.data.vertices, neighbourPoly.vertices[k] * 3, pa, k * 3, 3);
                }

                boolean overlap = false;
                for (int idx = 0, len = resultRef.getSize(); idx < len; idx++) {
                    long pastRef = resultRef.get(idx);
                    // Connected polys do not overlap.
                    boolean connected = false;
                    for (int k = curTile.polyLinks[curPoly.index]; k != NavMesh.DT_NULL_LINK; k = curTile.links.get(k).indexOfNextLink) {
                        if (curTile.links.get(k).neighborRef == pastRef) {
                            connected = true;
                            break;
                        }
                    }
                    if (connected) {
                        continue;
                    }

                    // Potentially overlapping.
                    tileAndPoly = nav.getTileAndPolyByRefUnsafe(pastRef);
                    MeshTile pastTile = tileAndPoly.getFirst();
                    Poly pastPoly = tileAndPoly.getSecond();

                    // Get vertices and test overlap
                    int npb = pastPoly.vertCount;
                    for (int k = 0; k < npb; ++k) {
                        System.arraycopy(pastTile.data.vertices, pastPoly.vertices[k] * 3, pb, k * 3, 3);
                    }

                    if (overlapPolyPoly2D(pa, npa, pb, npb)) {
                        overlap = true;
                        break;
                    }
                }
                if (overlap) {
                    continue;
                }

                resultRef.add(neighbourRef);
                resultParent.add(curRef);
                stack.add(neighbourNode);
            }
        }

        return Result.success(new FindLocalNeighbourhoodResult(resultRef, resultParent));
    }

    private static class SegInterval {
        long ref;
        int tmin, tmax;

        public SegInterval(long ref, int tmin, int tmax) {
            this.ref = ref;
            this.tmin = tmin;
            this.tmax = tmax;
        }

    }

    protected void insertInterval(List<SegInterval> ints, int tmin, int tmax, long ref) {
        // Find insertion point.
        int idx = 0;
        while (idx < ints.size()) {
            if (tmax <= ints.get(idx).tmin) {
                break;
            }
            idx++;
        }
        // Store
        ints.add(idx, new SegInterval(ref, tmin, tmax));
    }

    /// @par
    ///
    /// If the @p segmentRefs parameter is provided, then all polygon segments will be returned.
    /// Otherwise only the wall segments are returned.
    ///
    /// A segment that is normally a portal will be included in the result set as a
    /// wall if the @p filter results in the neighbor polygon becoomming impassable.
    ///
    /// The @p segmentVertices and @p segmentRefs buffers should normally be sized for the
    /// maximum segments per polygon of the source navigation mesh.
    ///
    /// Returns the segments for the specified polygon, optionally including portals.
    /// @param[in] ref The reference id of the polygon.
    /// @param[in] filter The polygon filter to apply to the query.
    /// @param[out] segmentVertices The segments. [(ax, ay, az, bx, by, bz) * segmentCount]
    /// @param[out] segmentRefs The reference ids of each segment's neighbor polygon.
    /// Or zero if the segment is a wall. [opt] [(parentRef) * @p segmentCount]
    /// @param[out] segmentCount The number of segments returned.
    /// @param[in] maxSegments The maximum number of segments the result arrays can hold.
    /// @returns The status flags for the query.
    public Result<GetPolyWallSegmentsResult> getPolyWallSegments(long ref, boolean storePortals, QueryFilter filter) {
        Result<Pair<MeshTile, Poly>> tileAndPoly = nav.getTileAndPolyByRef(ref);
        if (tileAndPoly.failed()) {
            return Result.of(tileAndPoly.status, tileAndPoly.message);
        }
        if (Objects.isNull(filter)) {
            return Result.invalidParam();
        }
        MeshTile tile = tileAndPoly.result.getFirst();
        Poly poly = tileAndPoly.result.getSecond();

        LongArrayList segmentRefs = new LongArrayList();
        List<float[]> segmentVertices = new ArrayList<>();
        List<SegInterval> ints = new ArrayList<>(16);

        for (int i = 0, j = poly.vertCount - 1; i < poly.vertCount; j = i++) {
            // Skip non-solid edges.
            ints.clear();
            if ((poly.neighborData[j] & NavMesh.DT_EXT_LINK) != 0) {
                // Tile border.
                for (int k = tile.polyLinks[poly.index]; k != NavMesh.DT_NULL_LINK; k = tile.links.get(k).indexOfNextLink) {
                    Link link = tile.links.get(k);
                    if (link.indexOfPolyEdge == j) {
                        if (link.neighborRef != 0) {
                            Pair<MeshTile, Poly> tileAndPolyUnsafe = nav.getTileAndPolyByRefUnsafe(link.neighborRef);
                            MeshTile neiTile = tileAndPolyUnsafe.getFirst();
                            Poly neiPoly = tileAndPolyUnsafe.getSecond();
                            if (filter.passFilter(link.neighborRef, neiTile, neiPoly)) {
                                insertInterval(ints, link.bmin, link.bmax, link.neighborRef);
                            }
                        }
                    }
                }
            } else {
                // Internal edge
                long neiRef = 0;
                if (poly.neighborData[j] != 0) {
                    int idx = (poly.neighborData[j] - 1);
                    neiRef = nav.getPolyRefBase(tile) | idx;
                    if (!filter.passFilter(neiRef, tile, tile.data.polygons[idx])) {
                        neiRef = 0;
                    }
                }
                // If the edge leads to another polygon and portals are not stored, skip.
                if (neiRef != 0 && !storePortals) {
                    continue;
                }

                int vj = poly.vertices[j] * 3;
                int vi = poly.vertices[i] * 3;
                float[] seg = new float[6];
                System.arraycopy(tile.data.vertices, vj, seg, 0, 3);
                System.arraycopy(tile.data.vertices, vi, seg, 3, 3);
                segmentVertices.add(seg);
                segmentRefs.add(neiRef);
                continue;
            }

            // Add sentinels
            insertInterval(ints, -1, 0, 0);
            insertInterval(ints, 255, 256, 0);

            // Store segments.
            int vj = poly.vertices[j] * 3;
            int vi = poly.vertices[i] * 3;
            for (int k = 1; k < ints.size(); ++k) {
                // Portal segment.
                if (storePortals && ints.get(k).ref != 0) {
                    float tmin = ints.get(k).tmin / 255f;
                    float tmax = ints.get(k).tmax / 255f;
                    float[] seg = new float[6];
                    copy(seg, 0, lerp(tile.data.vertices, vj, vi, tmin));
                    copy(seg, 3, lerp(tile.data.vertices, vj, vi, tmax));
                    segmentVertices.add(seg);
                    segmentRefs.add(ints.get(k).ref);
                }

                // Wall segment.
                int imin = ints.get(k - 1).tmax;
                int imax = ints.get(k).tmin;
                if (imin != imax) {
                    float tmin = imin / 255f;
                    float tmax = imax / 255f;
                    float[] seg = new float[6];
                    copy(seg, 0, lerp(tile.data.vertices, vj, vi, tmin));
                    copy(seg, 3, lerp(tile.data.vertices, vj, vi, tmax));
                    segmentVertices.add(seg);
                    segmentRefs.add(0L);
                }
            }
        }

        return Result.success(new GetPolyWallSegmentsResult(segmentVertices, segmentRefs));
    }

    /// @par
    ///
    /// @p hitPos is not adjusted using the height detail data.
    ///
    /// @p hitDist will equal the search radius if there is no wall within the
    /// radius. In this case the values of @p hitPos and @p hitNormal are
    /// undefined.
    ///
    /// The normal will become unpredicable if @p hitDist is a very small number.
    ///
    /// Finds the distance from the specified position to the nearest polygon wall.
    /// @param[in] startRef The reference id of the polygon containing @p centerPos.
    /// @param[in] centerPos The center of the search circle. [(x, y, z)]
    /// @param[in] maxRadius The radius of the search circle.
    /// @param[in] filter The polygon filter to apply to the query.
    /// @param[out] hitDist The distance to the nearest wall from @p centerPos.
    /// @param[out] hitPos The nearest position on the wall that was hit. [(x, y, z)]
    /// @param[out] hitNormal The normalized ray formed from the wall point to the
    /// source point. [(x, y, z)]
    /// @returns The status flags for the query.
    @SuppressWarnings("unused")
    public Result<FindDistanceToWallResult> findDistanceToWall(long startRef, Vector3f centerPos, float maxRadius,
                                                               QueryFilter filter) {

        // Validate input
        if (!nav.isValidPolyRef(startRef) || Objects.isNull(centerPos) || !isFinite(centerPos) || maxRadius < 0
                || !Float.isFinite(maxRadius) || Objects.isNull(filter)) {
            return Result.invalidParam();
        }

        nodePool.clear();
        openList.clear();

        Node startNode = nodePool.getNode(startRef);
        copy(startNode.pos, centerPos);
        startNode.parentIndex = 0;
        startNode.cost = 0;
        startNode.totalCost = 0;
        startNode.polygonRef = startRef;
        startNode.flags = Node.OPEN;
        openList.offer(startNode);

        float radiusSqr = sqr(maxRadius);
        Vector3f hitPos = new Vector3f();
        VectorPtr bestvj = null;
        VectorPtr bestvi = null;
        while (!openList.isEmpty()) {
            Node bestNode = openList.poll();
            bestNode.flags &= ~Node.OPEN;
            bestNode.flags |= Node.CLOSED;

            // Get poly and tile.
            // The API input has been checked already, skip checking internal data.
            long bestRef = bestNode.polygonRef;
            Pair<MeshTile, Poly> tileAndPoly = nav.getTileAndPolyByRefUnsafe(bestRef);
            MeshTile bestTile = tileAndPoly.getFirst();
            Poly bestPoly = tileAndPoly.getSecond();

            // Get parent poly and tile.
            long parentRef = 0;
            if (bestNode.parentIndex != 0) {
                parentRef = nodePool.getNodeAtIdx(bestNode.parentIndex).polygonRef;
            }

            // Hit test walls.
            for (int i = 0, j = bestPoly.vertCount - 1; i < bestPoly.vertCount; j = i++) {
                // Skip non-solid edges.
                if ((bestPoly.neighborData[j] & NavMesh.DT_EXT_LINK) != 0) {
                    // Tile border.
                    boolean solid = true;
                    for (int k = bestTile.polyLinks[bestPoly.index]; k != NavMesh.DT_NULL_LINK; k = bestTile.links.get(k).indexOfNextLink) {
                        Link link = bestTile.links.get(k);
                        if (link.indexOfPolyEdge == j) {
                            if (link.neighborRef != 0) {
                                Pair<MeshTile, Poly> linkTileAndPoly = nav.getTileAndPolyByRefUnsafe(link.neighborRef);
                                MeshTile neiTile = linkTileAndPoly.getFirst();
                                Poly neiPoly = linkTileAndPoly.getSecond();
                                if (filter.passFilter(link.neighborRef, neiTile, neiPoly)) {
                                    solid = false;
                                }
                            }
                            break;
                        }
                    }
                    if (!solid) {
                        continue;
                    }
                } else if (bestPoly.neighborData[j] != 0) {
                    // Internal edge
                    int idx = (bestPoly.neighborData[j] - 1);
                    long ref = nav.getPolyRefBase(bestTile) | idx;
                    if (filter.passFilter(ref, bestTile, bestTile.data.polygons[idx])) {
                        continue;
                    }
                }

                // Calc distance to the edge.
                int vj = bestPoly.vertices[j] * 3;
                int vi = bestPoly.vertices[i] * 3;
                Pair<Float, Float> distseg = distancePtSegSqr2D(centerPos, bestTile.data.vertices, vj, vi);
                float distSqr = distseg.getFirst();
                float tseg = distseg.getSecond();

                // Edge is too far, skip.
                if (distSqr > radiusSqr) {
                    continue;
                }

                // Hit wall, update radius.
                radiusSqr = distSqr;
                // Calculate hit pos.
                hitPos.x = bestTile.data.vertices[vj] + (bestTile.data.vertices[vi] - bestTile.data.vertices[vj]) * tseg;
                hitPos.y = bestTile.data.vertices[vj + 1] + (bestTile.data.vertices[vi + 1] - bestTile.data.vertices[vj + 1]) * tseg;
                hitPos.z = bestTile.data.vertices[vj + 2] + (bestTile.data.vertices[vi + 2] - bestTile.data.vertices[vj + 2]) * tseg;
                bestvj = new VectorPtr(bestTile.data.vertices, vj);
                bestvi = new VectorPtr(bestTile.data.vertices, vi);
            }

            for (int i = bestTile.polyLinks[bestPoly.index]; i != NavMesh.DT_NULL_LINK; i = bestTile.links.get(i).indexOfNextLink) {
                Link link = bestTile.links.get(i);
                long neighbourRef = link.neighborRef;
                // Skip invalid neighbours and do not follow back to parent.
                if (neighbourRef == 0 || neighbourRef == parentRef) {
                    continue;
                }

                // Expand to neighbour.
                Pair<MeshTile, Poly> neighbourTileAndPoly = nav.getTileAndPolyByRefUnsafe(neighbourRef);
                MeshTile neighbourTile = neighbourTileAndPoly.getFirst();
                Poly neighbourPoly = neighbourTileAndPoly.getSecond();

                // Skip off-mesh connections.
                if (neighbourPoly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
                    continue;
                }

                // Calc distance to the edge.
                int va = bestPoly.vertices[link.indexOfPolyEdge] * 3;
                int vb = bestPoly.vertices[(link.indexOfPolyEdge + 1) % bestPoly.vertCount] * 3;
                Pair<Float, Float> distseg = distancePtSegSqr2D(centerPos, bestTile.data.vertices, va, vb);
                float distSqr = distseg.getFirst();
                // If the circle is not touching the next polygon, skip it.
                if (distSqr > radiusSqr) {
                    continue;
                }

                if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    continue;
                }

                Node neighbourNode = nodePool.getNode(neighbourRef);

                if ((neighbourNode.flags & Node.CLOSED) != 0) {
                    continue;
                }

                // Cost
                if (neighbourNode.flags == 0) {
                    Result<Vector3f> midPoint = getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile);
                    if (midPoint.succeeded()) {
                        neighbourNode.pos = midPoint.result;
                    }
                }

                float total = bestNode.totalCost + bestNode.pos.distance(neighbourNode.pos);

                // The node is already in open list and the new result is worse, skip.
                if ((neighbourNode.flags & Node.OPEN) != 0 && total >= neighbourNode.totalCost) {
                    continue;
                }

                neighbourNode.polygonRef = neighbourRef;
                neighbourNode.flags = (neighbourNode.flags & ~Node.CLOSED);
                neighbourNode.parentIndex = nodePool.getNodeIdx(bestNode);
                neighbourNode.totalCost = total;

                if ((neighbourNode.flags & Node.OPEN) != 0) {
                    openList.remove(neighbourNode);
                    openList.offer(neighbourNode);
                } else {
                    neighbourNode.flags |= Node.OPEN;
                    openList.offer(neighbourNode);
                }
            }
        }

        // Calc hit normal.
        Vector3f hitNormal = new Vector3f();
        if (bestvi != null) {
            Vector3f tangent = sub(bestvi, bestvj);
            hitNormal.x = tangent.z;
            hitNormal.y = 0;
            hitNormal.z = -tangent.x;
            hitNormal.normalize();
        }
        return Result.success(new FindDistanceToWallResult((float) Math.sqrt(radiusSqr), hitPos, hitNormal));
    }

    /// Returns true if the polygon reference is valid and passes the filter restrictions.
    /// @param[in] ref The polygon reference to check.
    /// @param[in] filter The filter to apply.
    public boolean isValidPolyRef(long ref, QueryFilter filter) {
        Result<Pair<MeshTile, Poly>> tileAndPolyResult = nav.getTileAndPolyByRef(ref);
        if (tileAndPolyResult.failed()) {
            return false;
        }
        Pair<MeshTile, Poly> tileAndPoly = tileAndPolyResult.result;
        // If cannot pass filter, assume flags has changed and boundary is invalid.
        return filter.passFilter(ref, tileAndPoly.getFirst(), tileAndPoly.getSecond());
    }

    /// Gets the navigation mesh the query object is using.
    /// @return The navigation mesh the query object is using.
    public NavMesh getAttachedNavMesh() {
        return nav;
    }

    /**
     * Gets a path from the explored nodes in the previous search.
     *
     * @param endRef The reference id of the end polygon.
     * @returns An ordered list of polygon references representing the path. (Start to end.)
     * @remarks The result of this function depends on the state of the query object. For that reason it should only be
     * used immediately after one of the two Dijkstra searches, findPolysAroundCircle or findPolysAroundShape.
     */
    @SuppressWarnings("unused")
    public Result<LongArrayList> getPathFromDijkstraSearch(long endRef) {
        if (!nav.isValidPolyRef(endRef)) {
            return Result.invalidParam("Invalid end ref");
        }
        List<Node> nodes = nodePool.findNodes(endRef);
        if (nodes.size() != 1) {
            return Result.invalidParam("Invalid end ref");
        }
        Node endNode = nodes.get(0);
        if ((endNode.flags & CLOSED) == 0) {
            return Result.invalidParam("Invalid end ref");
        }
        return Result.success(getPathToNode(endNode));
    }

    /**
     * Gets the path leading to the specified end node.
     */
    protected LongArrayList getPathToNode(Node endNode) {
        LongArrayList path = new LongArrayList();
        // Reverse the path.
        Node curNode = endNode;
        do {
            path.add(curNode.polygonRef);
            Node nextNode = nodePool.getNodeAtIdx(curNode.parentIndex);
            if (curNode.shortcut != null) {
                // remove potential duplicates from shortcut path
                for (int i = curNode.shortcut.getSize() - 1; i >= 0; i--) {
                    long id = curNode.shortcut.get(i);
                    if (id != curNode.polygonRef && id != nextNode.polygonRef) {
                        path.add(id);
                    }
                }
            }
            curNode = nextNode;
        } while (curNode != null);
        path.reverse();
        return path;
    }

    /**
     * The closed list is the list of polygons that were fully evaluated during the last navigation graph search. (A* or
     * Dijkstra)
     */
    @SuppressWarnings("unused")
    public boolean isInClosedList(long ref) {
        for (Node n : nodePool.findNodes(ref)) {
            if ((n.flags & CLOSED) != 0) {
                return true;
            }
        }
        return false;
    }

}