/*
recast4j copyright (c) 2021 Piotr Piastucki piotr@jtilia.org

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
import org.recast4j.LongArrayList;
import org.recast4j.Pair;

import java.util.Objects;

import static org.recast4j.Vectors.*;

@SuppressWarnings("unused")
public class LegacyNavMeshQuery extends NavMeshQuery {

    private static final float H_SCALE = 0.999f; // Search heuristic scale.

    @SuppressWarnings("unused")
    public LegacyNavMeshQuery(NavMesh nav) {
        super(nav);
    }

    @Override
    public Result<LongArrayList> findPath(long startRef, long endRef, Vector3f startPos, Vector3f endPos, QueryFilter filter,
                                          int options, float raycastLimit) {
        return findPath(startRef, endRef, startPos, endPos, filter);
    }

    @Override
    public Result<LongArrayList> findPath(long startRef, long endRef, Vector3f startPos, Vector3f endPos,
                                          QueryFilter filter) {
        // Validate input
        if (!nav.isValidPolyRef(startRef) || !nav.isValidPolyRef(endRef) || !isFinite(startPos) || !isFinite(endPos)) {
            return Result.invalidParam();
        }

        if (startRef == endRef) {
            LongArrayList path = new LongArrayList(1);
            path.add(startRef);
            return Result.success(path);
        }

        nodePool.clear();
        openList.clear();

        Node startNode = nodePool.getNode(startRef);
        copy(startNode.pos, startPos);
        startNode.parentIndex = 0;
        startNode.cost = 0;
        startNode.totalCost = startPos.distance(endPos) * H_SCALE;
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
            MeshTile bestTile = tileAndPoly.first;
            Poly bestPoly = tileAndPoly.second;

            // Get parent poly and tile.
            long parentRef = 0;
            MeshTile parentTile = null;
            Poly parentPoly = null;
            if (bestNode.parentIndex != 0) {
                parentRef = nodePool.getNodeAtIdx(bestNode.parentIndex).polygonRef;
            }
            if (parentRef != 0) {
                tileAndPoly = nav.getTileAndPolyByRefUnsafe(parentRef);
                parentTile = tileAndPoly.first;
                parentPoly = tileAndPoly.second;
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
                MeshTile neighbourTile = tileAndPoly.first;
                Poly neighbourPoly = tileAndPoly.second;

                if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    continue;
                }

                // deal explicitly with crossing tile boundaries
                int crossSide = 0;
                if (bestTile.links.get(i).side != 0xff) {
                    crossSide = bestTile.links.get(i).side >> 1;
                }

                // get the node
                Node neighbourNode = nodePool.getNode(neighbourRef, crossSide);

                // If the node is visited the first time, calculate node position.
                if (neighbourNode.flags == 0) {
                    Result<Vector3f> midpod = getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile);
                    if (!midpod.failed()) {
                        neighbourNode.pos = midpod.result;
                    }
                }

                // Calculate cost and heuristic.
                float cost;
                float heuristic;

                // Special case for last node.
                float curCost = filter.getCost(bestNode.pos, neighbourNode.pos, parentRef, parentTile, parentPoly,
                        bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly);
                if (neighbourRef == endRef) {
                    // Cost
                    float endCost = filter.getCost(neighbourNode.pos, endPos, bestRef, bestTile, bestPoly, neighbourRef,
                            neighbourTile, neighbourPoly, 0L, null, null);
                    cost = bestNode.cost + curCost + endCost;
                    heuristic = 0;
                } else {
                    // Cost
                    cost = bestNode.cost + curCost;
                    heuristic = neighbourNode.pos.distance(endPos) * H_SCALE;
                }

                float total = cost + heuristic;

                // The node is already in open list and the new result is worse, skip.
                if ((neighbourNode.flags & Node.OPEN) != 0 && total >= neighbourNode.totalCost) {
                    continue;
                }
                // The node is already visited and process, and the new result is worse, skip.
                if ((neighbourNode.flags & Node.CLOSED) != 0 && total >= neighbourNode.totalCost) {
                    continue;
                }

                // Add or update the node.
                neighbourNode.parentIndex = nodePool.getNodeIdx(bestNode);
                neighbourNode.polygonRef = neighbourRef;
                neighbourNode.flags = (neighbourNode.flags & ~Node.CLOSED);
                neighbourNode.cost = cost;
                neighbourNode.totalCost = total;

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
                if (heuristic < lastBestNodeCost) {
                    lastBestNodeCost = heuristic;
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
     * Updates an in-progress sliced path query.
     *
     * @param maxIter The maximum number of iterations to perform.
     * @return The status flags for the query.
     */
    @Override
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
            MeshTile bestTile = tileAndPoly.result.first;
            Poly bestPoly = tileAndPoly.result.second;
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
                if (tileAndPoly.failed() || (grandpaRef != 0 && !nav.isValidPolyRef(grandpaRef))) {
                    // The polygon has disappeared during the sliced query,
                    // fail.
                    queryData.status = Status.FAILURE;
                    return Result.of(queryData.status, iter);
                }
                parentTile = tileAndPoly.result.first;
                parentPoly = tileAndPoly.result.second;
            }

            // decide whether to test raycast to previous nodes
            boolean tryLOS = false;
            if ((queryData.options & DT_FINDPATH_ANY_ANGLE) != 0) {
                if ((parentRef != 0) && (parentNode.pos.distanceSquared(bestNode.pos) < queryData.raycastLimitSqr)) {
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
                MeshTile neighbourTile = tileAndPolyUns.first;
                Poly neighbourPoly = tileAndPolyUns.second;

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
                if (neighbourNode.flags == 0) {
                    Result<Vector3f> midpod = getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile);
                    if (!midpod.failed()) {
                        neighbourNode.pos = midpod.result;
                    }
                }

                // Calculate cost and heuristic.
                float cost = 0;
                float heuristic;

                // raycast parent
                boolean foundShortCut = false;
                if (tryLOS) {
                    Result<RaycastHit> rayHit = raycast(parentRef, parentNode.pos, neighbourNode.pos, queryData.filter,
                            DT_RAYCAST_USE_COSTS, grandpaRef);
                    if (rayHit.succeeded()) {
                        foundShortCut = rayHit.result.t >= 1f;
                        if (foundShortCut) {
                            // shortcut found using raycast. Using shorter cost
                            // instead
                            cost = parentNode.cost + rayHit.result.pathCost;
                        }
                    }
                }

                // update move cost
                if (!foundShortCut) {
                    // No shortcut found.
                    float curCost = queryData.filter.getCost(bestNode.pos, neighbourNode.pos, parentRef, parentTile,
                            parentPoly, bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly);
                    cost = bestNode.cost + curCost;
                }

                // Special case for last node.
                if (neighbourRef == queryData.endRef) {
                    float endCost = queryData.filter.getCost(neighbourNode.pos, queryData.endPos, bestRef, bestTile,
                            bestPoly, neighbourRef, neighbourTile, neighbourPoly, 0, null, null);

                    cost = cost + endCost;
                    heuristic = 0;
                } else {
                    heuristic = neighbourNode.pos.distance(queryData.endPos) * H_SCALE;
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
                neighbourNode.flags = (neighbourNode.flags & ~(Node.CLOSED | Node.PARENT_DETACHED));
                neighbourNode.cost = cost;
                neighbourNode.totalCost = total;
                if (foundShortCut) {
                    neighbourNode.flags = (neighbourNode.flags | Node.PARENT_DETACHED);
                }

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

    /**
     * Finalizes and returns the results of a sliced path query.
     * @returns An ordered list of polygon references representing the path. (Start to end.)
     * */
    @Override
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

            Node prev = null;
            Node node = queryData.lastBestNode;
            int prevRay = 0;
            do {
                Node next = nodePool.getNodeAtIdx(node.parentIndex);
                node.parentIndex = nodePool.getNodeIdx(prev);
                prev = node;
                int nextRay = node.flags & Node.PARENT_DETACHED; // keep track of whether parent is not adjacent
                // (i.e. due to raycast shortcut)
                node.flags = (node.flags & ~Node.PARENT_DETACHED) | prevRay; // and store it in the reversed
                // path's node
                prevRay = nextRay;
                node = next;
            } while (node != null);

            // Store path
            node = prev;
            do {
                Node next = nodePool.getNodeAtIdx(node.parentIndex);
                if ((node.flags & Node.PARENT_DETACHED) != 0) {
                    Result<RaycastHit> iresult = raycast(node.polygonRef, node.pos, next.pos, queryData.filter, 0, 0);
                    if (iresult.succeeded()) {
                        path.addAll(iresult.result.path);
                    }
                    // raycast ends on poly boundary and the path might include the next poly boundary.
                    if (path.get(path.getSize() - 1) == next.polygonRef) {
                        path.remove(path.getSize() - 1); // remove to avoid duplicates
                    }
                } else {
                    path.add(node.polygonRef);
                }

                node = next;
            } while (node != null);
        }

        Status status = queryData.status;
        // Reset query.
        queryData = new QueryData();

        return Result.of(status, path);
    }

    /**
     * Finalizes and returns the results of an incomplete sliced path query, returning the path
     * to the furthest polygon on the existing path that was visited during the search.
     * @param existing An array of polygon references for the existing path.
     * @returns An ordered list of polygon references representing the path. (Start to end.)
     * */
    @Override
    public Result<LongArrayList> finalizeSlicedFindPathPartial(LongArrayList existing) {

        LongArrayList path = new LongArrayList(64);
        if (Objects.isNull(existing) || existing.getSize() <= 0) {
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
            // Find furthest existing node that was visited.
            Node prev = null;
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

            // Reverse the path.
            int prevRay = 0;
            do {
                Node next = nodePool.getNodeAtIdx(node.parentIndex);
                node.parentIndex = nodePool.getNodeIdx(prev);
                prev = node;
                int nextRay = node.flags & Node.PARENT_DETACHED; // keep track of whether parent is not adjacent
                // (i.e. due to raycast shortcut)
                node.flags = (node.flags & ~Node.PARENT_DETACHED) | prevRay; // and store it in the reversed
                // path's node
                prevRay = nextRay;
                node = next;
            } while (node != null);

            // Store path
            node = prev;
            do {
                Node next = nodePool.getNodeAtIdx(node.parentIndex);
                if ((node.flags & Node.PARENT_DETACHED) != 0) {
                    Result<RaycastHit> iresult = raycast(node.polygonRef, node.pos, next.pos, queryData.filter, 0, 0);
                    if (iresult.succeeded()) {
                        path.addAll(iresult.result.path);
                    }
                    // raycast ends on poly boundary, and the path might include the next poly boundary.
                    if (path.get(path.getSize() - 1) == next.polygonRef) {
                        path.remove(path.getSize() - 1); // remove to avoid duplicates
                    }
                } else {
                    path.add(node.polygonRef);
                }

                node = next;
            } while (node != null);
        }
        Status status = queryData.status;
        // Reset query.
        queryData = new QueryData();

        return Result.of(status, path);
    }

    @Override
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
            // The API input has been cheked already, skip checking internal data.
            long bestRef = bestNode.polygonRef;
            Pair<MeshTile, Poly> tileAndPoly = nav.getTileAndPolyByRefUnsafe(bestRef);
            MeshTile bestTile = tileAndPoly.first;
            Poly bestPoly = tileAndPoly.second;

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
                                MeshTile neiTile = linkTileAndPoly.first;
                                Poly neiPoly = linkTileAndPoly.second;
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
                float distSqr = distseg.first;
                float tseg = distseg.second;

                // Edge is too far, skip.
                if (distSqr > radiusSqr) {
                    continue;
                }

                // Hit wall, update radius.
                radiusSqr = distSqr;
                // Calculate hit pos.
                float[] vs = bestTile.data.vertices;
                hitPos.x = vs[vj] + (vs[vi] - vs[vj]) * tseg;
                hitPos.y = vs[vj + 1] + (vs[vi + 1] - vs[vj + 1]) * tseg;
                hitPos.z = vs[vj + 2] + (vs[vi + 2] - vs[vj + 2]) * tseg;
                bestvj = new VectorPtr(vs, vj);
                bestvi = new VectorPtr(vs, vi);
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
                MeshTile neighbourTile = neighbourTileAndPoly.first;
                Poly neighbourPoly = neighbourTileAndPoly.second;

                // Skip off-mesh connections.
                if (neighbourPoly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
                    continue;
                }

                // Calc distance to the edge.
                int va = bestPoly.vertices[link.indexOfPolyEdge] * 3;
                int vb = bestPoly.vertices[(link.indexOfPolyEdge + 1) % bestPoly.vertCount] * 3;
                Pair<Float, Float> distseg = distancePtSegSqr2D(centerPos, bestTile.data.vertices, va, vb);
                float distSqr = distseg.first;
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
                    Result<Vector3f> midPoint = getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly,
                            neighbourTile);
                    if (midPoint.succeeded()) {
                        neighbourNode.pos = midPoint.result;
                    }
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
            hitNormal.z = -tangent.x;
            hitNormal.normalize();
        }
        return Result.success(new FindDistanceToWallResult((float) Math.sqrt(radiusSqr), hitPos, hitNormal));
    }
}
