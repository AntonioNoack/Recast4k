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
package org.recast4j.detour

import org.joml.Vector3f
import org.recast4j.LongArrayList
import org.recast4j.Vectors
import java.util.*
import kotlin.math.sqrt

class LegacyNavMeshQuery(val nav: NavMesh) : NavMeshQuery(nav) {

    override fun findPath(
        startRef: Long, endRef: Long, startPos: Vector3f, endPos: Vector3f, filter: QueryFilter,
        options: Int, raycastLimit: Float
    ) = findPath(startRef, endRef, startPos, endPos, filter)

    override fun findPath(
        startRef: Long, endRef: Long, startPos: Vector3f, endPos: Vector3f,
        filter: QueryFilter
    ): Result<LongArrayList?> {
        // Validate input
        if (!nav.isValidPolyRef(startRef) || !nav.isValidPolyRef(endRef) || !Vectors.isFinite(startPos) || !Vectors.isFinite(
                endPos
            )
        ) return Result.invalidParam()

        if (startRef == endRef) {
            val path = LongArrayList(1)
            path.add(startRef)
            return Result.success(path)
        }
        nodePool.clear()
        openList.clear()
        val startNode = nodePool.getNode(startRef)
        Vectors.copy(startNode.pos, startPos)
        startNode.parentIndex = 0
        startNode.cost = 0f
        startNode.totalCost = startPos.distance(endPos) * H_SCALE
        startNode.polygonRef = startRef
        startNode.flags = Node.OPEN
        openList.offer(startNode)
        var lastBestNode = startNode
        var lastBestNodeCost = startNode.totalCost
        var status = Status.SUCCESS
        while (!openList.isEmpty()) {
            // Remove node from open list and put it in closed list.
            val bestNode = openList.poll()
            bestNode.flags = bestNode.flags and Node.OPEN.inv()
            bestNode.flags = bestNode.flags or Node.CLOSED

            // Reached the goal, stop searching.
            if (bestNode.polygonRef == endRef) {
                lastBestNode = bestNode
                break
            }

            // Get current poly and tile.
            // The API input has been checked already, skip checking internal data.
            val bestRef = bestNode.polygonRef
            val bestTile = nav.getTileByRefUnsafe(bestRef)
            val bestPoly = nav.getPolyByRefUnsafe(bestRef, bestTile)

            // Get parent poly and tile.
            var parentRef = 0L
            var parentTile: MeshTile? = null
            var parentPoly: Poly? = null
            if (bestNode.parentIndex != 0) {
                parentRef = nodePool.getNodeAtIdx(bestNode.parentIndex)!!.polygonRef
            }
            if (parentRef != 0L) {
                parentTile = nav.getTileByRefUnsafe(parentRef)
                parentPoly = nav.getPolyByRefUnsafe(parentRef, parentTile)
            }
            var i = bestTile.polyLinks[bestPoly.index]
            while (i != NavMesh.DT_NULL_LINK) {
                val neighbourRef = bestTile.links[i].neighborRef

                // Skip invalid ids and do not expand back to where we came from.
                if (neighbourRef == 0L || neighbourRef == parentRef) {
                    i = bestTile.links[i].indexOfNextLink
                    continue
                }

                // Get neighbour poly and tile.
                // The API input has been checked already, skip checking internal data.
                val neighbourTile = nav.getTileByRefUnsafe(neighbourRef)
                val neighbourPoly = nav.getPolyByRefUnsafe(neighbourRef, neighbourTile)
                if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    i = bestTile.links[i].indexOfNextLink
                    continue
                }

                // deal explicitly with crossing tile boundaries
                var crossSide = 0
                if (bestTile.links[i].side != 0xff) {
                    crossSide = bestTile.links[i].side shr 1
                }

                // get the node
                val neighbourNode = nodePool.getNode(neighbourRef, crossSide)

                // If the node is visited the first time, calculate node position.
                if (neighbourNode.flags == 0) {
                    val midpod =
                        getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile)
                    if (midpod != null) {
                        neighbourNode.pos.set(midpod)
                    }
                }

                // Calculate cost and heuristic.
                var cost: Float
                var heuristic: Float

                // Special case for last node.
                val curCost = filter.getCost(
                    bestNode.pos, neighbourNode.pos, parentRef, parentTile, parentPoly,
                    bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly
                )
                if (neighbourRef == endRef) {
                    // Cost
                    val endCost = filter.getCost(
                        neighbourNode.pos, endPos, bestRef, bestTile, bestPoly, neighbourRef,
                        neighbourTile, neighbourPoly, 0L, null, null
                    )
                    cost = bestNode.cost + curCost + endCost
                    heuristic = 0f
                } else {
                    // Cost
                    cost = bestNode.cost + curCost
                    heuristic = neighbourNode.pos.distance(endPos) * H_SCALE
                }
                val total = cost + heuristic

                // The node is already in open list and the new result is worse, skip.
                if (neighbourNode.flags and Node.OPEN != 0 && total >= neighbourNode.totalCost) {
                    i = bestTile.links[i].indexOfNextLink
                    continue
                }
                // The node is already visited and process, and the new result is worse, skip.
                if (neighbourNode.flags and Node.CLOSED != 0 && total >= neighbourNode.totalCost) {
                    i = bestTile.links[i].indexOfNextLink
                    continue
                }

                // Add or update the node.
                neighbourNode.parentIndex = nodePool.getNodeIdx(bestNode)
                neighbourNode.polygonRef = neighbourRef
                neighbourNode.flags = neighbourNode.flags and Node.CLOSED.inv()
                neighbourNode.cost = cost
                neighbourNode.totalCost = total
                if (neighbourNode.flags and Node.OPEN != 0) {
                    // Already in open, update node location.
                    openList.remove(neighbourNode)
                    openList.offer(neighbourNode)
                } else {
                    // Put the node in open list.
                    neighbourNode.flags = neighbourNode.flags or Node.OPEN
                    openList.offer(neighbourNode)
                }

                // Update nearest node to target so far.
                if (heuristic < lastBestNodeCost) {
                    lastBestNodeCost = heuristic
                    lastBestNode = neighbourNode
                }
                i = bestTile.links[i].indexOfNextLink
            }
        }
        val path = getPathToNode(lastBestNode)
        if (lastBestNode.polygonRef != endRef) {
            status = Status.PARTIAL_RESULT
        }
        return Result.of(status, path)
    }

    /**
     * Updates an in-progress sliced path query.
     *
     * @param maxIter The maximum number of iterations to perform.
     * @return The status flags for the query.
     */
    override fun updateSlicedFindPath(maxIter: Int): Result<Int> {
        if (!queryData.status.isInProgress) {
            return Result.of(queryData.status, 0)
        }

        // Make sure the request is still valid.
        if (!nav.isValidPolyRef(queryData.startRef) || !nav.isValidPolyRef(queryData.endRef)) {
            queryData.status = Status.FAILURE
            return Result.of(queryData.status, 0)
        }
        var iter = 0
        while (iter < maxIter && !openList.isEmpty()) {
            iter++

            // Remove node from open list and put it in closed list.
            val bestNode = openList.poll()
            bestNode.flags = bestNode.flags and Node.OPEN.inv()
            bestNode.flags = bestNode.flags or Node.CLOSED

            // Reached the goal, stop searching.
            if (bestNode.polygonRef == queryData.endRef) {
                queryData.lastBestNode = bestNode
                queryData.status = Status.SUCCESS
                return Result.of(queryData.status, iter)
            }

            // Get current poly and tile.
            // The API input has been checked already, skip checking internal
            // data.
            val bestRef = bestNode.polygonRef
            val bestTile = nav.getTileByRef(bestRef)
            val bestPoly = nav.getPolyByRef(bestRef, bestTile)
            if (bestTile == null || bestPoly == null) {
                queryData.status = Status.FAILURE
                // The polygon has disappeared during the sliced query, fail.
                return Result.of(queryData.status, iter)
            }
            // Get parent and grand parent poly and tile.
            var parentRef = 0L
            var grandpaRef = 0L
            var parentTile: MeshTile? = null
            var parentPoly: Poly? = null
            var parentNode: Node? = null
            if (bestNode.parentIndex != 0) {
                parentNode = nodePool.getNodeAtIdx(bestNode.parentIndex)!!
                parentRef = parentNode.polygonRef
                if (parentNode.parentIndex != 0) {
                    grandpaRef = nodePool.getNodeAtIdx(parentNode.parentIndex)!!.polygonRef
                }
            }
            if (parentRef != 0L) {
                parentTile = nav.getTileByRef(parentRef)
                parentPoly = nav.getPolyByRef(parentRef, parentTile)
                if (parentPoly == null || grandpaRef != 0L && !nav.isValidPolyRef(grandpaRef)) {
                    // The polygon has disappeared during the sliced query,
                    // fail.
                    queryData.status = Status.FAILURE
                    return Result.of(queryData.status, iter)
                }
            }

            // decide whether to test raycast to previous nodes
            var tryLOS = false
            if (queryData.options and DT_FINDPATH_ANY_ANGLE != 0) {
                if (parentRef != 0L && parentNode!!.pos.distanceSquared(bestNode.pos) < queryData.raycastLimitSqr) {
                    tryLOS = true
                }
            }
            var i = bestTile.polyLinks[bestPoly.index]
            while (i != NavMesh.DT_NULL_LINK) {
                val neighbourRef = bestTile.links[i].neighborRef

                // Skip invalid ids and do not expand back to where we came
                // from.
                if (neighbourRef == 0L || neighbourRef == parentRef) {
                    i = bestTile.links[i].indexOfNextLink
                    continue
                }

                // Get neighbour poly and tile.
                // The API input has been checked already, skip checking internal
                // data.
                val neighbourTile = nav.getTileByRefUnsafe(neighbourRef)
                val neighbourPoly = nav.getPolyByRefUnsafe(neighbourRef, neighbourTile)
                if (!queryData.filter!!.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    i = bestTile.links[i].indexOfNextLink
                    continue
                }

                // get the neighbor node
                val neighbourNode = nodePool.getNode(neighbourRef, 0)

                // do not expand to nodes that were already visited from the
                // same parent
                if (neighbourNode.parentIndex != 0 && neighbourNode.parentIndex == bestNode.parentIndex) {
                    i = bestTile.links[i].indexOfNextLink
                    continue
                }

                // If the node is visited the first time, calculate node
                // position.
                if (neighbourNode.flags == 0) {
                    val midpod =
                        getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile)
                    if (midpod != null) {
                        neighbourNode.pos.set(midpod)
                    }
                }

                // Calculate cost and heuristic.
                var cost = 0f
                var heuristic: Float

                // raycast parent
                var foundShortCut = false
                if (tryLOS) {
                    val rayHit = raycast(
                        parentRef, parentNode!!.pos, neighbourNode.pos, queryData.filter!!,
                        DT_RAYCAST_USE_COSTS, grandpaRef
                    )
                    if (rayHit.succeeded()) {
                        foundShortCut = rayHit.result!!.t >= 1f
                        if (foundShortCut) {
                            // shortcut found using raycast. Using shorter cost
                            // instead
                            cost = parentNode.cost + rayHit.result.pathCost
                        }
                    }
                }

                // update move cost
                if (!foundShortCut) {
                    // No shortcut found.
                    val curCost = queryData.filter!!.getCost(
                        bestNode.pos, neighbourNode.pos, parentRef, parentTile,
                        parentPoly, bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly
                    )
                    cost = bestNode.cost + curCost
                }

                // Special case for last node.
                if (neighbourRef == queryData.endRef) {
                    val endCost = queryData.filter!!.getCost(
                        neighbourNode.pos, queryData.endPos, bestRef, bestTile,
                        bestPoly, neighbourRef, neighbourTile, neighbourPoly, 0, null, null
                    )
                    cost = cost + endCost
                    heuristic = 0f
                } else {
                    heuristic = neighbourNode.pos.distance(queryData.endPos) * H_SCALE
                }
                val total = cost + heuristic

                // The node is already in open list and the new result is worse,
                // skip.
                if (neighbourNode.flags and Node.OPEN != 0 && total >= neighbourNode.totalCost) {
                    i = bestTile.links[i].indexOfNextLink
                    continue
                }
                // The node is already visited and process, and the new result
                // is worse, skip.
                if (neighbourNode.flags and Node.CLOSED != 0 && total >= neighbourNode.totalCost) {
                    i = bestTile.links[i].indexOfNextLink
                    continue
                }

                // Add or update the node.
                neighbourNode.parentIndex = if (foundShortCut) bestNode.parentIndex else nodePool.getNodeIdx(bestNode)
                neighbourNode.polygonRef = neighbourRef
                neighbourNode.flags = neighbourNode.flags and (Node.CLOSED or Node.PARENT_DETACHED).inv()
                neighbourNode.cost = cost
                neighbourNode.totalCost = total
                if (foundShortCut) {
                    neighbourNode.flags = neighbourNode.flags or Node.PARENT_DETACHED
                }
                if (neighbourNode.flags and Node.OPEN != 0) {
                    // Already in open, update node location.
                    openList.remove(neighbourNode)
                    openList.offer(neighbourNode)
                } else {
                    // Put the node in open list.
                    neighbourNode.flags = neighbourNode.flags or Node.OPEN
                    openList.offer(neighbourNode)
                }

                // Update nearest node to target so far.
                if (heuristic < queryData.lastBestNodeCost) {
                    queryData.lastBestNodeCost = heuristic
                    queryData.lastBestNode = neighbourNode
                }
                i = bestTile.links[i].indexOfNextLink
            }
        }

        // Exhausted all nodes, but could not find path.
        if (openList.isEmpty()) {
            queryData.status = Status.PARTIAL_RESULT
        }
        return Result.of(queryData.status, iter)
    }

    /**
     * Finalizes and returns the results of a sliced path query.
     * @returns An ordered list of polygon references representing the path. (Start to end.)
     */
    override fun finalizeSlicedFindPath(): Result<LongArrayList> {
        val path = LongArrayList(64)
        if (queryData.status.isFailed) {
            // Reset query.
            queryData = QueryData()
            return Result.failure(path)
        }
        if (queryData.startRef == queryData.endRef) {
            // Special case: the search starts and ends at same poly.
            path.add(queryData.startRef)
        } else {
            // Reverse the path.
            if (queryData.lastBestNode!!.polygonRef != queryData.endRef) {
                queryData.status = Status.PARTIAL_RESULT
            }
            var prev: Node? = null
            var node = queryData.lastBestNode
            var prevRay = 0
            do {
                val next = nodePool.getNodeAtIdx(node!!.parentIndex)
                node.parentIndex = nodePool.getNodeIdx(prev)
                prev = node
                val nextRay = node.flags and Node.PARENT_DETACHED // keep track of whether parent is not adjacent
                // (i.e. due to raycast shortcut)
                node.flags = node.flags and Node.PARENT_DETACHED.inv() or prevRay // and store it in the reversed
                // path's node
                prevRay = nextRay
                node = next
            } while (node != null)

            // Store path
            node = prev
            do {
                val next = nodePool.getNodeAtIdx(node!!.parentIndex)
                if (node.flags and Node.PARENT_DETACHED != 0) {
                    val iresult = raycast(node.polygonRef, node.pos, next!!.pos, queryData.filter!!, 0, 0)
                    if (iresult.succeeded()) {
                        path.addAll(iresult.result!!.path)
                    }
                    // raycast ends on poly boundary and the path might include the next poly boundary.
                    if (path[path.size - 1] == next.polygonRef) {
                        path.remove(path.size - 1) // remove to avoid duplicates
                    }
                } else {
                    path.add(node.polygonRef)
                }
                node = next
            } while (node != null)
        }
        val status = queryData.status
        // Reset query.
        queryData = QueryData()
        return Result.of(status, path)
    }

    /**
     * Finalizes and returns the results of an incomplete sliced path query, returning the path
     * to the furthest polygon on the existing path that was visited during the search.
     * @param existing An array of polygon references for the existing path.
     * @returns An ordered list of polygon references representing the path. (Start to end.)
     */
    override fun finalizeSlicedFindPathPartial(existing: LongArrayList): Result<LongArrayList> {
        val path = LongArrayList(64)
        if (Objects.isNull(existing) || existing.size <= 0) {
            return Result.failure(path)
        }
        if (queryData.status.isFailed) {
            // Reset query.
            queryData = QueryData()
            return Result.failure(path)
        }
        if (queryData.startRef == queryData.endRef) {
            // Special case: the search starts and ends at same poly.
            path.add(queryData.startRef)
        } else {
            // Find furthest existing node that was visited.
            var prev: Node? = null
            var node: Node? = null
            for (i in existing.size - 1 downTo 0) {
                node = nodePool.findNode(existing[i])
                if (node != null) {
                    break
                }
            }
            if (node == null) {
                queryData.status = Status.PARTIAL_RESULT
                node = queryData.lastBestNode
            }

            // Reverse the path.
            var prevRay = 0
            do {
                val next = nodePool.getNodeAtIdx(node!!.parentIndex)
                node.parentIndex = nodePool.getNodeIdx(prev)
                prev = node
                val nextRay = node.flags and Node.PARENT_DETACHED // keep track of whether parent is not adjacent
                // (i.e. due to raycast shortcut)
                node.flags = node.flags and Node.PARENT_DETACHED.inv() or prevRay // and store it in the reversed
                // path's node
                prevRay = nextRay
                node = next
            } while (node != null)

            // Store path
            node = prev
            do {
                val next = nodePool.getNodeAtIdx(node!!.parentIndex)
                if (node.flags and Node.PARENT_DETACHED != 0) {
                    val iresult = raycast(node.polygonRef, node.pos, next!!.pos, queryData.filter!!, 0, 0)
                    if (iresult.succeeded()) {
                        path.addAll(iresult.result!!.path)
                    }
                    // raycast ends on poly boundary, and the path might include the next poly boundary.
                    if (path[path.size - 1] == next.polygonRef) {
                        path.remove(path.size - 1) // remove to avoid duplicates
                    }
                } else {
                    path.add(node.polygonRef)
                }
                node = next
            } while (node != null)
        }
        val status = queryData.status
        // Reset query.
        queryData = QueryData()
        return Result.of(status, path)
    }

    override fun findDistanceToWall(
        startRef: Long,
        centerPos: Vector3f,
        maxRadius: Float,
        filter: QueryFilter
    ): Result<FindDistanceToWallResult> {

        // Validate input
        if (!nav.isValidPolyRef(startRef) || !centerPos.isFinite || maxRadius < 0 || !maxRadius.isFinite())
            return Result.invalidParam()

        nodePool.clear()
        openList.clear()
        val startNode = nodePool.getNode(startRef)
        Vectors.copy(startNode.pos, centerPos)
        startNode.parentIndex = 0
        startNode.cost = 0f
        startNode.totalCost = 0f
        startNode.polygonRef = startRef
        startNode.flags = Node.OPEN
        openList.offer(startNode)
        var radiusSqr = Vectors.sqr(maxRadius)
        val hitPos = Vector3f()
        var bestvj: VectorPtr? = null
        var bestvi: VectorPtr? = null
        while (!openList.isEmpty()) {
            val bestNode = openList.poll()
            bestNode.flags = bestNode.flags and Node.OPEN.inv()
            bestNode.flags = bestNode.flags or Node.CLOSED

            // Get poly and tile.
            // The API input has been checked already, skip checking internal data.
            val bestRef = bestNode.polygonRef
            val bestTile = nav.getTileByRefUnsafe(bestRef)
            val bestPoly = nav.getPolyByRefUnsafe(bestRef,bestTile)

            // Get parent poly and tile.
            var parentRef = 0L
            if (bestNode.parentIndex != 0) {
                parentRef = nodePool.getNodeAtIdx(bestNode.parentIndex)!!.polygonRef
            }

            // Hit test walls.
            run {
                var i = 0
                var j = bestPoly.vertCount - 1
                while (i < bestPoly.vertCount) {

                    // Skip non-solid edges.
                    if (bestPoly.neighborData[j] and NavMesh.DT_EXT_LINK != 0) {
                        // Tile border.
                        var solid = true
                        var k = bestTile.polyLinks[bestPoly.index]
                        while (k != NavMesh.DT_NULL_LINK) {
                            val link = bestTile.links[k]
                            if (link.indexOfPolyEdge == j) {
                                if (link.neighborRef != 0L) {
                                    val neighbourRef = link.neighborRef
                                    val neighbourTile = nav.getTileByRefUnsafe(neighbourRef)
                                    val neighbourPoly = nav.getPolyByRefUnsafe(neighbourRef, neighbourTile)
                                    if (filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                                        solid = false
                                    }
                                }
                                break
                            }
                            k = bestTile.links[k].indexOfNextLink
                        }
                        if (!solid) {
                            j = i++
                            continue
                        }
                    } else if (bestPoly.neighborData[j] != 0) {
                        // Internal edge
                        val idx = bestPoly.neighborData[j] - 1
                        val ref = nav.getPolyRefBase(bestTile) or idx.toLong()
                        if (filter.passFilter(ref, bestTile, bestTile.data!!.polygons[idx])) {
                            j = i++
                            continue
                        }
                    }

                    // Calc distance to the edge.
                    val vj = bestPoly.vertices[j] * 3
                    val vi = bestPoly.vertices[i] * 3
                    val (distSqr, tseg) = Vectors.distancePtSegSqr2D(centerPos, bestTile.data!!.vertices, vj, vi)

                    // Edge is too far, skip.
                    if (distSqr > radiusSqr) {
                        j = i++
                        continue
                    }

                    // Hit wall, update radius.
                    radiusSqr = distSqr
                    // Calculate hit pos.
                    val vs = bestTile.data!!.vertices
                    hitPos.x = vs[vj] + (vs[vi] - vs[vj]) * tseg
                    hitPos.y = vs[vj + 1] + (vs[vi + 1] - vs[vj + 1]) * tseg
                    hitPos.z = vs[vj + 2] + (vs[vi + 2] - vs[vj + 2]) * tseg
                    bestvj = VectorPtr(vs, vj)
                    bestvi = VectorPtr(vs, vi)
                    j = i++
                }
            }
            var i = bestTile.polyLinks[bestPoly.index]
            while (i != NavMesh.DT_NULL_LINK) {
                val link = bestTile.links[i]
                val neighbourRef = link.neighborRef
                // Skip invalid neighbours and do not follow back to parent.
                if (neighbourRef == 0L || neighbourRef == parentRef) {
                    i = bestTile.links[i].indexOfNextLink
                    continue
                }

                // Expand to neighbour.
                val neighbourTile = nav.getTileByRefUnsafe(neighbourRef)
                val neighbourPoly = nav.getPolyByRefUnsafe(neighbourRef, neighbourTile)

                // Skip off-mesh connections.
                if (neighbourPoly.type == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
                    i = bestTile.links[i].indexOfNextLink
                    continue
                }

                // Calc distance to the edge.
                val va = bestPoly.vertices[link.indexOfPolyEdge] * 3
                val vb = bestPoly.vertices[(link.indexOfPolyEdge + 1) % bestPoly.vertCount] * 3
                val (distSqr) = Vectors.distancePtSegSqr2D(centerPos, bestTile.data!!.vertices, va, vb)
                // If the circle is not touching the next polygon, skip it.
                if (distSqr > radiusSqr) {
                    i = bestTile.links[i].indexOfNextLink
                    continue
                }
                if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    i = bestTile.links[i].indexOfNextLink
                    continue
                }
                val neighbourNode = nodePool.getNode(neighbourRef)
                if (neighbourNode.flags and Node.CLOSED != 0) {
                    i = bestTile.links[i].indexOfNextLink
                    continue
                }

                // Cost
                if (neighbourNode.flags == 0) {
                    val midPoint = getEdgeMidPoint(
                        bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly,
                        neighbourTile
                    )
                    if (midPoint != null) {
                        neighbourNode.pos.set(midPoint)
                    }
                }
                val total = bestNode.totalCost + bestNode.pos.distance(neighbourNode.pos)

                // The node is already in open list, and the new result is worse, skip.
                if (neighbourNode.flags and Node.OPEN != 0 && total >= neighbourNode.totalCost) {
                    i = bestTile.links[i].indexOfNextLink
                    continue
                }
                neighbourNode.polygonRef = neighbourRef
                neighbourNode.flags = neighbourNode.flags and Node.CLOSED.inv()
                neighbourNode.parentIndex = nodePool.getNodeIdx(bestNode)
                neighbourNode.totalCost = total
                if (neighbourNode.flags and Node.OPEN != 0) {
                    openList.remove(neighbourNode)
                    openList.offer(neighbourNode)
                } else {
                    neighbourNode.flags = neighbourNode.flags or Node.OPEN
                    openList.offer(neighbourNode)
                }
                i = bestTile.links[i].indexOfNextLink
            }
        }

        // Calc hit normal.
        val hitNormal = Vector3f()
        if (bestvi != null) {
            val tangent = Vectors.sub(bestvi!!, bestvj!!)
            hitNormal.x = tangent.z
            hitNormal.z = -tangent.x
            hitNormal.normalize()
        }
        return Result.success(FindDistanceToWallResult(sqrt(radiusSqr), hitPos, hitNormal))
    }

    companion object {
        private const val H_SCALE = 0.999f // Search heuristic scale.
    }
}