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
package org.recast4j.detour.crowd;

import org.joml.Vector3f;
import org.recast4j.LongArrayList;
import org.recast4j.Pair;
import org.recast4j.detour.*;

import java.util.ArrayList;
import java.util.List;

import static org.recast4j.Vectors.*;

/**
 * Represents a dynamic polygon corridor used to plan agent movement.
 * <p>
 * The corridor is loaded with a path, usually obtained from a #NavMeshQuery::findPath() query. The corridor is then
 * used to plan local movement, with the corridor automatically updating as needed to deal with inaccurate agent
 * locomotion.
 * <p>
 * Example of a common use case:
 * <p>
 * -# Construct the corridor object and call -# Obtain a path from a #dtNavMeshQuery object. -# Use #reset() to set the
 * agent's current position. (At the beginning of the path.) -# Use #setCorridor() to load the path and target. -# Use
 * #findCorners() to plan movement. (This handles dynamic path straightening.) -# Use #movePosition() to feed agent
 * movement back into the corridor. (The corridor will automatically adjust as needed.) -# If the target is moving, use
 * #moveTargetPosition() to update the end of the corridor. (The corridor will automatically adjust as needed.) -#
 * Repeat the previous 3 steps to continue to move the agent.
 * <p>
 * The corridor position and target are always constrained to the navigation mesh.
 * <p>
 * One of the difficulties in maintaining a path is that floating point errors, locomotion inaccuracies, and/or local
 * steering can result in the agent crossing the boundary of the path corridor, temporarily invalidating the path. This
 * class uses local mesh queries to detect and update the corridor as needed to handle these types of issues.
 * <p>
 * The fact that local mesh queries are used to move the position and target locations results in two beahviors that
 * need to be considered:
 * <p>
 * Every time a move function is used there is a chance that the path will become non-optimial. Basically, the further
 * the target is moved from its original location, and the further the position is moved outside the original corridor,
 * the more likely the path will become non-optimal. This issue can be addressed by periodically running the
 * #optimizePathTopology() and #optimizePathVisibility() methods.
 * <p>
 * All local mesh queries have distance limitations. (Review the #dtNavMeshQuery methods for details.) So the most
 * accurate use case is to move the position and target in small increments. If a large increment is used, then the
 * corridor may not be able to accurately find the new location. Because of this limiation, if a position is moved in a
 * large increment, then compare the desired and resulting polygon references. If the two do not match, then path
 * replanning may be needed. E.g. If you move the target, check #getLastPoly() to see if it is the expected polygon.
 */
public class PathCorridor {

    /**
     * Gets the current position within the corridor. (In the first polygon.)
     */
    public final Vector3f pos = new Vector3f();

    /**
     * Gets the current target within the corridor. (In the last polygon.)
     */
    public final Vector3f target = new Vector3f();

    /**
     * The corridor's path.
     */
    public LongArrayList path;

    protected LongArrayList mergeCorridorStartMoved(LongArrayList path, LongArrayList visited) {
        int furthestPath = -1;
        int furthestVisited = -1;

        // Find furthest common polygon.
        for (int i = path.getSize() - 1; i >= 0; --i) {
            boolean found = false;
            for (int j = visited.getSize() - 1; j >= 0; --j) {
                if (path.get(i) == visited.get(j)) {
                    furthestPath = i;
                    furthestVisited = j;
                    found = true;
                }
            }
            if (found) {
                break;
            }
        }

        // If no intersection found just return current path.
        if (furthestPath == -1) {
            return path;
        }

        // Concatenate paths.

        // Adjust beginning of the buffer to include the visited.
        LongArrayList result = new LongArrayList();
        // Store visited
        for (int i = visited.getSize() - 1; i > furthestVisited; --i) {
            result.add(visited.get(i));
        }
        result.addAll(path, furthestPath, path.getSize());
        return result;
    }

    protected LongArrayList mergeCorridorEndMoved(LongArrayList path, LongArrayList visited) {
        int furthestPath = -1;
        int furthestVisited = -1;

        // Find the furthest common polygon.
        for (int i = 0; i < path.getSize(); ++i) {
            boolean found = false;
            for (int j = visited.getSize() - 1; j >= 0; --j) {
                if (path.get(i) == visited.get(j)) {
                    furthestPath = i;
                    furthestVisited = j;
                    found = true;
                }
            }
            if (found) {
                break;
            }
        }

        // If no intersection found just return current path.
        if (furthestPath == -1) {
            return path;
        }

        // Concatenate paths.
        path.shrink(furthestPath);
        path.addAll(visited, furthestVisited, visited.getSize());
        return path;
    }

    protected LongArrayList mergeCorridorStartShortcut(LongArrayList path, LongArrayList visited) {

        int furthestPath = -1;
        int furthestVisited = -1;

        // Find the furthest common polygon.
        for (int i = path.getSize() - 1; i >= 0; --i) {
            boolean found = false;
            for (int j = visited.getSize() - 1; j >= 0; --j) {
                if (path.get(i) == visited.get(j)) {
                    furthestPath = i;
                    furthestVisited = j;
                    found = true;
                }
            }
            if (found) {
                break;
            }
        }

        // If no intersection found just return current path.
        if (furthestPath == -1 || furthestVisited <= 0) {
            return path;
        }

        // Concatenate paths.

        // Adjust beginning of the buffer to include the visited.
        visited.shrink(furthestVisited);
        visited.addAll(path, furthestPath, path.getSize());
        return visited;
    }

    /**
     * Allocates the corridor's path buffer.
     */
    public PathCorridor() {
        path = new LongArrayList();
    }

    /**
     * Resets the path corridor to the specified position.
     *
     * @param ref The polygon reference containing the position.
     * @param pos The new position in the corridor. [(x, y, z)]
     */
    public void reset(long ref, Vector3f pos) {
        path.clear();
        path.add(ref);
        this.pos.set(pos);
        target.set(pos);
    }

    private static final float MIN_TARGET_DIST = sqr(0.01f);

    /**
     * Finds the corners in the corridor from the position toward the target. (The straightened path.)
     * <p>
     * This is the function used to plan local movement within the corridor. One or more corners can be detected in
     * order to plan movement. It performs essentially the same function as #dtNavMeshQuery::findStraightPath.
     * <p>
     * Due to internal optimizations, the maximum number of corners returned will be (@p maxCorners - 1) For example: If
     * the buffers are sized to hold 10 corners, the function will never return more than 9 corners. So if 10 corners
     * are needed, the buffers should be sized for 11 corners.
     * <p>
     * If the target is within range, it will be the last corner and have a polygon reference id of zero.
     *
     * @return Corners
     * @param[in] navquery The query object used to build the corridor.
     */
    public List<StraightPathItem> findCorners(int maxCorners, NavMeshQuery navquery) {
        List<StraightPathItem> path = new ArrayList<>();
        Result<List<StraightPathItem>> result = navquery.findStraightPath(pos, target, this.path, maxCorners, 0);
        if (result.succeeded()) {
            path = result.result;
            // Prune points in the beginning of the path which are too close.
            int start = 0;
            for (StraightPathItem spi : path) {
                if ((spi.getFlags() & NavMeshQuery.DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0
                        || dist2DSqr(spi.getPos(), pos) > MIN_TARGET_DIST) {
                    break;
                }
                start++;
            }
            int end = path.size();
            // Prune points after an off-mesh connection.
            for (int i = start; i < path.size(); i++) {
                StraightPathItem spi = path.get(i);
                if ((spi.getFlags() & NavMeshQuery.DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0) {
                    end = i + 1;
                    break;
                }
            }
            path = path.subList(start, end);
        }
        return path;
    }

    /**
     * Attempts to optimize the path if the specified point is visible from the current position.
     * <p>
     * Inaccurate locomotion or dynamic obstacle avoidance can force the agent position significantly outside the
     * original corridor. Over time this can result in the formation of a non-optimal corridor. Non-optimal paths can
     * also form near the corners of tiles.
     * <p>
     * This function uses an efficient local visibility search to try to optimize the corridor between the current
     * position and @p next.
     * <p>
     * The corridor will change only if @p next is visible from the current position and moving directly toward the
     * point is better than following the existing path.
     * <p>
     * The more inaccurate the agent movement, the more beneficial this function becomes. Simply adjust the frequency of
     * the call to match the needs to the agent.
     * <p>
     * This function is not suitable for long distance searches.
     *
     * @param next                  The point to search toward. [(x, y, z])
     * @param pathOptimizationRange The maximum range to search. [Limit: > 0]
     * @param navquery              The query object used to build the corridor.
     * @param filter                The filter to apply to the operation.
     */

    public void optimizePathVisibility(Vector3f next, float pathOptimizationRange, NavMeshQuery navquery,
                                       QueryFilter filter) {
        // Clamp the ray to max distance.
        float dist = dist2D(pos, next);

        // If too close to the goal, do not try to optimize.
        if (dist < 0.01f) {
            return;
        }

        // Overshoot a little. This helps to optimize open fields in tiled
        // meshes.
        dist = Math.min(dist + 0.01f, pathOptimizationRange);

        // Adjust ray length.
        Vector3f delta = sub(next, pos);
        Vector3f goal = mad(pos, delta, pathOptimizationRange / dist);

        Result<RaycastHit> rc = navquery.raycast(path.get(0), pos, goal, filter, 0, 0);
        if (rc.succeeded()) {
            if (rc.result.path.getSize() > 1 && rc.result.t > 0.99f) {
                path = mergeCorridorStartShortcut(path, rc.result.path);
            }
        }
    }

    /**
     * Attempts to optimize the path using a local area search. (Partial replanning.)
     * <p>
     * Inaccurate locomotion or dynamic obstacle avoidance can force the agent position significantly outside the
     * original corridor. Over time this can result in the formation of a non-optimal corridor. This function will use a
     * local area path search to try to re-optimize the corridor.
     * <p>
     * The more inaccurate the agent movement, the more beneficial this function becomes. Simply adjust the frequency of
     * the call to match the needs to the agent.
     *
     * @param query The query object used to build the corridor.
     * @param filter   The filter to apply to the operation.
     */
    void optimizePathTopology(NavMeshQuery query, QueryFilter filter, int maxIterations) {
        if (path.getSize() < 3) {
            return;
        }
        query.initSlicedFindPath(path.get(0), path.get(path.getSize() - 1), pos, target, filter, 0);
        query.updateSlicedFindPath(maxIterations);
        Result<LongArrayList> fpr = query.finalizeSlicedFindPathPartial(path);
        if (fpr.succeeded() && fpr.result.getSize() > 0) {
            path = mergeCorridorStartShortcut(path, fpr.result);
        }
    }

    public boolean moveOverOffmeshConnection(long offMeshConRef, long[] refs, Vector3f start, Vector3f end,
                                             NavMeshQuery navquery) {
        // Advance the path up to and over the off-mesh connection.
        long prevRef = 0, polyRef = path.get(0);
        int npos = 0;
        while (npos < path.getSize() && polyRef != offMeshConRef) {
            prevRef = polyRef;
            polyRef = path.get(npos);
            npos++;
        }
        if (npos == path.getSize()) {
            // Could not find offMeshConRef
            return false;
        }

        // Prune path
        path = path.subList(npos, path.getSize());
        refs[0] = prevRef;
        refs[1] = polyRef;

        NavMesh nav = navquery.getAttachedNavMesh();
        Result<Pair<Vector3f, Vector3f>> startEnd = nav.getOffMeshConnectionPolyEndPoints(refs[0], refs[1]);
        if (startEnd.succeeded()) {
            copy(pos, startEnd.result.second);
            copy(start, startEnd.result.first);
            copy(end, startEnd.result.second);
            return true;
        }
        return false;
    }

    /**
     * Moves the position from the current location to the desired location, adjusting the corridor as needed to reflect
     * the change.
     * <p>
     * Behavior:
     * <p>
     * - The movement is constrained to the surface of the navigation mesh. - The corridor is automatically adjusted
     * (shorted or lengthened) in order to remain valid. - The new position will be located in the adjusted corridor's
     * first polygon.
     * <p>
     * The expected use case is that the desired position will be 'near' the current corridor. What is considered 'near'
     * depends on local polygon density, query search extents, etc.
     * <p>
     * The resulting position will differ from the desired position if the desired position is not on the navigation
     * mesh, or it can't be reached using a local search.
     *
     * @param npos     The desired new position. [(x, y, z)]
     * @param navquery The query object used to build the corridor.
     * @param filter   The filter to apply to the operation.
     */
    @SuppressWarnings({"unused", "UnusedReturnValue"})
    public boolean movePosition(Vector3f npos, NavMeshQuery navquery, QueryFilter filter) {
        // Move along navmesh and update new position.
        Result<MoveAlongSurfaceResult> masResult = navquery.moveAlongSurface(path.get(0), pos, npos, filter);
        if (masResult.succeeded()) {
            path = mergeCorridorStartMoved(path, masResult.result.visited);
            // Adjust the position to stay on top of the navmesh.
            copy(pos, masResult.result.resultPos);
            Result<Float> hr = navquery.getPolyHeight(path.get(0), masResult.result.resultPos);
            if (hr.succeeded()) pos.y = hr.result;
            return true;
        }
        return false;
    }

    /**
     * Moves the target from the curent location to the desired location, adjusting the corridor as needed to reflect
     * the change. Behavior: - The movement is constrained to the surface of the navigation mesh. - The corridor is
     * automatically adjusted (shorted or lengthened) in order to remain valid. - The new target will be located in the
     * adjusted corridor's last polygon.
     * <p>
     * The expected use case is that the desired target will be 'near' the current corridor. What is considered 'near'
     * depends on local polygon density, query search extents, etc. The resulting target will differ from the desired
     * target if the desired target is not on the navigation mesh, or it can't be reached using a local search.
     *
     * @param npos     The desired new target position. [(x, y, z)]
     * @param navquery The query object used to build the corridor.
     * @param filter   The filter to apply to the operation.
     */
    @SuppressWarnings("unused")
    public boolean moveTargetPosition(Vector3f npos, NavMeshQuery navquery, QueryFilter filter, boolean adjustPositionToTopOfNavMesh) {
        // Move along navmesh and update new position.
        Result<MoveAlongSurfaceResult> masResult = navquery.moveAlongSurface(path.get(path.getSize() - 1), target, npos, filter);
        if (masResult.succeeded()) {
            path = mergeCorridorEndMoved(path, masResult.result.visited);
            Vector3f resultPos = masResult.result.resultPos;
            if (adjustPositionToTopOfNavMesh) {
                float h = target.y;
                navquery.getPolyHeight(path.get(path.getSize() - 1), npos);
                resultPos.y = h;
            }
            target.set(resultPos);
            return true;
        }
        return false;
    }

    /**
     * Loads a new path and target into the corridor. The current corridor position is expected to be within the first
     * polygon in the path. The target is expected to be in the last polygon.
     *
     * @param target The target location within the last polygon of the path. [(x, y, z)]
     * @param path   The path corridor.
     * @warning The size of the path must not exceed the size of corridor's path buffer set during #init().
     */

    public void setCorridor(Vector3f target, LongArrayList path) {
        this.target.set(target);
        this.path = new LongArrayList(path);
    }

    public void fixPathStart(long safeRef, Vector3f safePos) {
        pos.set(safePos);
        if (path.getSize() < 3 && path.getSize() > 0) {
            long p = path.get(path.getSize() - 1);
            path.clear();
            path.add(safeRef);
            path.add(0L);
            path.add(p);
        } else {
            path.clear();
            path.add(safeRef);
            path.add(0L);
        }

    }

    @SuppressWarnings("unused")
    public void trimInvalidPath(long safeRef, Vector3f safePos, NavMeshQuery navquery, QueryFilter filter) {
        // Keep valid path as far as possible.
        int n = 0;
        while (n < path.getSize() && navquery.isValidPolyRef(path.get(n), filter)) {
            n++;
        }

        if (n == 0) {
            // The first polyref is bad, use current safe values.
            copy(pos, safePos);
            path.clear();
            path.add(safeRef);
        } else if (n < path.getSize()) {
            path.shrink(n);
            // The path is partially usable.
        }
        // Clamp target pos to last poly
        Result<Vector3f> result = navquery.closestPointOnPolyBoundary(path.get(path.getSize() - 1), target);
        if (result.succeeded()) {
            target.set(result.result);
        }
    }

    /**
     * Checks the current corridor path to see if its polygon references remain valid. The path can be invalidated if
     * there are structural changes to the underlying navigation mesh, or the state of a polygon within the path changes
     * resulting in it being filtered out. (E.g. An exclusion or inclusion flag changes.)
     *
     * @param maxLookAhead The number of polygons from the beginning of the corridor to search.
     * @param navquery     The query object used to build the corridor.
     * @param filter       The filter to apply to the operation.
     */
    boolean isValid(int maxLookAhead, NavMeshQuery navquery, QueryFilter filter) {
        // Check that all polygons still pass query filter.
        int n = Math.min(path.getSize(), maxLookAhead);
        for (int i = 0; i < n; ++i) {
            if (!navquery.isValidPolyRef(path.get(i), filter)) {
                return false;
            }
        }

        return true;
    }

    /**
     * The polygon reference id of the first polygon in the corridor, the polygon containing the position.
     *
     * @return The polygon reference id of the first polygon in the corridor. (Or zero if there is no path.)
     */
    public long getFirstPoly() {
        return path.isEmpty() ? 0 : path.get(0);
    }

    /**
     * The polygon reference id of the last polygon in the corridor, the polygon containing the target.
     *
     * @return The polygon reference id of the last polygon in the corridor. (Or zero if there is no path.)
     */
    public long getLastPoly() {
        return path.isEmpty() ? 0 : path.get(path.getSize() - 1);
    }

}
