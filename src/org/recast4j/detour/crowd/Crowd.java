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
import org.recast4j.Vectors;
import org.recast4j.detour.*;
import org.recast4j.detour.crowd.CrowdAgent.CrowdAgentState;
import org.recast4j.detour.crowd.CrowdAgent.MoveRequestState;
import org.recast4j.detour.crowd.ObstacleAvoidanceQuery.ObstacleAvoidanceParams;
import org.recast4j.detour.crowd.debug.CrowdAgentDebugInfo;
import org.recast4j.detour.crowd.debug.ObstacleAvoidanceDebugData;

import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.IntFunction;

import static org.joml.Math.clamp;
import static org.recast4j.Vectors.*;

/**
 * Members in this module implement local steering and dynamic avoidance features.
 * <p>
 * The crowd is the big beast of the navigation features. It not only handles a lot of the path management for you, but
 * also local steering and dynamic avoidance between members of the crowd. I.e. It can keep your agents from running
 * into each other.
 * <p>
 * Main class: Crowd
 * <p>
 * The #dtNavMeshQuery and #dtPathCorridor classes provide perfectly good, easy to use path planning features. But in
 * the end they only give you points that your navigation client should be moving toward. When it comes to deciding
 * things like agent velocity and steering to avoid other agents, that is up to you to implement. Unless, of course, you
 * decide to use Crowd.
 * <p>
 * Basically, you add an agent to the crowd, providing various configuration settings such as maximum speed and
 * acceleration. You also provide a local target to move toward. The crowd manager then provides, with every update, the
 * new agent position and velocity for the frame. The movement will be constrained to the navigation mesh, and steering
 * will be applied to ensure agents managed by the crowd do not collide with each other.
 * <p>
 * This is very powerful feature set. But it comes with limitations.
 * <p>
 * The biggest limitation is that you must give control of the agent's position completely over to the crowd manager.
 * You can update things like maximum speed and acceleration. But in order for the crowd manager to do its thing, it
 * can't allow you to constantly be giving it overrides to position and velocity. So you give up direct control of the
 * agent's movement. It belongs to the crowd.
 * <p>
 * The second-biggest limitation revolves around the fact that the crowd manager deals with local planning. So the
 * agent's target should never be more than 256 polygons away from its current position. If it is, you risk your agent
 * failing to reach its target. So you may still need to do long distance planning and provide the crowd manager with
 * intermediate targets.
 * <p>
 * Other significant limitations:
 * <p>
 * - All agents using the crowd manager will use the same #dtQueryFilter. - Crowd management is relatively expensive.
 * The maximum agents under crowd management at any one time is between 20 and 30. A good place to start is a maximum of
 * 25 agents for 0.5ms per frame.
 *
 * @note This is a summary list of members. Use the index or search feature to find minor members.
 * struct dtCrowdAgentParams
 * var dtCrowdAgentParams::obstacleAvoidanceType
 * par #dtCrowd permits agents to use different avoidance configurations. This value is the index of the
 * #dtObstacleAvoidanceParams within the crowd.
 * var dtCrowdAgentParams::collisionQueryRange
 * par Collision elements include other agents and navigation mesh boundaries.
 * <p>
 * This value is often based on the agent radius and/or maximum speed. E.g. radius * 8
 * var dtCrowdAgentParams::pathOptimizationRange
 * par Only applicable if #updateFlags includes the #CROWD_OPTIMIZE_VIS flag.
 * <p>
 * This value is often based on the agent radius. E.g., radius * 30
 * var dtCrowdAgentParams::separationWeight
 * par A higher value will result in agents trying to stay farther away from each other at the cost of more difficult
 * steering in tight spaces.
 * see CrowdAgent, Crowd::addAgent(), Crowd::updateAgentParameters()
 * see dtObstacleAvoidanceParams, dtCrowd::setObstacleAvoidanceParams(), dtCrowd::getObstacleAvoidanceParams()
 * see dtPathCorridor::optimizePathVisibility()
 * <p>
 * This is the core class of the @ref crowd module. See the @ref crowd documentation for a summary of the crowd
 * features. A common method for setting up the crowd is as follows: -# Allocate the crowd -# Set the avoidance
 * configurations using #setObstacleAvoidanceParams(). -# Add agents using #addAgent() and make an initial movement
 * request using #requestMoveTarget(). A common process for managing the crowd is as follows: -# Call #update() to allow
 * the crowd to manage its agents. -# Retrieve agent information using #getActiveAgents(). -# Make movement requests
 * using #requestMoveTarget() when movement goal changes. -# Repeat every frame. Some agent configuration settings can
 * be updated using #updateAgentParameters(). But the crowd owns the agent position. So it is not possible to update an
 * active agent's position. If agent position must be fed back into the crowd, the agent must be removed and re-added.
 * Notes: - Path related information is available for newly added agents only after an #update() has been performed. -
 * Agent objects are kept in a pool and re-used. So it is important when using agent objects to check the value of
 * #dtCrowdAgent::active to determine if the agent is actually in use or not. - This class is meant to provide 'local'
 * movement. There is a limit of 256 polygons in the path corridor. So it is not meant to provide automatic pathfinding
 * services over long distances.
 * <p>
 * see dtAllocCrowd(), dtFreeCrowd(), init(), dtCrowdAgent
 */
public class Crowd {

    /// The maximum number of corners a crowd agent will look ahead in the path.
    /// This value is used for sizing the crowd agent corner buffers.
    /// Due to the behavior of the crowd manager, the actual number of useful
    /// corners will be one less than this number.
    static final int CROWDAGENT_MAX_CORNERS = 4;

    /// The maximum number of crowd avoidance configurations supported by the
    /// crowd manager.
    /// @see dtObstacleAvoidanceParams, dtCrowd::setObstacleAvoidanceParams(), dtCrowd::getObstacleAvoidanceParams(),
    /// dtCrowdAgentParams::obstacleAvoidanceType
    static final int CROWD_MAX_OBSTAVOIDANCE_PARAMS = 8;

    /// The maximum number of query filter types supported by the crowd manager.
    /// @see dtQueryFilter, dtCrowd::getFilter() dtCrowd::getEditableFilter(),
    /// dtCrowdAgentParams::queryFilterType
    static final int CROWD_MAX_QUERY_FILTER_TYPE = 16;

    private final AtomicInteger agentId = new AtomicInteger();
    private final Set<CrowdAgent> activeAgents;
    public final PathQueue pathQueue;
    private final ObstacleAvoidanceParams[] m_obstacleQueryParams = new ObstacleAvoidanceParams[CROWD_MAX_OBSTAVOIDANCE_PARAMS];
    private final ObstacleAvoidanceQuery obstacleAvoidanceQuery;
    public ProximityGrid grid;
    public final Vector3f queryExtends = new Vector3f();
    private final QueryFilter[] m_filters = new QueryFilter[CROWD_MAX_QUERY_FILTER_TYPE];
    private NavMeshQuery navQuery;
    private NavMesh navMesh;
    public final CrowdConfig config;
    public final CrowdTelemetry telemetry = new CrowdTelemetry();
    int m_velocitySampleCount;

    @SuppressWarnings("unused")
    public Crowd(CrowdConfig config, NavMesh nav) {
        this(config, nav, i -> new DefaultQueryFilter());
    }

    public Crowd(CrowdConfig config, NavMesh nav, IntFunction<QueryFilter> queryFilterFactory) {

        this.config = config;
        set(queryExtends, config.maxAgentRadius * 2f, config.maxAgentRadius * 1.5f, config.maxAgentRadius * 2f);

        obstacleAvoidanceQuery = new ObstacleAvoidanceQuery(config.maxObstacleAvoidanceCircles, config.maxObstacleAvoidanceSegments);

        for (int i = 0; i < CROWD_MAX_QUERY_FILTER_TYPE; i++) {
            m_filters[i] = queryFilterFactory.apply(i);
        }
        // Init obstacle query params.
        for (int i = 0; i < CROWD_MAX_OBSTAVOIDANCE_PARAMS; ++i) {
            m_obstacleQueryParams[i] = new ObstacleAvoidanceParams();
        }

        // Allocate temp buffer for merging paths.
        pathQueue = new PathQueue(config);
        activeAgents = new HashSet<>();

        // The navQuery is mostly used for local searches, no need for large node pool.
        navMesh = nav;
        navQuery = new NavMeshQuery(nav);
    }

    @SuppressWarnings("unused")
    public void setNavMesh(NavMesh nav) {
        navMesh = nav;
        navQuery = new NavMeshQuery(nav);
    }

    /**
     * Sets the shared avoidance configuration for the specified index.
     */
    @SuppressWarnings("unused")
    public void setObstacleAvoidanceParams(int idx, ObstacleAvoidanceParams params) {
        if (idx >= 0 && idx < CROWD_MAX_OBSTAVOIDANCE_PARAMS) {
            m_obstacleQueryParams[idx] = new ObstacleAvoidanceParams(params);
        }
    }

    /**
     * Gets the shared avoidance configuration for the specified index.
     */
    @SuppressWarnings("unused")
    public ObstacleAvoidanceParams getObstacleAvoidanceParams(int idx) {
        if (idx >= 0 && idx < CROWD_MAX_OBSTAVOIDANCE_PARAMS) {
            return m_obstacleQueryParams[idx];
        }
        return null;
    }

    /**
     * Updates the specified agent's configuration.
     */
    public void updateAgentParameters(CrowdAgent agent, CrowdAgentParams params) {
        agent.params = params;
    }

    /**
     * Adds a new agent to the crowd.
     *
     * @param pos    The requested position of the agent. [(x, y, z)]
     * @param params The configutation of the agent.
     * @return The newly created agent object
     */
    @SuppressWarnings("unused")
    public CrowdAgent addAgent(Vector3f pos, CrowdAgentParams params) {
        CrowdAgent ag = new CrowdAgent(agentId.getAndIncrement());
        activeAgents.add(ag);
        updateAgentParameters(ag, params);

        // Find nearest position on navmesh and place the agent there.
        Result<FindNearestPolyResult> nearestPoly = navQuery.findNearestPoly(pos, queryExtends, m_filters[ag.params.queryFilterType]);

        Vector3f nearest = nearestPoly.succeeded() ? nearestPoly.result.nearestPos : pos;
        long ref = nearestPoly.succeeded() ? nearestPoly.result.nearestRef : 0L;
        ag.corridor.reset(ref, nearest);
        ag.boundary.reset();
        ag.partial = false;

        ag.topologyOptTime = 0;
        ag.targetReplanTime = 0;

        set(ag.desiredVelocity, 0, 0, 0);
        set(ag.desiredVelAdjusted, 0, 0, 0);
        set(ag.actualVelocity, 0, 0, 0);
        copy(ag.currentPosition, nearest);

        ag.desiredSpeed = 0;

        if (ref != 0) {
            ag.state = CrowdAgentState.WALKING;
        } else {
            ag.state = CrowdAgentState.INVALID;
        }

        ag.targetState = MoveRequestState.NONE;

        return ag;
    }

    /**
     * Removes the agent from the crowd.
     *
     * @param agent Agent to be removed
     */
    @SuppressWarnings("unused")
    public void removeAgent(CrowdAgent agent) {
        activeAgents.remove(agent);
    }

    private void requestMoveTargetReplan(CrowdAgent ag, long ref, Vector3f pos) {
        ag.setTarget(ref, pos);
        ag.targetReplan = true;
    }

    /// Submits a new move request for the specified agent.
    /// @param[in] idx The agent index. [Limits: 0 <= value < #getAgentCount()]
    /// @param[in] ref The position's polygon reference.
    /// @param[in] pos The position within the polygon. [(x, y, z)]
    /// @return True if the request was successfully submitted.
    ///
    /// This method is used when a new target is set.
    ///
    /// The position will be constrained to the surface of the navigation mesh.
    ///
    /// The request will be processed during the next #update().
    @SuppressWarnings("unused")
    public boolean requestMoveTarget(CrowdAgent agent, long ref, Vector3f pos) {
        if (ref == 0) return false;
        // Initialize request.
        agent.setTarget(ref, pos);
        agent.targetReplan = false;
        return true;
    }

    /// Submits a new move request for the specified agent.
    /// @param[in] idx The agent index. [Limits: 0 <= value < #getAgentCount()]
    /// @param[in] vel The movement velocity. [(x, y, z)]
    /// @return True if the request was successfully submitted.
    @SuppressWarnings("unused")
    public boolean requestMoveVelocity(CrowdAgent agent, Vector3f vel) {
        // Initialize request.
        agent.targetRef = 0;
        agent.targetPos.set(vel); // anno: what??
        agent.targetPathQueryResult = null;
        agent.targetReplan = false;
        agent.targetState = MoveRequestState.VELOCITY;
        return true;
    }

    /**
     * Resets any request for the specified agent.
     * @return True if the request was successfully reset.
     * */
    @SuppressWarnings("unused")
    public boolean resetMoveTarget(CrowdAgent agent) {
        // Initialize request.
        agent.targetRef = 0;
        set(agent.targetPos, 0, 0, 0);
        set(agent.desiredVelocity, 0, 0, 0);
        agent.targetPathQueryResult = null;
        agent.targetReplan = false;
        agent.targetState = MoveRequestState.NONE;
        return true;
    }

    /**
     * Gets the active agents int the agent pool.
     *
     * @return List of active agents
     */
    public List<CrowdAgent> getActiveAgents() {
        return new ArrayList<>(activeAgents);
    }

    @SuppressWarnings("unused")
    public QueryFilter getFilter(int i) {
        return i >= 0 && i < CROWD_MAX_QUERY_FILTER_TYPE ? m_filters[i] : null;
    }

    public CrowdTelemetry update(float dt, CrowdAgentDebugInfo debug) {
        m_velocitySampleCount = 0;

        telemetry.start();

        Collection<CrowdAgent> agents = getActiveAgents();

        // Check that all agents still have valid paths.
        checkPathValidity(agents, dt);

        // Update async move request and path finder.
        updateMoveRequest(agents, dt);

        // Optimize path topology.
        updateTopologyOptimization(agents, dt);

        // Register agents to proximity grid.
        buildProximityGrid(agents);

        // Get nearby navmesh segments and agents to collide with.
        buildNeighbours(agents);

        // Find next corner to steer to.
        findCorners(agents, debug);

        // Trigger off-mesh connections (depends on corners).
        triggerOffMeshConnections(agents);

        // Calculate steering.
        calculateSteering(agents);

        // Velocity planning.
        planVelocity(debug, agents);

        // Integrate.
        integrate(dt, agents);

        // Handle collisions.
        handleCollisions(agents);

        moveAgents(agents);

        // Update agents using off-mesh connection.
        updateOffMeshConnections(agents, dt);
        return telemetry;
    }

    private void checkPathValidity(Collection<CrowdAgent> agents, float dt) {
        telemetry.start("checkPathValidity");

        for (CrowdAgent ag : agents) {

            if (ag.state != CrowdAgentState.WALKING) {
                continue;
            }

            ag.targetReplanTime += dt;

            boolean replan = false;

            // First check, that the current location is valid.
            Vector3f agentPos = new Vector3f();
            long agentRef = ag.corridor.getFirstPoly();
            copy(agentPos, ag.currentPosition);
            if (!navQuery.isValidPolyRef(agentRef, m_filters[ag.params.queryFilterType])) {
                // Current location is not valid, try to reposition.
                // TODO: this can snap agents, how to handle that?
                Result<FindNearestPolyResult> nearestPoly = navQuery.findNearestPoly(ag.currentPosition, queryExtends,
                        m_filters[ag.params.queryFilterType]);
                agentRef = nearestPoly.succeeded() ? nearestPoly.result.nearestRef : 0L;
                if (nearestPoly.succeeded()) {
                    agentPos.set(nearestPoly.result.nearestPos);
                }

                if (agentRef == 0) {
                    // Could not find location in navmesh, set state to invalid.
                    ag.corridor.reset(0, agentPos);
                    ag.partial = false;
                    ag.boundary.reset();
                    ag.state = CrowdAgentState.INVALID;
                    continue;
                }

                // Make sure the first polygon is valid, but leave other valid
                // polygons in the path so that replanner can adjust the path
                // better.
                ag.corridor.fixPathStart(agentRef, agentPos);
                // ag.corridor.trimInvalidPath(agentRef, agentPos, m_navquery,
                // &m_filter);
                ag.boundary.reset();
                copy(ag.currentPosition, agentPos);

                replan = true;
            }

            // If the agent does not have move target or is controlled by
            // velocity, no need to recover the target nor replan.
            if (ag.targetState == MoveRequestState.NONE
                    || ag.targetState == MoveRequestState.VELOCITY) {
                continue;
            }

            // Try to recover move request position.
            if (ag.targetState != MoveRequestState.FAILED) {
                if (!navQuery.isValidPolyRef(ag.targetRef, m_filters[ag.params.queryFilterType])) {
                    // Current target is not valid, try to reposition.
                    Result<FindNearestPolyResult> fnp = navQuery.findNearestPoly(ag.targetPos, queryExtends,
                            m_filters[ag.params.queryFilterType]);
                    ag.targetRef = fnp.succeeded() ? fnp.result.nearestRef : 0L;
                    if (fnp.succeeded()) {
                        copy(ag.targetPos, fnp.result.nearestPos);
                    }
                    replan = true;
                }
                if (ag.targetRef == 0) {
                    // Failed to reposition target, fail moverequest.
                    ag.corridor.reset(agentRef, agentPos);
                    ag.partial = false;
                    ag.targetState = MoveRequestState.NONE;
                }
            }

            // If nearby corridor is not valid, replan.
            if (!ag.corridor.isValid(config.checkLookAhead, navQuery, m_filters[ag.params.queryFilterType])) {
                // Fix current path.
                // ag.corridor.trimInvalidPath(agentRef, agentPos, m_navquery,
                // &m_filter);
                // ag.boundary.reset();
                replan = true;
            }

            // If the end of the path is near and it is not the requested
            // location, replan.
            if (ag.targetState == MoveRequestState.VALID) {
                if (ag.targetReplanTime > config.targetReplanDelay && ag.corridor.path.getSize() < config.checkLookAhead
                        && ag.corridor.getLastPoly() != ag.targetRef) {
                    replan = true;
                }
            }

            // Try to replan path to goal.
            if (replan) {
                if (ag.targetState != MoveRequestState.NONE) {
                    requestMoveTargetReplan(ag, ag.targetRef, ag.targetPos);
                }
            }
        }
        telemetry.stop("checkPathValidity");
    }

    private void updateMoveRequest(Collection<CrowdAgent> agents, float dt) {
        telemetry.start("updateMoveRequest");

        PriorityQueue<CrowdAgent> queue = new PriorityQueue<>(
                (a1, a2) -> Float.compare(a2.targetReplanTime, a1.targetReplanTime));

        // Fire off new requests.
        for (CrowdAgent ag : agents) {
            if (ag.state == CrowdAgentState.INVALID) {
                continue;
            }
            if (ag.targetState == MoveRequestState.NONE
                    || ag.targetState == MoveRequestState.VELOCITY) {
                continue;
            }

            if (ag.targetState == MoveRequestState.REQUESTING) {
                LongArrayList path = ag.corridor.path;
                if (path.isEmpty()) {
                    throw new IllegalArgumentException("Empty path");
                }
                // Quick search towards the goal.
                navQuery.initSlicedFindPath(path.get(0), ag.targetRef, ag.currentPosition, ag.targetPos,
                        m_filters[ag.params.queryFilterType], 0);
                navQuery.updateSlicedFindPath(config.maxTargetFindPathIterations);
                Result<LongArrayList> pathFound;
                if (ag.targetReplan) // && npath > 10)
                {
                    // Try to use existing steady path during replan if
                    // possible.
                    pathFound = navQuery.finalizeSlicedFindPathPartial(path);
                } else {
                    // Try to move towards target when goal changes.
                    pathFound = navQuery.finalizeSlicedFindPath();
                }
                LongArrayList reqPath = pathFound.result;
                Vector3f reqPos = new Vector3f();
                if (pathFound.succeeded() && reqPath.getSize() > 0) {
                    // In progress or succeed.
                    if (reqPath.get(reqPath.getSize() - 1) != ag.targetRef) {
                        // Partial path, constrain target position inside the
                        // last polygon.
                        Result<ClosestPointOnPolyResult> cr = navQuery.closestPointOnPoly(reqPath.get(reqPath.getSize() - 1),
                                ag.targetPos);
                        if (cr.succeeded()) {
                            reqPos = cr.result.pos;
                        } else {
                            reqPath = new LongArrayList();
                        }
                    } else {
                        copy(reqPos, ag.targetPos);
                    }
                } else {
                    // Could not find path, start the request from current
                    // location.
                    copy(reqPos, ag.currentPosition);
                    reqPath = new LongArrayList();
                    reqPath.add(path.get(0));
                }

                ag.corridor.setCorridor(reqPos, reqPath);
                ag.boundary.reset();
                ag.partial = false;

                if (reqPath.get(reqPath.getSize() - 1) == ag.targetRef) {
                    ag.targetState = MoveRequestState.VALID;
                    ag.targetReplanTime = 0;
                } else {
                    // The path is longer or potentially unreachable, full plan.
                    ag.targetState = MoveRequestState.WAITING_FOR_QUEUE;
                }
                ag.targetReplanWaitTime = 0;
            }

            if (ag.targetState == MoveRequestState.WAITING_FOR_QUEUE) {
                queue.add(ag);
            }
        }

        while (!queue.isEmpty()) {
            CrowdAgent ag = queue.poll();
            ag.targetPathQueryResult = pathQueue.request(ag.corridor.getLastPoly(), ag.targetRef, ag.corridor.target,
                    ag.targetPos, m_filters[ag.params.queryFilterType]);
            if (ag.targetPathQueryResult != null) {
                ag.targetState = MoveRequestState.WAITING_FOR_PATH;
            } else {
                telemetry.recordMaxTimeToEnqueueRequest(ag.targetReplanWaitTime);
                ag.targetReplanWaitTime += dt;
            }
        }

        // Update requests.
        telemetry.start("pathQueueUpdate");
        pathQueue.update(navMesh);
        telemetry.stop("pathQueueUpdate");

        // Process path results.
        for (CrowdAgent ag : agents) {
            if (ag.targetState == MoveRequestState.NONE
                    || ag.targetState == MoveRequestState.VELOCITY) {
                continue;
            }

            if (ag.targetState == MoveRequestState.WAITING_FOR_PATH) {
                // telemetry.recordPathWaitTime(ag.targetReplanTime);
                // Poll path queue.
                Status status = ag.targetPathQueryResult.status;
                if (status != null && status.isFailed()) {
                    // Path find failed, retry if the target location is still
                    // valid.
                    ag.targetPathQueryResult = null;
                    if (ag.targetRef != 0) {
                        ag.targetState = MoveRequestState.REQUESTING;
                    } else {
                        ag.targetState = MoveRequestState.FAILED;
                    }
                    ag.targetReplanTime = 0;
                } else if (status != null && status.isSuccess()) {
                    LongArrayList path = ag.corridor.path;
                    if (path.isEmpty()) {
                        throw new IllegalArgumentException("Empty path");
                    }

                    // Apply results.
                    Vector3f targetPos = ag.targetPos;

                    boolean valid = true;
                    LongArrayList res = ag.targetPathQueryResult.path;
                    if (res.isEmpty()) {
                        valid = false;
                    }

                    ag.partial = status.isPartial();

                    // Merge result and existing path.
                    // The agent might have moved whilst the request is
                    // being processed, so the path may have changed.
                    // We assume that the end of the path is at the same
                    // location
                    // where the request was issued.

                    // The last ref in the old path should be the same as
                    // the location where the request was issued..
                    if (valid && path.get(path.getSize() - 1) != res.get(0)) {
                        valid = false;
                    }

                    if (valid) {
                        // Put the old path infront of the old path.
                        if (path.getSize() > 1) {
                            path.remove(path.getSize() - 1);
                            path.addAll(res);
                            res = path;
                            // Remove trackbacks
                            for (int j = 1; j < res.getSize() - 1; ++j) {
                                if (j - 1 >= 0 && j + 1 < res.getSize()) {
                                    if (res.get(j - 1) == res.get(j + 1)) {
                                        res.remove(j + 1);
                                        res.remove(j);
                                        j -= 2;
                                    }
                                }
                            }
                        }

                        // Check for partial path.
                        if (res.get(res.getSize() - 1) != ag.targetRef) {
                            // Partial path, constrain target position inside
                            // the last polygon.
                            Result<ClosestPointOnPolyResult> cr = navQuery.closestPointOnPoly(res.get(res.getSize() - 1), targetPos);
                            if (cr.succeeded()) {
                                targetPos = cr.result.pos;
                            } else {
                                valid = false;
                            }
                        }
                    }

                    if (valid) {
                        // Set current corridor.
                        ag.corridor.setCorridor(targetPos, res);
                        // Force to update boundary.
                        ag.boundary.reset();
                        ag.targetState = MoveRequestState.VALID;
                    } else {
                        // Something went wrong.
                        ag.targetState = MoveRequestState.FAILED;
                    }

                    ag.targetReplanTime = 0;
                }
                telemetry.recordMaxTimeToFindPath(ag.targetReplanWaitTime);
                ag.targetReplanWaitTime += dt;
            }
        }
        telemetry.stop("updateMoveRequest");
    }

    private void updateTopologyOptimization(Collection<CrowdAgent> agents, float dt) {
        telemetry.start("updateTopologyOptimization");

        PriorityQueue<CrowdAgent> queue = new PriorityQueue<>((a1, a2) -> Float.compare(a2.topologyOptTime, a1.topologyOptTime));

        for (CrowdAgent ag : agents) {
            if (ag.state != CrowdAgentState.WALKING) {
                continue;
            }
            if (ag.targetState == MoveRequestState.NONE
                    || ag.targetState == MoveRequestState.VELOCITY) {
                continue;
            }
            if ((ag.params.updateFlags & CrowdAgentParams.CROWD_OPTIMIZE_TOPO) == 0) {
                continue;
            }
            ag.topologyOptTime += dt;
            if (ag.topologyOptTime >= config.topologyOptimizationTimeThreshold) {
                queue.add(ag);
            }
        }

        while (!queue.isEmpty()) {
            CrowdAgent ag = queue.poll();
            ag.corridor.optimizePathTopology(navQuery, m_filters[ag.params.queryFilterType], config.maxTopologyOptimizationIterations);
            ag.topologyOptTime = 0;
        }
        telemetry.stop("updateTopologyOptimization");

    }

    private void buildProximityGrid(Collection<CrowdAgent> agents) {
        telemetry.start("buildProximityGrid");
        grid = new ProximityGrid(config.maxAgentRadius * 3);
        for (CrowdAgent ag : agents) {
            Vector3f p = ag.currentPosition;
            float r = ag.params.radius;
            grid.addItem(ag, p.x - r, p.z - r, p.x + r, p.z + r);
        }
        telemetry.stop("buildProximityGrid");
    }

    private void buildNeighbours(Collection<CrowdAgent> agents) {
        telemetry.start("buildNeighbours");
        for (CrowdAgent ag : agents) {
            if (ag.state != CrowdAgentState.WALKING) {
                continue;
            }

            // Update the collision boundary after certain distance has been passed or
            // if it has become invalid.
            float updateThr = ag.params.collisionQueryRange * 0.25f;
            if (dist2DSqr(ag.currentPosition, ag.boundary.center) > sqr(updateThr)
                    || !ag.boundary.isValid(navQuery, m_filters[ag.params.queryFilterType])) {
                ag.boundary.update(ag.corridor.getFirstPoly(), ag.currentPosition, ag.params.collisionQueryRange, navQuery,
                        m_filters[ag.params.queryFilterType]);
            }
            // Query neighbour agents
            ag.neis = getNeighbours(ag.currentPosition, ag.params.height, ag.params.collisionQueryRange, ag, grid);
        }
        telemetry.stop("buildNeighbours");
    }

    private List<CrowdNeighbour> getNeighbours(Vector3f pos, float height, float range, CrowdAgent skip, ProximityGrid grid) {

        List<CrowdNeighbour> result = new ArrayList<>();
        Set<CrowdAgent> proxAgents = grid.queryItems(pos.x - range, pos.z - range, pos.x + range, pos.z + range);

        for (CrowdAgent ag : proxAgents) {

            if (ag == skip) {
                continue;
            }

            // Check for overlap.
            Vector3f diff = sub(pos, ag.currentPosition);
            if (Math.abs(diff.y) >= (height + ag.params.height) / 2f) {
                continue;
            }
            diff.y = 0;
            float distSqr = diff.lengthSquared();
            if (distSqr > sqr(range)) continue;

            result.add(new CrowdNeighbour(ag, distSqr));
        }
        result.sort((o1, o2) -> Float.compare(o1.dist, o2.dist));
        return result;

    }

    private void findCorners(Collection<CrowdAgent> agents, CrowdAgentDebugInfo debug) {
        telemetry.start("findCorners");
        CrowdAgent debugAgent = debug != null ? debug.agent : null;
        for (CrowdAgent ag : agents) {

            if (ag.state != CrowdAgentState.WALKING) {
                continue;
            }
            if (ag.targetState == MoveRequestState.NONE
                    || ag.targetState == MoveRequestState.VELOCITY) {
                continue;
            }

            // Find corners for steering
            ag.corners = ag.corridor.findCorners(CROWDAGENT_MAX_CORNERS, navQuery);

            // Check to see if the corner after the next corner is directly visible,
            // and short cut to there.
            if ((ag.params.updateFlags & CrowdAgentParams.CROWD_OPTIMIZE_VIS) != 0 && ag.corners.size() > 0) {
                Vector3f target = ag.corners.get(Math.min(1, ag.corners.size() - 1)).pos;
                ag.corridor.optimizePathVisibility(target, ag.params.pathOptimizationRange, navQuery,
                        m_filters[ag.params.queryFilterType]);

                // Copy data for debug purposes.
                if (debugAgent == ag) {
                    debug.optStart.set(ag.corridor.pos);
                    debug.optEnd.set(target);
                }
            } else {
                // Copy data for debug purposes.
                if (debugAgent == ag) {
                    debug.optStart.set(0f);
                    debug.optEnd.set(0f);
                }
            }
        }
        telemetry.stop("findCorners");
    }

    private void triggerOffMeshConnections(Collection<CrowdAgent> agents) {
        telemetry.start("triggerOffMeshConnections");
        for (CrowdAgent ag : agents) {

            if (ag.state != CrowdAgentState.WALKING) {
                continue;
            }
            if (ag.targetState == MoveRequestState.NONE
                    || ag.targetState == MoveRequestState.VELOCITY) {
                continue;
            }

            // Check
            float triggerRadius = ag.params.radius * 2.25f;
            if (ag.overOffmeshConnection(triggerRadius)) {
                // Prepare to off-mesh connection.
                CrowdAgentAnimation anim = ag.animation;

                // Adjust the path over the off-mesh connection.
                long[] refs = new long[2];
                if (ag.corridor.moveOverOffmeshConnection(ag.corners.get(ag.corners.size() - 1).ref, refs, anim.startPos, anim.endPos, navQuery)) {
                    copy(anim.initPos, ag.currentPosition);
                    anim.polyRef = refs[1];
                    anim.active = true;
                    anim.t = 0f;
                    anim.tMax = (dist2D(anim.startPos, anim.endPos) / ag.params.maxSpeed) * 0.5f;

                    ag.state = CrowdAgentState.OFFMESH;
                    ag.corners.clear();
                    ag.neis.clear();
                }
                // else Path validity check will ensure that bad/blocked connections will be replanned.
            }
        }
        telemetry.stop("triggerOffMeshConnections");
    }

    private void calculateSteering(Collection<CrowdAgent> agents) {
        telemetry.start("calculateSteering");
        for (CrowdAgent ag : agents) {

            if (ag.state != CrowdAgentState.WALKING) {
                continue;
            }
            if (ag.targetState == MoveRequestState.NONE) {
                continue;
            }

            Vector3f dvel = new Vector3f();

            if (ag.targetState == MoveRequestState.VELOCITY) {
                copy(dvel, ag.targetPos);
                ag.desiredSpeed = ag.targetPos.length();
            } else {
                // Calculate steering direction.
                if ((ag.params.updateFlags & CrowdAgentParams.CROWD_ANTICIPATE_TURNS) != 0) {
                    ag.calcSmoothSteerDirection(dvel);
                } else {
                    ag.calcStraightSteerDirection(dvel);
                }
                // Calculate speed scale, which tells the agent to slowdown at the end of the path.
                float slowDownRadius = ag.params.radius * 2; // TODO: make less hacky.
                float speedScale = ag.getDistanceToGoal(slowDownRadius) / slowDownRadius;

                ag.desiredSpeed = ag.params.maxSpeed;
                dvel.mul(ag.desiredSpeed * speedScale);
            }

            // Separation
            if ((ag.params.updateFlags & CrowdAgentParams.CROWD_SEPARATION) != 0) {
                float separationDist = ag.params.collisionQueryRange;
                float invSeparationDist = 1f / separationDist;
                float separationWeight = ag.params.separationWeight;

                float w = 0;
                Vector3f disp = new Vector3f();

                for (int j = 0; j < ag.neis.size(); ++j) {
                    CrowdAgent nei = ag.neis.get(j).agent;

                    Vector3f diff = sub(ag.currentPosition, nei.currentPosition);
                    diff.y = 0;

                    float distSqr = diff.lengthSquared();
                    if (distSqr < 0.00001f || distSqr > sqr(separationDist)) {
                        continue;
                    }
                    float dist = (float) Math.sqrt(distSqr);
                    float weight = separationWeight * (1f - sqr(dist * invSeparationDist));

                    mad2(disp, diff, weight / dist);
                    w += 1f;
                }

                if (w > 0.0001f) {
                    // Adjust desired velocity.
                    mad2(dvel, disp, 1f / w);
                    // Clamp desired velocity to desired speed.
                    float speedSqr = dvel.lengthSquared();
                    float desiredSqr = sqr(ag.desiredSpeed);
                    if (speedSqr > desiredSqr) {
                        dvel.mul(desiredSqr / speedSqr);
                    }
                }
            }

            // Set the desired velocity.
            copy(ag.desiredVelocity, dvel);
        }
        telemetry.stop("calculateSteering");
    }

    private void planVelocity(CrowdAgentDebugInfo debug, Collection<CrowdAgent> agents) {
        telemetry.start("planVelocity");
        CrowdAgent debugAgent = debug != null ? debug.agent : null;
        for (CrowdAgent ag : agents) {

            if (ag.state != CrowdAgentState.WALKING) {
                continue;
            }

            if ((ag.params.updateFlags & CrowdAgentParams.CROWD_OBSTACLE_AVOIDANCE) != 0) {
                obstacleAvoidanceQuery.reset();

                // Add neighbours as obstacles.
                for (int j = 0; j < ag.neis.size(); ++j) {
                    CrowdAgent nei = ag.neis.get(j).agent;
                    obstacleAvoidanceQuery.addCircle(nei.currentPosition, nei.params.radius, nei.actualVelocity, nei.desiredVelocity);
                }

                // Append neighbour segments as obstacles.
                for (int j = 0; j < ag.boundary.segments.size(); ++j) {
                    LocalBoundary.Segment s = ag.boundary.segments.get(j);
                    if (triArea2D(ag.currentPosition, s.start, s.end) < 0f) continue;
                    obstacleAvoidanceQuery.addSegment(s.start, s.end);
                }

                ObstacleAvoidanceDebugData vod = null;
                if (debugAgent == ag) {
                    vod = debug.vod;
                }

                // Sample new safe velocity.

                ObstacleAvoidanceParams params = m_obstacleQueryParams[ag.params.obstacleAvoidanceType];
                Pair<Integer, Vector3f> nsnvel = obstacleAvoidanceQuery.sampleVelocityAdaptive(ag.currentPosition, ag.params.radius, ag.desiredSpeed, ag.actualVelocity, ag.desiredVelocity, params, vod);
                ag.desiredVelAdjusted = nsnvel.second;
                m_velocitySampleCount += nsnvel.first;
            } else {
                // If not using velocity planning, new velocity is directly the desired velocity.
                copy(ag.desiredVelAdjusted, ag.desiredVelocity);
            }
        }
        telemetry.stop("planVelocity");
    }

    private void integrate(float dt, Collection<CrowdAgent> agents) {
        telemetry.start("integrate");
        for (CrowdAgent ag : agents) {
            if (ag.state != CrowdAgentState.WALKING) {
                continue;
            }
            ag.integrate(dt);
        }
        telemetry.stop("integrate");
    }

    private void handleCollisions(Collection<CrowdAgent> agents) {
        telemetry.start("handleCollisions");
        for (int iter = 0; iter < 4; ++iter) {
            for (CrowdAgent ag : agents) {
                long idx0 = ag.idx;
                if (ag.state != CrowdAgentState.WALKING) {
                    continue;
                }

                set(ag.disp, 0, 0, 0);

                float w = 0;
                for (int j = 0; j < ag.neis.size(); ++j) {
                    CrowdAgent nei = ag.neis.get(j).agent;
                    long idx1 = nei.idx;
                    Vector3f diff = sub(ag.currentPosition, nei.currentPosition);
                    diff.y = 0;

                    float dist = diff.lengthSquared();
                    if (dist > sqr(ag.params.radius + nei.params.radius)) {
                        continue;
                    }
                    dist = (float) Math.sqrt(dist);
                    float pen = (ag.params.radius + nei.params.radius) - dist;
                    if (dist < 0.0001f) {
                        // Agents on top of each other, try to choose diverging separation directions.
                        if (idx0 > idx1) {
                            set(diff, -ag.desiredVelocity.z, 0, ag.desiredVelocity.x);
                        } else {
                            set(diff, ag.desiredVelocity.z, 0, -ag.desiredVelocity.x);
                        }
                        pen = 0.01f;
                    } else {
                        pen = (1f / dist) * (pen * 0.5f) * config.collisionResolveFactor;
                    }

                    mad2(ag.disp, diff, pen);

                    w += 1f;
                }

                if (w > 0.0001f) {
                    float iw = 1f / w;
                    ag.disp.mul(iw);
                }
            }

            for (CrowdAgent ag : agents) {
                if (ag.state != CrowdAgentState.WALKING) {
                    continue;
                }

                ag.currentPosition = Vectors.add(ag.currentPosition, ag.disp);
            }
        }

        telemetry.stop("handleCollisions");
    }

    private void moveAgents(Collection<CrowdAgent> agents) {
        telemetry.start("moveAgents");
        for (CrowdAgent ag : agents) {
            if (ag.state != CrowdAgentState.WALKING) {
                continue;
            }

            // Move along navmesh.
            ag.corridor.movePosition(ag.currentPosition, navQuery, m_filters[ag.params.queryFilterType]);
            // Get valid constrained position back.
            copy(ag.currentPosition, ag.corridor.pos);

            // If not using path, truncate the corridor to just one poly.
            if (ag.targetState == MoveRequestState.NONE
                    || ag.targetState == MoveRequestState.VELOCITY) {
                ag.corridor.reset(ag.corridor.getFirstPoly(), ag.currentPosition);
                ag.partial = false;
            }

        }
        telemetry.stop("moveAgents");
    }

    private void updateOffMeshConnections(Collection<CrowdAgent> agents, float dt) {
        telemetry.start("updateOffMeshConnections");
        for (CrowdAgent ag : agents) {
            CrowdAgentAnimation anim = ag.animation;
            if (!anim.active) {
                continue;
            }

            anim.t += dt;
            if (anim.t > anim.tMax) {
                // Reset animation
                anim.active = false;
                // Prepare agent for walking.
                ag.state = CrowdAgentState.WALKING;
                continue;
            }

            // Update position
            float ta = anim.tMax * 0.15f;
            float tb = anim.tMax;
            if (anim.t < ta) {
                float u = tween(anim.t, 0f, ta);
                ag.currentPosition = lerp(anim.initPos, anim.startPos, u);
            } else {
                float u = tween(anim.t, ta, tb);
                ag.currentPosition = lerp(anim.startPos, anim.endPos, u);
            }

            // Update velocity.
            set(ag.actualVelocity, 0, 0, 0);
            set(ag.desiredVelocity, 0, 0, 0);
        }
        telemetry.stop("updateOffMeshConnections");
    }

    private float tween(float t, float t0, float t1) {
        return clamp((t - t0) / (t1 - t0), 0f, 1f);
    }

    /**
     * Provides neighbor data for agents managed by the crowd.
     */
    public static class CrowdNeighbour {
        /**
         * The index of the neighbor in the crowd.
         */
        public final CrowdAgent agent;
        /**
         * The distance between the current agent and the neighbor.
         */
        final float dist;

        public CrowdNeighbour(CrowdAgent agent, float dist) {
            this.agent = agent;
            this.dist = dist;
        }
    }

}
