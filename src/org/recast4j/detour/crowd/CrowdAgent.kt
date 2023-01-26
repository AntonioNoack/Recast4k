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
package org.recast4j.detour.crowd

import org.joml.Vector3f
import org.recast4j.Vectors
import org.recast4j.detour.NavMeshQuery
import org.recast4j.detour.StraightPathItem
import org.recast4j.detour.crowd.Crowd.CrowdNeighbour
import kotlin.math.min

/**
 * Represents an agent managed by a #dtCrowd object.
 */
class CrowdAgent(idx: Int) {
    /**
     * The type of navigation mesh polygon the agent is currently traversing.
     */
    enum class CrowdAgentState {
        /** The agent is not in a valid state.  */
        INVALID,
        /** The agent is traversing a normal navigation mesh polygon.  */
        WALKING,
        /** The agent is traversing an off-mesh connection.  */
        OFFMESH
    }

    enum class MoveRequestState {
        NONE, FAILED, VALID, REQUESTING, WAITING_FOR_QUEUE, WAITING_FOR_PATH, VELOCITY
    }

    val idx: Long

    /** The type of mesh polygon the agent is traversing. (See: #CrowdAgentState)  */
    var state: CrowdAgentState? = null

    /**
     * True if the agent has valid path (targetState == CROWDAGENT_TARGET_VALID), and the path does not lead to the
     * requested position, else false.
     */
    var partial = false

    /** The path corridor the agent is using.  */
    var corridor: PathCorridor

    /** The local boundary data for the agent.  */
    var boundary: LocalBoundary

    /** Time since the agent's path corridor was optimized.  */
    var topologyOptTime = 0f

    /** The known neighbors of the agent.  */
    var neis = ArrayList<CrowdNeighbour>()

    /** The desired speed.  */
    var desiredSpeed = 0f
    var currentPosition = Vector3f()

    /** A temporary value used to accumulate agent displacement during iterative collision resolution  */
    var disp = Vector3f()

    /** The desired velocity of the agent. Based on the current path, calculated from scratch each frame.  */
    var desiredVelocity = Vector3f()

    /** The desired velocity adjusted by obstacle avoidance, calculated from scratch each frame  */
    var desiredVelAdjusted = Vector3f()

    /** The actual velocity of the agent. The change from nvel -> vel is constrained by max acceleration  */
    var actualVelocity = Vector3f()

    /** The agent's configuration parameters.  */
    lateinit var params: CrowdAgentParams

    /** The local path corridor corners for the agent.  */
    var corners = ArrayList<StraightPathItem>()

    /** State of the movement request.  */
    var targetState: MoveRequestState? = null

    /** Target polyref of the movement request.  */
    var targetRef: Long = 0

    /** Target position of the movement request (or velocity in case of CROWDAGENT_TARGET_VELOCITY).  */
    val targetPos = Vector3f()

    /** Pathfinder query  */
    var targetPathQueryResult: PathQueryResult? = null

    /** Flag indicating that the current path is being replanned.  */
    var targetReplan = false

    /** Time since the agent's target was replanned.  */
    var targetReplanTime = 0f
    var targetReplanWaitTime = 0f
    var animation: CrowdAgentAnimation

    init {
        this.idx = idx.toLong()
        corridor = PathCorridor()
        boundary = LocalBoundary()
        animation = CrowdAgentAnimation()
    }

    fun integrate(dt: Float) {
        // Fake dynamic constraint.
        val maxDelta = params.maxAcceleration * dt
        val dv = Vectors.sub(desiredVelAdjusted, actualVelocity)
        val ds = dv.length()
        if (ds > maxDelta) dv.mul(maxDelta / ds)
        actualVelocity.add(dv)

        // Integrate
        if (actualVelocity.length() > 0.0001f) Vectors.mad2(
            currentPosition,
            actualVelocity,
            dt
        ) else actualVelocity.set(0f)
    }

    fun overOffmeshConnection(radius: Float): Boolean {
        if (corners.isEmpty()) return false
        val offMeshConnection = corners[corners.size - 1].flags and NavMeshQuery.DT_STRAIGHTPATH_OFFMESH_CONNECTION != 0
        if (offMeshConnection) {
            val distSq = Vectors.dist2DSqr(currentPosition, corners[corners.size - 1].pos)
            return distSq < radius * radius
        }
        return false
    }

    fun getDistanceToGoal(range: Float): Float {
        if (corners.isEmpty()) return range
        val endOfPath = corners[corners.size - 1].flags and NavMeshQuery.DT_STRAIGHTPATH_END != 0
        return if (endOfPath) min(Vectors.dist2D(currentPosition, corners[corners.size - 1].pos), range) else range
    }

    fun calcSmoothSteerDirection(dst: Vector3f): Vector3f {
        if (corners.isNotEmpty()) {
            val ip0 = 0
            val ip1 = min(1, corners.size - 1)
            val p0 = corners[ip0].pos
            val p1 = corners[ip1].pos
            val dir0 = Vectors.sub(p0, currentPosition)
            val dir1 = Vectors.sub(p1, currentPosition)
            dir0.y = 0f
            dir1.y = 0f
            val len0 = dir0.length()
            val len1 = dir1.length()
            if (len1 > 0.001f) dir1.mul(1f / len1)
            dst.x = dir0.x - dir1.x * len0 * 0.5f
            dst.y = 0f
            dst.z = dir0.z - dir1.z * len0 * 0.5f
            dst.normalize()
        }
        return dst
    }

    fun calcStraightSteerDirection(dst: Vector3f): Vector3f {
        if (corners.isNotEmpty()) {
            dst.set(corners[0].pos).sub(currentPosition)
            dst.y = 0f
            dst.normalize()
        }
        return dst
    }

    fun setTarget(ref: Long, pos: Vector3f) {
        targetRef = ref
        targetPos.set(pos)
        targetPathQueryResult = null
        targetState = if (targetRef != 0L) MoveRequestState.REQUESTING else MoveRequestState.FAILED
    }
}