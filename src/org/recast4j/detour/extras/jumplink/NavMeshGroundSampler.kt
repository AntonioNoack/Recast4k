package org.recast4j.detour.extras.jumplink

import org.joml.Vector3f
import org.recast4j.detour.*
import org.recast4j.recast.RecastBuilder.RecastBuilderResult
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference

internal class NavMeshGroundSampler : AbstractGroundSampler() {

    companion object {
        val f0 = FloatArray(0)
        val i0 = IntArray(0)
    }

    private val filter: QueryFilter = NoOpFilter()
    private class NoOpFilter : QueryFilter {

        override fun passFilter(ref: Long, tile: MeshTile?, poly: Poly): Boolean {
            return true
        }

        override fun getCost(
            pa: Vector3f,
            pb: Vector3f,
            prevRef: Long,
            prevTile: MeshTile?,
            prevPoly: Poly?,
            curRef: Long,
            curTile: MeshTile?,
            curPoly: Poly?,
            nextRef: Long,
            nextTile: MeshTile?,
            nextPoly: Poly?
        ): Float {
            return 0f
        }
    }

    override fun sample(cfg: JumpLinkBuilderConfig, result: RecastBuilderResult, es: EdgeSampler) {
        val navMeshQuery = createNavMesh(result, cfg.agentRadius, cfg.agentHeight, cfg.agentClimb)
        sampleGround(cfg, es) { pt: Vector3f, h: Float ->
            getNavMeshHeight(navMeshQuery, pt, cfg.cellSize, h)
        }
    }

    private fun createNavMesh(
        r: RecastBuilderResult,
        agentRadius: Float,
        agentHeight: Float,
        agentClimb: Float
    ): NavMeshQuery {
        val params = NavMeshDataCreateParams(
            r.mesh.vertices,
            r.mesh.numVertices,
            r.mesh.polygons,
            r.mesh.flags,
            r.mesh.areaIds,
            r.mesh.numPolygons,
            r.mesh.maxVerticesPerPolygon,
            r.meshDetail!!.subMeshes,
            r.meshDetail.vertices,
            r.meshDetail.numVertices,
            r.meshDetail.triangles,
            r.meshDetail.numTriangles,
            f0, f0, i0, i0, i0, i0,
            0, 0, 0, 0, 0,
            r.mesh.bmin, r.mesh.bmax,
            agentRadius,
            agentHeight,
            agentClimb,
            r.mesh.cellSize,
            r.mesh.cellHeight,
            true
        )
        return NavMeshQuery(NavMesh(NavMeshBuilder.createNavMeshData(params)!!, params.maxVerticesPerPolygon, 0))
    }

    private fun getNavMeshHeight(
        navMeshQuery: NavMeshQuery,
        pt: Vector3f,
        cellSize: Float,
        heightRange: Float
    ): Pair<Boolean, Float> {
        val halfExtents = Vector3f(cellSize, heightRange, cellSize)
        val maxHeight = pt.y + heightRange
        val found = AtomicBoolean()
        val minHeight = AtomicReference(pt.y)
        navMeshQuery.queryPolygons(pt, halfExtents, filter, object : PolyQuery {
            override fun process(tile: MeshTile, poly: Poly, ref: Long) {
                val y = navMeshQuery.getPolyHeight(ref, pt)
                if (java.lang.Float.isFinite(y)) {
                    if (y > minHeight.get() && y < maxHeight) {
                        minHeight.set(y)
                        found.set(true)
                    }
                }
            }
        })
        return if (found.get()) {
            Pair(true, minHeight.get())
        } else Pair(false, pt.y)
    }
}