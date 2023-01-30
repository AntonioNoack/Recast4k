package org.recast4j.detour.extras.jumplink

import org.joml.Vector3f
import org.recast4j.detour.*
import org.recast4j.recast.RecastBuilder.RecastBuilderResult
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference

internal class NavMeshGroundSampler : AbstractGroundSampler() {
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
        r: RecastBuilderResult?,
        agentRadius: Float,
        agentHeight: Float,
        agentClimb: Float
    ): NavMeshQuery {
        val params = NavMeshDataCreateParams()
        params.vertices = r!!.mesh.vertices
        params.vertCount = r.mesh.numVertices
        params.polys = r.mesh.polygons
        params.polyAreas = r.mesh.areaIds
        params.polyFlags = r.mesh.flags
        params.polyCount = r.mesh.numPolygons
        params.maxVerticesPerPolygon = r.mesh.maxVerticesPerPolygon
        params.detailMeshes = r.meshDetail!!.subMeshes
        params.detailVertices = r.meshDetail.vertices
        params.detailVerticesCount = r.meshDetail.numVertices
        params.detailTris = r.meshDetail.triangles
        params.detailTriCount = r.meshDetail.numTriangles
        params.walkableRadius = agentRadius
        params.walkableHeight = agentHeight
        params.walkableClimb = agentClimb
        params.bmin = r.mesh.bmin
        params.bmax = r.mesh.bmax
        params.cellSize = r.mesh.cellSize
        params.cellHeight = r.mesh.cellHeight
        params.buildBvTree = true
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