package org.recast4j.detour.extras.jumplink;

import kotlin.Pair;
import org.joml.Vector3f;
import org.recast4j.detour.*;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

class NavMeshGroundSampler extends AbstractGroundSampler {

    private final QueryFilter filter = new NoOpFilter();

    private static class NoOpFilter implements QueryFilter {

        @Override
        public boolean passFilter(long ref, MeshTile tile, Poly poly) {
            return true;
        }

        @Override
        public float getCost(Vector3f pa, Vector3f pb, long prevRef, MeshTile prevTile, Poly prevPoly, long curRef,
                             MeshTile curTile, Poly curPoly, long nextRef, MeshTile nextTile, Poly nextPoly) {
            return 0;
        }

    }

    @Override
    public void sample(JumpLinkBuilderConfig cfg, RecastBuilderResult result, EdgeSampler es) {
        NavMeshQuery navMeshQuery = createNavMesh(result, cfg.agentRadius, cfg.agentHeight, cfg.agentClimb);
        sampleGround(cfg, es, (pt, h) -> getNavMeshHeight(navMeshQuery, pt, cfg.cellSize, h));
    }

    private NavMeshQuery createNavMesh(RecastBuilderResult r, float agentRadius, float agentHeight, float agentClimb) {
        NavMeshDataCreateParams params = new NavMeshDataCreateParams();
        params.vertices = r.mesh.vertices;
        params.vertCount = r.mesh.numVertices;
        params.polys = r.mesh.polygons;
        params.polyAreas = r.mesh.areaIds;
        params.polyFlags = r.mesh.flags;
        params.polyCount = r.mesh.numPolygons;
        params.maxVerticesPerPolygon = r.mesh.maxVerticesPerPolygon;
        params.detailMeshes = r.meshDetail.meshes;
        params.detailVertices = r.meshDetail.vertices;
        params.detailVerticesCount = r.meshDetail.numVertices;
        params.detailTris = r.meshDetail.triangles;
        params.detailTriCount = r.meshDetail.numTriangles;
        params.walkableRadius = agentRadius;
        params.walkableHeight = agentHeight;
        params.walkableClimb = agentClimb;
        params.bmin = r.mesh.bmin;
        params.bmax = r.mesh.bmax;
        params.cellSize = r.mesh.cellSize;
        params.cellHeight = r.mesh.cellHeight;
        params.buildBvTree = true;
        return new NavMeshQuery(new NavMesh(NavMeshBuilder.createNavMeshData(params), params.maxVerticesPerPolygon, 0));
    }

    private Pair<Boolean, Float> getNavMeshHeight(NavMeshQuery navMeshQuery, Vector3f pt, float cellSize, float heightRange) {
        Vector3f halfExtents = new Vector3f(cellSize, heightRange, cellSize);
        float maxHeight = pt.y + heightRange;
        AtomicBoolean found = new AtomicBoolean();
        AtomicReference<Float> minHeight = new AtomicReference<>(pt.y);
        navMeshQuery.queryPolygons(pt, halfExtents, filter, (tile, poly, ref) -> {
            float y = navMeshQuery.getPolyHeight(ref, pt);
            if (Float.isFinite(y)) {
                if (y > minHeight.get() && y < maxHeight) {
                    minHeight.set(y);
                    found.set(true);
                }
            }
        });
        if (found.get()) {
            return new Pair<>(true, minHeight.get());
        }
        return new Pair<>(false, pt.y);
    }

}
