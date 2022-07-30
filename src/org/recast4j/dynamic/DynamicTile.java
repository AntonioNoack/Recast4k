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

package org.recast4j.dynamic;

import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.recast4j.detour.MeshData;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshBuilder;
import org.recast4j.detour.NavMeshDataCreateParams;
import org.recast4j.dynamic.collider.Collider;
import org.recast4j.dynamic.io.VoxelTile;
import org.recast4j.recast.Heightfield;
import org.recast4j.recast.PolyMesh;
import org.recast4j.recast.PolyMeshDetail;
import org.recast4j.recast.RecastBuilder;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;
import org.recast4j.recast.RecastConfig;
import org.recast4j.recast.Telemetry;

class DynamicTile {

    final VoxelTile voxelTile;
    DynamicTileCheckpoint checkpoint;
    RecastBuilderResult recastResult;
    MeshData meshData;
    private final Map<Long, Collider> colliders = new ConcurrentHashMap<>();
    private boolean dirty = true;
    private long id;

    DynamicTile(VoxelTile voxelTile) {
        this.voxelTile = voxelTile;
    }

    boolean build(RecastBuilder builder, DynamicNavMeshConfig config, Telemetry telemetry) {
        if (dirty) {
            Heightfield heightfield = buildHeightfield(config, telemetry);
            RecastBuilderResult r = buildRecast(builder, config, voxelTile, heightfield, telemetry);
            NavMeshDataCreateParams params = navMeshCreateParams(voxelTile.tileX, voxelTile.tileZ, voxelTile.cellSize,
                    voxelTile.cellHeight, config, r);
            meshData = NavMeshBuilder.createNavMeshData(params);
            return true;
        }
        return false;
    }

    private Heightfield buildHeightfield(DynamicNavMeshConfig config, Telemetry telemetry) {
        Collection<Long> rasterizedColliders = checkpoint != null ? checkpoint.colliders : Collections.emptySet();
        Heightfield heightfield = checkpoint != null ? checkpoint.heightfield : voxelTile.heightfield();
        colliders.forEach((id, c) -> {
            if (!rasterizedColliders.contains(id)) {
                heightfield.bmax.y = Math.max(heightfield.bmax.y, c.bounds()[4] + heightfield.ch * 2);
                c.rasterize(heightfield, telemetry);
            }
        });
        if (config.enableCheckpoints) {
            checkpoint = new DynamicTileCheckpoint(heightfield, new HashSet<>(colliders.keySet()));
        }
        return heightfield;
    }

    private RecastBuilderResult buildRecast(RecastBuilder builder, DynamicNavMeshConfig config, VoxelTile vt,
            Heightfield heightfield, Telemetry telemetry) {
        RecastConfig rcConfig = new RecastConfig(config.useTiles, config.tileSizeX, config.tileSizeZ, vt.borderSize,
                config.partitionType, vt.cellSize, vt.cellHeight, config.walkableSlopeAngle, true, true, true,
                config.walkableHeight, config.walkableRadius, config.walkableClimb, config.minRegionArea, config.regionMergeArea,
                config.maxEdgeLen, config.maxSimplificationError,
                Math.min(DynamicNavMesh.MAX_VERTICES_PER_POLY, config.verticesPerPoly), true, config.detailSampleDistance,
                config.detailSampleMaxError, null);
        RecastBuilderResult r = builder.build(vt.tileX, vt.tileZ, null, rcConfig, heightfield, telemetry);
        if (config.keepIntermediateResults) {
            recastResult = r;
        }
        return r;
    }

    void addCollider(long cid, Collider collider) {
        colliders.put(cid, collider);
        dirty = true;
    }

    boolean containsCollider(long cid) {
        return colliders.containsKey(cid);
    }

    void removeCollider(long colliderId) {
        if (colliders.remove(colliderId) != null) {
            dirty = true;
            checkpoint = null;
        }
    }

    private NavMeshDataCreateParams navMeshCreateParams(int tilex, int tileZ, float cellSize, float cellHeight,
            DynamicNavMeshConfig config, RecastBuilderResult rcResult) {
        PolyMesh mesh = rcResult.mesh;
        PolyMeshDetail meshDetail = rcResult.meshDetail;
        NavMeshDataCreateParams params = new NavMeshDataCreateParams();
        for (int i = 0; i < mesh.npolys; ++i) {
            mesh.flags[i] = 1;
        }
        params.tileX = tilex;
        params.tileZ = tileZ;
        params.vertices = mesh.vertices;
        params.vertCount = mesh.nvertices;
        params.polys = mesh.polys;
        params.polyAreas = mesh.areas;
        params.polyFlags = mesh.flags;
        params.polyCount = mesh.npolys;
        params.nvp = mesh.nvp;
        if (meshDetail != null) {
            params.detailMeshes = meshDetail.meshes;
            params.detailVertices = meshDetail.vertices;
            params.detailVerticesCount = meshDetail.nvertices;
            params.detailTris = meshDetail.tris;
            params.detailTriCount = meshDetail.ntris;
        }
        params.walkableHeight = config.walkableHeight;
        params.walkableRadius = config.walkableRadius;
        params.walkableClimb = config.walkableClimb;
        params.bmin = mesh.bmin;
        params.bmax = mesh.bmax;
        params.cs = cellSize;
        params.ch = cellHeight;
        params.buildBvTree = true;

        params.offMeshConCount = 0;
        params.offMeshConVertices = new float[0];
        params.offMeshConRad = new float[0];
        params.offMeshConDir = new int[0];
        params.offMeshConAreas = new int[0];
        params.offMeshConFlags = new int[0];
        params.offMeshConUserID = new int[0];
        return params;
    }

    void addTo(NavMesh navMesh) {
        if (meshData != null) {
            id = navMesh.addTile(meshData, 0, 0);
        } else {
            navMesh.removeTile(id);
            id = 0;
        }
    }
}
