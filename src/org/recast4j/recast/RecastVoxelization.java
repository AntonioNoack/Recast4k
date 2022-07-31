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

package org.recast4j.recast;

import org.recast4j.recast.geom.InputGeomProvider;
import org.recast4j.recast.geom.TriMesh;

public class RecastVoxelization {

    public static Heightfield buildSolidHeightfield(InputGeomProvider geomProvider, RecastBuilderConfig builderCfg,
                                                    Telemetry ctx) {
        RecastConfig cfg = builderCfg.cfg;

        // Allocate voxel heightfield where we rasterize our input data to.
        Heightfield solid = new Heightfield(builderCfg.width, builderCfg.height, builderCfg.bmin, builderCfg.bmax, cfg.cellSize,
                cfg.cellHeight, cfg.borderSize);

        // Allocate array that can hold triangle area types.
        // If you have multiple meshes you need to process, allocate
        // and array which can hold the max number of triangles you need to
        // process.

        // Find triangles which are walkable based on their slope and rasterize them.
        // If your input data is multiple meshes, you can transform them here,
        // calculate the are type for each of the meshes and rasterize them.
        for (TriMesh geom : geomProvider.meshes()) {
            float[] vertices = geom.vertices;
            if (cfg.useTiles) {
                float[] bmin = {builderCfg.bmin.x, builderCfg.bmin.z};
                float[] bmax = {builderCfg.bmax.x, builderCfg.bmax.z};
                geom.foreachChunkOverlappingRect(bmin, bmax, node -> {
                    int[] tris = node.triangles;
                    int numTris = tris.length / 3;
                    int[] triAreas = Recast.markWalkableTriangles(cfg.walkableSlopeAngle, vertices, tris, numTris, cfg.walkableAreaMod);
                    RecastRasterization.rasterizeTriangles(solid, vertices, tris, triAreas, numTris, cfg.walkableClimb, ctx);
                });
            } else {
                int[] tris = geom.triangles;
                int numTris = tris.length / 3;
                int[] triAreas = Recast.markWalkableTriangles(cfg.walkableSlopeAngle, vertices, tris, numTris, cfg.walkableAreaMod);
                RecastRasterization.rasterizeTriangles(solid, vertices, tris, triAreas, numTris, cfg.walkableClimb, ctx);
            }
        }

        return solid;
    }

}
