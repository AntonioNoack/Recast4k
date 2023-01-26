/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j Copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.recast

import org.joml.Vector3f
import org.recast4j.recast.RecastConstants.PartitionType
import org.recast4j.recast.geom.ConvexVolumeProvider
import org.recast4j.recast.geom.InputGeomProvider
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executor
import java.util.concurrent.atomic.AtomicInteger

class RecastBuilder {
    interface RecastBuilderProgressListener {
        fun onProgress(completed: Int, total: Int)
    }

    private val progressListener: RecastBuilderProgressListener?

    constructor() {
        progressListener = null
    }

    constructor(progressListener: RecastBuilderProgressListener?) {
        this.progressListener = progressListener
    }

    class RecastBuilderResult(
        val tileX: Int,
        val tileZ: Int,
        val solidHeightField: Heightfield,
        val compactHeightField: CompactHeightfield,
        val countourSet: ContourSet,
        val mesh: PolyMesh,
        val meshDetail: PolyMeshDetail?,
        val telemetry: Telemetry
    )

    fun buildTiles(geom: InputGeomProvider, cfg: RecastConfig, executor: Executor?): List<RecastBuilderResult> {
        val bmin = geom.meshBoundsMin
        val bmax = geom.meshBoundsMax
        val tw = Recast.calcTileCountX(bmin, bmax, cfg.cellSize, cfg.tileSizeX)
        val th = Recast.calcTileCountY(bmin, bmax, cfg.cellSize, cfg.tileSizeZ)
        return executor?.let { buildMultiThread(geom, cfg, bmin, bmax, tw, th, it) }
            ?: buildSingleThread(geom, cfg, bmin, bmax, tw, th)
    }

    private fun buildSingleThread(
        geom: InputGeomProvider, cfg: RecastConfig, bmin: Vector3f, bmax: Vector3f,
        tw: Int, th: Int
    ): List<RecastBuilderResult> {
        val result: MutableList<RecastBuilderResult> = ArrayList(tw * th)
        val counter = AtomicInteger()
        for (y in 0 until th) {
            for (x in 0 until tw) {
                result.add(buildTile(geom, cfg, bmin, bmax, x, y, counter, tw * th))
            }
        }
        return result
    }

    private fun buildMultiThread(
        geom: InputGeomProvider, cfg: RecastConfig, bmin: Vector3f, bmax: Vector3f,
        tw: Int, th: Int, executor: Executor
    ): List<RecastBuilderResult> {
        val result: MutableList<RecastBuilderResult> = ArrayList(tw * th)
        val counter = AtomicInteger()
        val latch = CountDownLatch(tw * th)
        for (x in 0 until tw) {
            for (y in 0 until th) {
                executor.execute {
                    try {
                        val tile = buildTile(geom, cfg, bmin, bmax, x, y, counter, tw * th)
                        synchronized(result) { result.add(tile) }
                    } catch (e: Exception) {
                        e.printStackTrace()
                    }
                    latch.countDown()
                }
            }
        }
        try {
            latch.await()
        } catch (ignored: InterruptedException) {
        }
        return result
    }

    private fun buildTile(
        geom: InputGeomProvider, cfg: RecastConfig, bmin: Vector3f, bmax: Vector3f, tx: Int,
        ty: Int, counter: AtomicInteger, total: Int
    ): RecastBuilderResult {
        val result = build(geom, RecastBuilderConfig(cfg, bmin, bmax, tx, ty))
        progressListener?.onProgress(counter.incrementAndGet(), total)
        return result
    }

    fun build(geom: InputGeomProvider, builderCfg: RecastBuilderConfig): RecastBuilderResult {
        val cfg = builderCfg.cfg
        val ctx = Telemetry()
        //
        // Step 1. Rasterize input polygon soup.
        //
        val solid = RecastVoxelization.buildSolidHeightfield(geom, builderCfg, ctx)
        return build(builderCfg.tileX, builderCfg.tileZ, geom, cfg, solid, ctx)
    }

    fun build(
        tileX: Int, tileZ: Int, geom: ConvexVolumeProvider?, cfg: RecastConfig, solid: Heightfield,
        ctx: Telemetry
    ): RecastBuilderResult {
        filterHeightfield(solid, cfg, ctx)
        val chf = buildCompactHeightfield(geom, cfg, ctx, solid)

        // Partition the heightfield so that we can use simple algorithm later
        // to triangulate the walkable areas.
        // There are 3 martitioning methods, each with some pros and cons:
        // 1) Watershed partitioning
        // - the classic Recast partitioning
        // - creates the nicest tessellation
        // - usually slowest
        // - partitions the heightfield into nice regions without holes or
        // overlaps
        // - the are some corner cases where this method creates produces holes
        // and overlaps
        // - holes may appear when a small obstacles is close to large open area
        // (triangulation can handle this)
        // - overlaps may occur if you have narrow spiral corridors (i.e
        // stairs), this make triangulation to fail
        // * generally the best choice if you precompute the nacmesh, use this
        // if you have large open areas
        // 2) Monotone partioning
        // - fastest
        // - partitions the heightfield into regions without holes and overlaps
        // (guaranteed)
        // - creates long thin polygons, which sometimes causes paths with
        // detours
        // * use this if you want fast navmesh generation
        // 3) Layer partitoining
        // - quite fast
        // - partitions the heighfield into non-overlapping regions
        // - relies on the triangulation code to cope with holes (thus slower
        // than monotone partitioning)
        // - produces better triangles than monotone partitioning
        // - does not have the corner cases of watershed partitioning
        // - can be slow and create a bit ugly tessellation (still better than
        // monotone)
        // if you have large open areas with small obstacles (not a problem if
        // you use tiles)
        // * good choice to use for tiled navmesh with medium and small sized
        // tiles
        if (cfg.partitionType == PartitionType.WATERSHED) {
            // Prepare for region partitioning, by calculating distance field
            // along the walkable surface.
            RecastRegion.buildDistanceField(ctx, chf)
            // Partition the walkable surface into simple regions without holes.
            RecastRegion.buildRegions(ctx, chf, cfg.minRegionArea, cfg.mergeRegionArea)
        } else if (cfg.partitionType == PartitionType.MONOTONE) {
            // Partition the walkable surface into simple regions without holes.
            // Monotone partitioning does not need distancefield.
            RecastRegion.buildRegionsMonotone(ctx, chf, cfg.minRegionArea, cfg.mergeRegionArea)
        } else {
            // Partition the walkable surface into simple regions without holes.
            RecastRegion.buildLayerRegions(ctx, chf, cfg.minRegionArea)
        }

        //
        // Step 5. Trace and simplify region contours.
        //

        // Create contours.
        val cset = RecastContour.buildContours(
            ctx, chf, cfg.maxSimplificationError, cfg.maxEdgeLen,
            RecastConstants.RC_CONTOUR_TESS_WALL_EDGES
        )

        //
        // Step 6. Build polygons mesh from contours.
        //
        val pmesh = RecastMesh.buildPolyMesh(ctx, cset, cfg.maxVerticesPerPoly)

        //
        // Step 7. Create detail mesh which allows to access approximate height
        // on each polygon.
        //
        val dmesh = if (cfg.buildMeshDetail)
            RecastMeshDetail.buildPolyMeshDetail(ctx, pmesh, chf, cfg.detailSampleDist, cfg.detailSampleMaxError)
        else null
        return RecastBuilderResult(tileX, tileZ, solid, chf, cset, pmesh, dmesh, ctx)
    }

    /*
     * Step 2. Filter walkable surfaces.
     */
    private fun filterHeightfield(solid: Heightfield, cfg: RecastConfig, ctx: Telemetry) {
        // Once all geometry is rasterized, we do initial pass of filtering to
        // remove unwanted overhangs caused by the conservative rasterization
        // as well as filter spans where the character cannot possibly stand.
        if (cfg.filterLowHangingObstacles) {
            RecastFilter.filterLowHangingWalkableObstacles(ctx, cfg.walkableClimb, solid)
        }
        if (cfg.filterLedgeSpans) {
            RecastFilter.filterLedgeSpans(ctx, cfg.walkableHeight, cfg.walkableClimb, solid)
        }
        if (cfg.filterWalkableLowHeightSpans) {
            RecastFilter.filterWalkableLowHeightSpans(ctx, cfg.walkableHeight, solid)
        }
    }

    /** Step 3. Partition walkable surface to simple regions.  */
    private fun buildCompactHeightfield(
        volumeProvider: ConvexVolumeProvider?, cfg: RecastConfig, ctx: Telemetry,
        solid: Heightfield
    ): CompactHeightfield {
        // Compact the heightfield so that it is faster to handle from now on.
        // This will result more cache coherent data as well as the neighbours
        // between walkable cells will be calculated.
        val chf = RecastCompact.buildCompactHeightfield(ctx, cfg.walkableHeight, cfg.walkableClimb, solid)

        // Erode the walkable area by agent radius.
        RecastArea.erodeWalkableArea(ctx, cfg.walkableRadius, chf)
        // (Optional) Mark areas.
        if (volumeProvider != null) {
            for (vol in volumeProvider.convexVolumes()) {
                RecastArea.markConvexPolyArea(ctx, vol.vertices, vol.minH, vol.maxH, vol.areaMod, chf)
            }
        }
        return chf
    }

    fun buildLayers(geom: InputGeomProvider, builderCfg: RecastBuilderConfig): Array<HeightfieldLayer>? {
        val ctx = Telemetry()
        val solid = RecastVoxelization.buildSolidHeightfield(geom, builderCfg, ctx)
        filterHeightfield(solid, builderCfg.cfg, ctx)
        val chf = buildCompactHeightfield(geom, builderCfg.cfg, ctx, solid)
        return RecastLayers.buildHeightfieldLayers(ctx, chf, builderCfg.cfg.walkableHeight)
    }
}