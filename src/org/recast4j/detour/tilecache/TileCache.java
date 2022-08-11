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
package org.recast4j.detour.tilecache;

import org.joml.Vector3f;
import org.recast4j.LongArrayList;
import org.recast4j.detour.MeshData;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshBuilder;
import org.recast4j.detour.NavMeshDataCreateParams;
import org.recast4j.detour.tilecache.TileCacheObstacle.TileCacheObstacleType;
import org.recast4j.detour.tilecache.io.TileCacheLayerHeaderReader;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import static org.recast4j.Vectors.*;

public class TileCache {

    int tileLutSize; /// < Tile hash lookup size (must be pot).
    int tileLutMask; /// < Tile hash lookup mask.

    private final CompressedTile[] posLookup; /// < Tile hash lookup.
    private CompressedTile nextFreeTile; /// < Freelist of tiles.
    private final CompressedTile[] tiles; /// < List of tiles. // TODO: (PP) replace with list

    private final int m_saltBits; /// < Number of salt bits in the tile ID.
    private final int m_tileBits; /// < Number of tile bits in the tile ID.

    private final NavMesh m_navmesh;
    private final TileCacheParams params;
    private final TileCacheStorageParams storageParams;

    private final TileCacheMeshProcess tmProcess;

    private final List<TileCacheObstacle> obstacles = new ArrayList<>();
    private TileCacheObstacle nextFreeObstacle;

    private final List<ObstacleRequest> requests = new ArrayList<>();
    private final LongArrayList update = new LongArrayList();

    private final TileCacheBuilder builder = new TileCacheBuilder();
    private final TileCacheLayerHeaderReader tileReader = new TileCacheLayerHeaderReader();

    private boolean contains(LongArrayList a, long v) {
        return a.contains(v);
    }

    /// Encodes a tile id.
    private long encodeTileId(int salt, int it) {
        return ((long) salt << m_tileBits) | it;
    }

    /// Decodes a tile salt.
    private int decodeTileIdSalt(long ref) {
        long saltMask = (1L << m_saltBits) - 1;
        return (int) ((ref >> m_tileBits) & saltMask);
    }

    /// Decodes a tile id.
    private int decodeTileIdTile(long ref) {
        long tileMask = (1L << m_tileBits) - 1;
        return (int) (ref & tileMask);
    }

    /// Encodes an obstacle id.
    private long encodeObstacleId(int salt, int it) {
        return ((long) salt << 16) | it;
    }

    /// Decodes an obstacle salt.
    private int decodeObstacleIdSalt(long ref) {
        long saltMask = ((long) 1 << 16) - 1;
        return (int) ((ref >> 16) & saltMask);
    }

    /// Decodes an obstacle id.
    private int decodeObstacleIdObstacle(long ref) {
        long tileMask = ((long) 1 << 16) - 1;
        return (int) (ref & tileMask);
    }

    public TileCache(TileCacheParams params, TileCacheStorageParams storageParams, NavMesh navmesh,
                     TileCacheMeshProcess tmprocs) {
        this.params = params;
        this.storageParams = storageParams;
        m_navmesh = navmesh;
        tmProcess = tmprocs;

        tileLutSize = nextPow2(this.params.maxTiles / 4);
        if (tileLutSize == 0) {
            tileLutSize = 1;
        }
        tileLutMask = tileLutSize - 1;
        tiles = new CompressedTile[this.params.maxTiles];
        posLookup = new CompressedTile[tileLutSize];
        for (int i = this.params.maxTiles - 1; i >= 0; --i) {
            tiles[i] = new CompressedTile(i);
            tiles[i].next = nextFreeTile;
            nextFreeTile = tiles[i];
        }
        m_tileBits = ilog2(nextPow2(this.params.maxTiles));
        m_saltBits = Math.min(31, 32 - m_tileBits);
        if (m_saltBits < 10) {
            throw new RuntimeException("Too few salt bits: " + m_saltBits);
        }
    }

    @SuppressWarnings("unused")
    public CompressedTile getTileByRef(long ref) {
        if (ref == 0) return null;
        int tileIndex = decodeTileIdTile(ref);
        int tileSalt = decodeTileIdSalt(ref);
        if (tileIndex >= params.maxTiles) return null;
        CompressedTile tile = tiles[tileIndex];
        if (tile.salt != tileSalt) return null;
        return tile;
    }

    public LongArrayList getTilesAt(int tx, int ty) {
        LongArrayList tiles = new LongArrayList();

        // Find tile based on hash.
        int h = NavMesh.computeTileHash(tx, ty, tileLutMask);
        CompressedTile tile = posLookup[h];
        while (tile != null) {
            if (tile.header != null && tile.header.tx == tx && tile.header.ty == ty) {
                tiles.add(getTileRef(tile));
            }
            tile = tile.next;
        }

        return tiles;
    }

    CompressedTile getTileAt(int tx, int ty, int tlayer) {
        // Find tile based on hash.
        int h = NavMesh.computeTileHash(tx, ty, tileLutMask);
        CompressedTile tile = posLookup[h];
        while (tile != null) {
            if (tile.header != null && tile.header.tx == tx && tile.header.ty == ty && tile.header.tlayer == tlayer) {
                return tile;
            }
            tile = tile.next;
        }
        return null;
    }

    public long getTileRef(CompressedTile tile) {
        if (tile == null) {
            return 0;
        }
        int it = tile.index;
        return encodeTileId(tile.salt, it);
    }

    public long getObstacleRef(TileCacheObstacle ob) {
        if (ob == null) return 0;
        int idx = ob.index;
        return encodeObstacleId(ob.salt, idx);
    }

    @SuppressWarnings("unused")
    public TileCacheObstacle getObstacleByRef(long ref) {
        if (ref == 0) return null;
        int idx = decodeObstacleIdObstacle(ref);
        if (idx >= obstacles.size()) return null;
        TileCacheObstacle ob = obstacles.get(idx);
        int salt = decodeObstacleIdSalt(ref);
        if (ob.salt != salt) return null;
        return ob;
    }

    public long addTile(byte[] data, int flags) throws IOException {
        // Make sure the data is in right format.
        ByteBuffer buf = ByteBuffer.wrap(data);
        buf.order(storageParams.byteOrder);
        TileCacheLayerHeader header = tileReader.read(buf, storageParams.cCompatibility);
        // Make sure the location is free.
        if (getTileAt(header.tx, header.ty, header.tlayer) != null) {
            return 0;
        }
        // Allocate a tile.
        CompressedTile tile = null;
        if (nextFreeTile != null) {
            tile = nextFreeTile;
            nextFreeTile = tile.next;
            tile.next = null;
        }

        // Make sure we could allocate a tile.
        if (tile == null) {
            throw new RuntimeException("Out of storage");
        }

        // Insert tile into the position lut.
        int h = NavMesh.computeTileHash(header.tx, header.ty, tileLutMask);
        tile.next = posLookup[h];
        posLookup[h] = tile;

        // Init tile.
        tile.header = header;
        tile.data = data;
        tile.compressed = align4(buf.position());
        tile.flags = flags;

        return getTileRef(tile);
    }

    private int align4(int i) {
        return (i + 3) & (~3);
    }

    @SuppressWarnings("unused")
    public void removeTile(long ref) {
        if (ref == 0) {
            throw new RuntimeException("Invalid tile ref");
        }
        int tileIndex = decodeTileIdTile(ref);
        int tileSalt = decodeTileIdSalt(ref);
        if (tileIndex >= params.maxTiles) {
            throw new RuntimeException("Invalid tile index");
        }
        CompressedTile tile = tiles[tileIndex];
        if (tile.salt != tileSalt) {
            throw new RuntimeException("Invalid tile salt");
        }

        // Remove tile from hash lookup.
        int h = NavMesh.computeTileHash(tile.header.tx, tile.header.ty, tileLutMask);
        CompressedTile prev = null;
        CompressedTile cur = posLookup[h];
        while (cur != null) {
            if (cur == tile) {
                if (prev != null) {
                    prev.next = cur.next;
                } else {
                    posLookup[h] = cur.next;
                }
                break;
            }
            prev = cur;
            cur = cur.next;
        }

        tile.header = null;
        tile.data = null;
        tile.compressed = 0;
        tile.flags = 0;

        // Update salt, salt should never be zero.
        tile.salt = (tile.salt + 1) & ((1 << m_saltBits) - 1);
        if (tile.salt == 0) {
            tile.salt++;
        }

        // Add to free list.
        tile.next = nextFreeTile;
        nextFreeTile = tile;

    }

    /**
     * Cylinder obstacle
     */
    @SuppressWarnings("unused")
    public long addObstacle(Vector3f pos, float radius, float height) {
        TileCacheObstacle ob = allocObstacle();
        ob.type = TileCacheObstacleType.CYLINDER;

        ob.pos.set(pos);
        ob.radius = radius;
        ob.height = height;

        return addObstacleRequest(ob).ref;
    }

    /**
     * Aabb obstacle
     */
    @SuppressWarnings("unused")
    public long addBoxObstacle(float[] bmin, float[] bmax) {
        TileCacheObstacle ob = allocObstacle();
        ob.type = TileCacheObstacleType.BOX;

        ob.bmin.set(bmin);
        ob.bmax.set(bmax);

        return addObstacleRequest(ob).ref;
    }

    /**
     * Box obstacle: can be rotated in Y
     */
    @SuppressWarnings("unused")
    public long addBoxObstacle(Vector3f center, float[] extents, float yRadians) {
        TileCacheObstacle ob = allocObstacle();
        ob.type = TileCacheObstacleType.ORIENTED_BOX;
        ob.center.set(center);
        ob.extents.set(extents);
        float coshalf = (float) Math.cos(0.5f * yRadians);
        float sinhalf = (float) Math.sin(-0.5f * yRadians);
        ob.rotAux[0] = coshalf * sinhalf;
        ob.rotAux[1] = coshalf * coshalf - 0.5f;
        return addObstacleRequest(ob).ref;
    }

    private ObstacleRequest addObstacleRequest(TileCacheObstacle ob) {
        ObstacleRequest req = new ObstacleRequest();
        req.action = ObstacleRequestAction.REQUEST_ADD;
        req.ref = getObstacleRef(ob);
        requests.add(req);
        return req;
    }

    @SuppressWarnings("unused")
    public void removeObstacle(long ref) {
        if (ref == 0) {
            return;
        }

        ObstacleRequest req = new ObstacleRequest();
        req.action = ObstacleRequestAction.REQUEST_REMOVE;
        req.ref = ref;
        requests.add(req);
    }

    private TileCacheObstacle allocObstacle() {
        TileCacheObstacle o = nextFreeObstacle;
        if (o == null) {
            o = new TileCacheObstacle(obstacles.size());
            obstacles.add(o);
        } else {
            nextFreeObstacle = o.next;
        }
        o.state = ObstacleState.DT_OBSTACLE_PROCESSING;
        o.touched.clear();
        o.pending.clear();
        o.next = null;
        return o;
    }

    LongArrayList queryTiles(Vector3f bmin, Vector3f bmax) {
        LongArrayList results = new LongArrayList();
        float tw = params.width * params.cellSize;
        float th = params.height * params.cellSize;
        int tx0 = (int) Math.floor((bmin.x - params.orig.x) / tw);
        int tx1 = (int) Math.floor((bmax.x - params.orig.x) / tw);
        int ty0 = (int) Math.floor((bmin.z - params.orig.z) / th);
        int ty1 = (int) Math.floor((bmax.z - params.orig.z) / th);
        Vector3f tbmin = new Vector3f();
        Vector3f tbmax = new Vector3f();
        for (int ty = ty0; ty <= ty1; ++ty) {
            for (int tx = tx0; tx <= tx1; ++tx) {
                LongArrayList tiles = getTilesAt(tx, ty);
                for (int i = 0, l = tiles.getSize(); i < l; i++) {
                    long t = tiles.get(i);
                    CompressedTile tile = this.tiles[decodeTileIdTile(t)];
                    calcTightTileBounds(tile.header, tbmin, tbmax);
                    if (overlapBounds(bmin, bmax, tbmin, tbmax)) {
                        results.add(t);
                    }
                }
            }
        }
        return results;
    }

    /**
     * Updates the tile cache by rebuilding tiles touched by unfinished obstacle requests.
     *
     * @return Returns true if the tile cache is fully up to date with obstacle requests and tile rebuilds. If the tile
     * cache is up to date another (immediate) call to update will have no effect; otherwise another call will
     * continue processing obstacle requests and tile rebuilds.
     */
    public boolean update() {
        if (update.isEmpty()) {
            // Process requests.
            for (ObstacleRequest req : requests) {
                int idx = decodeObstacleIdObstacle(req.ref);
                if (idx >= obstacles.size()) {
                    continue;
                }
                TileCacheObstacle ob = obstacles.get(idx);
                int salt = decodeObstacleIdSalt(req.ref);
                if (ob.salt != salt) {
                    continue;
                }

                if (req.action == ObstacleRequestAction.REQUEST_ADD) {
                    // Find touched tiles.
                    Vector3f bmin = new Vector3f();
                    Vector3f bmax = new Vector3f();
                    getObstacleBounds(ob, bmin, bmax);
                    ob.touched = queryTiles(bmin, bmax);
                    // Add tiles to update list.
                    ob.pending.clear();
                    for (int i = 0, l = ob.touched.getSize(); i < l; i++) {
                        long j = ob.touched.get(i);
                        if (!contains(update, j)) {
                            update.add(j);
                        }
                        ob.pending.add(j);
                    }
                } else if (req.action == ObstacleRequestAction.REQUEST_REMOVE) {
                    // Prepare to remove obstacle.
                    ob.state = ObstacleState.DT_OBSTACLE_REMOVING;
                    // Add tiles to update list.
                    ob.pending.clear();
                    for (int i = 0, l = ob.touched.getSize(); i < l; i++) {
                        long j = ob.touched.get(i);
                        if (!contains(update, j)) {
                            update.add(j);
                        }
                        ob.pending.add(j);
                    }
                }
            }

            requests.clear();
        }

        // Process updates
        if (!update.isEmpty()) {
            long ref = update.remove(0);
            // Build mesh
            buildNavMeshTile(ref);

            // Update obstacle states.
            for (TileCacheObstacle ob : obstacles) {
                if (ob.state == ObstacleState.DT_OBSTACLE_PROCESSING
                        || ob.state == ObstacleState.DT_OBSTACLE_REMOVING) {
                    // Remove handled tile from pending list.
                    ob.pending.remove(ref);

                    // If all pending tiles processed, change state.
                    if (ob.pending.isEmpty()) {
                        if (ob.state == ObstacleState.DT_OBSTACLE_PROCESSING) {
                            ob.state = ObstacleState.DT_OBSTACLE_PROCESSED;
                        } else if (ob.state == ObstacleState.DT_OBSTACLE_REMOVING) {
                            ob.state = ObstacleState.DT_OBSTACLE_EMPTY;
                            // Update salt, salt should never be zero.
                            ob.salt = (ob.salt + 1) & ((1 << 16) - 1);
                            if (ob.salt == 0) {
                                ob.salt++;
                            }
                            // Return obstacle to free list.
                            ob.next = nextFreeObstacle;
                            nextFreeObstacle = ob;
                        }
                    }
                }
            }
        }
        return update.isEmpty() && requests.isEmpty();

    }

    public void buildNavMeshTile(long ref) {
        int idx = decodeTileIdTile(ref);
        if (idx > params.maxTiles) {
            throw new RuntimeException("Invalid tile index");
        }
        CompressedTile tile = tiles[idx];
        int salt = decodeTileIdSalt(ref);
        if (tile.salt != salt) {
            throw new RuntimeException("Invalid tile salt");
        }
        int walkableClimbVx = (int) (params.walkableClimb / params.cellHeight);

        // Decompress tile layer data.
        TileCacheLayer layer = decompressTile(tile);

        // Rasterize obstacles.
        for (TileCacheObstacle ob : obstacles) {
            if (ob.state == ObstacleState.DT_OBSTACLE_EMPTY || ob.state == ObstacleState.DT_OBSTACLE_REMOVING) {
                continue;
            }
            if (contains(ob.touched, ref)) {
                if (ob.type == TileCacheObstacleType.CYLINDER) {
                    builder.markCylinderArea(layer, tile.header.bmin, params.cellSize, params.cellHeight, ob.pos, ob.radius,
                            ob.height, 0);
                } else if (ob.type == TileCacheObstacleType.BOX) {
                    builder.markBoxArea(layer, tile.header.bmin, params.cellSize, params.cellHeight, ob.bmin, ob.bmax, 0);
                } else if (ob.type == TileCacheObstacleType.ORIENTED_BOX) {
                    builder.markBoxArea(layer, tile.header.bmin, params.cellSize, params.cellHeight, ob.center, ob.extents,
                            ob.rotAux, 0);
                }
            }
        }
        // Build navmesh
        builder.buildTileCacheRegions(layer, walkableClimbVx);
        TileCacheContourSet lcset = builder.buildTileCacheContours(layer, walkableClimbVx,
                params.maxSimplificationError);
        TileCachePolyMesh polyMesh = builder.buildTileCachePolyMesh(lcset, m_navmesh.maxVerticesPerPoly);
        // Early out if the mesh tile is empty.
        if (polyMesh.numPolygons == 0) {
            m_navmesh.removeTile(m_navmesh.getTileRefAt(tile.header.tx, tile.header.ty, tile.header.tlayer));
            return;
        }
        NavMeshDataCreateParams params = new NavMeshDataCreateParams();
        params.vertices = polyMesh.vertices;
        params.vertCount = polyMesh.numVertices;
        params.polys = polyMesh.polys;
        params.polyAreas = polyMesh.areas;
        params.polyFlags = polyMesh.flags;
        params.polyCount = polyMesh.numPolygons;
        params.maxVerticesPerPolygon = m_navmesh.maxVerticesPerPoly;
        params.walkableHeight = this.params.walkableHeight;
        params.walkableRadius = this.params.walkableRadius;
        params.walkableClimb = this.params.walkableClimb;
        params.tileX = tile.header.tx;
        params.tileZ = tile.header.ty;
        params.tileLayer = tile.header.tlayer;
        params.cellSize = this.params.cellSize;
        params.cellHeight = this.params.cellHeight;
        params.buildBvTree = false;
        params.bmin = tile.header.bmin;
        params.bmax = tile.header.bmax;
        if (tmProcess != null) {
            tmProcess.process(params);
        }
        MeshData meshData = NavMeshBuilder.createNavMeshData(params);
        // Remove existing tile.
        m_navmesh.removeTile(m_navmesh.getTileRefAt(tile.header.tx, tile.header.ty, tile.header.tlayer));
        // Add new tile, or leave the location empty. if (navData) { // Let the
        if (meshData != null) {
            m_navmesh.addTile(meshData, 0, 0);
        }
    }

    public TileCacheLayer decompressTile(CompressedTile tile) {
        return builder.decompressTileCacheLayer(tile.data, storageParams.byteOrder,
                storageParams.cCompatibility);
    }

    void calcTightTileBounds(TileCacheLayerHeader header, Vector3f bmin, Vector3f bmax) {
        float cs = params.cellSize;
        bmin.x = header.bmin.x + header.minx * cs;
        bmin.y = header.bmin.y;
        bmin.z = header.bmin.z + header.miny * cs;
        bmax.x = header.bmin.x + (header.maxx + 1) * cs;
        bmax.y = header.bmax.y;
        bmax.z = header.bmin.z + (header.maxy + 1) * cs;
    }

    void getObstacleBounds(TileCacheObstacle ob, Vector3f bmin, Vector3f bmax) {
        if (ob.type == TileCacheObstacleType.CYLINDER) {
            bmin.x = ob.pos.x - ob.radius;
            bmin.y = ob.pos.y;
            bmin.z = ob.pos.z - ob.radius;
            bmax.x = ob.pos.x + ob.radius;
            bmax.y = ob.pos.y + ob.height;
            bmax.z = ob.pos.z + ob.radius;
        } else if (ob.type == TileCacheObstacleType.BOX) {
            bmin.set(ob.bmin);
            bmax.set(ob.bmax);
        } else if (ob.type == TileCacheObstacleType.ORIENTED_BOX) {
            float maxRadius = 1.41f * Math.max(ob.extents.x, ob.extents.z);
            bmin.x = ob.center.x - maxRadius;
            bmax.x = ob.center.x + maxRadius;
            bmin.y = ob.center.y - ob.extents.y;
            bmax.y = ob.center.y + ob.extents.y;
            bmin.z = ob.center.z - maxRadius;
            bmax.z = ob.center.z + maxRadius;
        }
    }

    public TileCacheParams getParams() {
        return params;
    }

    public int getTileCount() {
        return params.maxTiles;
    }

    public CompressedTile getTile(int i) {
        return tiles[i];
    }

    public NavMesh getNavMesh() {
        return m_navmesh;
    }
}
