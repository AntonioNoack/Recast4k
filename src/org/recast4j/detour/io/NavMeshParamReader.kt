package org.recast4j.detour.io

import org.recast4j.detour.NavMeshParams
import java.nio.ByteBuffer

class NavMeshParamReader {
    fun read(bb: ByteBuffer): NavMeshParams {
        val params = NavMeshParams()
        params.origin.set(bb.float, bb.float, bb.float)
        params.tileWidth = bb.float
        params.tileHeight = bb.float
        params.maxTiles = bb.int
        params.maxPolys = bb.int
        return params
    }
}