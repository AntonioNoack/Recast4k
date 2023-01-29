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
package org.recast4j.detour.tilecache.io

import org.recast4j.detour.tilecache.TileCacheLayerHeader
import java.io.IOException
import java.nio.ByteBuffer

class TileCacheLayerHeaderReader {
    fun read(data: ByteBuffer, cCompatibility: Boolean): TileCacheLayerHeader {
        val header = TileCacheLayerHeader()
        header.magic = data.int
        header.version = data.int
        if (header.magic != TileCacheLayerHeader.DT_TILECACHE_MAGIC) throw IOException("Invalid magic")
        if (header.version != TileCacheLayerHeader.DT_TILECACHE_VERSION) throw IOException("Invalid version")
        header.tx = data.int
        header.ty = data.int
        header.tlayer = data.int
        header.bmin.set(data.float, data.float, data.float)
        header.bmax.set(data.float, data.float, data.float)
        header.hmin = data.short.toInt() and 0xFFFF
        header.hmax = data.short.toInt() and 0xFFFF
        header.width = data.get().toInt() and 0xFF
        header.height = data.get().toInt() and 0xFF
        header.minx = data.get().toInt() and 0xFF
        header.maxx = data.get().toInt() and 0xFF
        header.miny = data.get().toInt() and 0xFF
        header.maxy = data.get().toInt() and 0xFF
        if (cCompatibility) {
            data.short // C struct padding
        }
        return header
    }
}