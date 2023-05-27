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

import java.util.stream.Collectors
import kotlin.math.floor

class ProximityGrid(val cellSize: Float) {

    private val invCellSize = 1f / cellSize
    private val items: MutableMap<ItemKey, MutableList<CrowdAgent>?> = HashMap()

    fun clear() {
        items.clear()
    }

    fun addItem(agent: CrowdAgent, minXf: Float, minYf: Float, maxXf: Float, maxYf: Float) {
        val minX = floor((minXf * invCellSize)).toInt()
        val minY = floor((minYf * invCellSize)).toInt()
        val maxX = floor((maxXf * invCellSize)).toInt()
        val maxY = floor((maxYf * invCellSize)).toInt()
        for (y in minY..maxY) {
            for (x in minX..maxX) {
                val key = ItemKey(x, y)
                val ids = items.computeIfAbsent(key) { ArrayList() }
                ids!!.add(agent)
            }
        }
    }

    fun queryItems(minXf: Float, minYf: Float, maxXf: Float, maxYf: Float): Set<CrowdAgent> {
        val minX = floor((minXf * invCellSize)).toInt()
        val minY = floor((minYf * invCellSize)).toInt()
        val maxX = floor((maxXf * invCellSize)).toInt()
        val maxY = floor((maxYf * invCellSize)).toInt()
        val result: MutableSet<CrowdAgent> = HashSet()
        for (y in minY..maxY) {
            for (x in minX..maxX) {
                val key = ItemKey(x, y)
                val ids: List<CrowdAgent>? = items[key]
                if (ids != null) {
                    result.addAll(ids)
                }
            }
        }
        return result
    }

    val itemCounts: List<IntArray>
        get() = items.entries.stream()
            .filter { (_, value): Map.Entry<ItemKey, List<CrowdAgent>?> -> value != null && value.isNotEmpty() }
            .map { (key, value): Map.Entry<ItemKey, List<CrowdAgent>?> ->
                intArrayOf(
                    key.x, key.y, value!!.size
                )
            }.collect(Collectors.toList())

    private class ItemKey(var x: Int, var y: Int) {
        override fun hashCode(): Int {
            val prime = 31
            var result = 1
            result = prime * result + x
            result = prime * result + y
            return result
        }

        override fun equals(other: Any?): Boolean {
            if (this === other) return true
            if (other == null) return false
            if (javaClass != other.javaClass) return false
            other as ItemKey
            return if (x != other.x) false else y == other.y
        }
    }
}