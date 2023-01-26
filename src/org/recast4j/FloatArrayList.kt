package org.recast4j

import kotlin.math.max

class FloatArrayList(cap: Int = 16) {

    var values = FloatArray(cap)
    var size = 0

    fun add(v: Float) {
        if (size >= values.size) {
            val data = FloatArray(max(values.size * 2, 16))
            System.arraycopy(values, 0, data, 0, size)
            this.values = data
        }
        values[size++] = v
    }

    operator fun get(index: Int) = values[index]
}