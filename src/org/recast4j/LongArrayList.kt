package org.recast4j

import kotlin.math.max
import kotlin.math.min

class LongArrayList(cap: Int = 16) {

    var values = LongArray(cap)
    var size = 0

    constructor(src: LongArrayList) : this(src.size) {
        System.arraycopy(src.values, 0, values, 0, src.size)
        size = src.size
    }

    fun add(v: Long) {
        if (size + 1 >= values.size) {
            val data = LongArray(max(values.size * 2, 16))
            System.arraycopy(values, 0, data, 0, size)
            this.values = data
        }
        values[size++] = v
    }

    fun get(index: Int) = values[index]
    fun reverse() {
        var j = size - 1
        val values = values
        for (i in 0 until size / 2) {
            val t = values[i]
            values[i] = values[j]
            values[j] = t
            j--
        }
    }

    fun addAll(list: LongArrayList) {
        if (size + list.size >= values.size) {
            val data = LongArray(max(values.size + max(values.size, list.size), 16))
            System.arraycopy(values, 0, data, 0, size)
            this.values = data
        }
        System.arraycopy(list.values, 0, values, size, list.size)
        size += list.size
    }

    fun addAll(list: LongArrayList, startIndex: Int, endIndex: Int) {
        val listSize = endIndex - startIndex
        if (size + listSize >= values.size) {
            val data = LongArray(max(values.size + max(values.size, listSize), 16))
            System.arraycopy(values, 0, data, 0, size)
            this.values = data
        }
        System.arraycopy(list.values, startIndex, values, size, listSize)
        size += listSize
    }

    fun remove(index: Int): Long {
        val oldValue = values[index]
        System.arraycopy(values, index + 1, values, index, size - index - 1)
        size--
        return oldValue
    }

    fun remove(value: Long) {
        val i = indexOf(value)
        if (i >= 0) remove(i)
    }

    fun indexOf(value: Long): Int {
        for (i in 0 until size) {
            if (values[i] == value) return i
        }
        return -1
    }

    fun contains(value: Long): Boolean {
        return indexOf(value) >= 0
    }

    fun clear() {
        size = 0
    }

    fun isEmpty() = size <= 0

    fun shrink(newSize: Int) {
        size = min(size,newSize)
    }

    fun subList(startIndex: Int, endIndex: Int): LongArrayList {
        val child = LongArrayList(endIndex - startIndex)
        System.arraycopy(values, startIndex, child.values, 0, endIndex - startIndex)
        child.size = size
        return child
    }

    companion object {
        val empty = LongArrayList(0)
    }

}