package org.recast4j

import kotlin.math.max
import kotlin.math.min

class IntArrayList(cap: Int = 16) {

    var values = IntArray(cap)
    var size = 0

    constructor(src: IntArrayList) : this(src.size) {
        System.arraycopy(src.values, 0, values, 0, src.size)
        size = src.size
    }

    fun add(v: Int) {
        if (size + 1 >= values.size) {
            val data = IntArray(max(values.size * 2, 16))
            System.arraycopy(values, 0, data, 0, size)
            this.values = data
        }
        values[size++] = v
    }

    fun add(index: Int, value: Int) {
        add(value)
        System.arraycopy(values, index , values, index + 1, size - index)
        values[index] = value
    }

    operator fun get(index: Int) = values[index]
    operator fun set(index: Int, value: Int) {
        values[index] = value
    }

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

    fun addAll(list: IntArrayList) {
        if (size + list.size >= values.size) {
            val data = IntArray(max(values.size + max(values.size, list.size), 16))
            System.arraycopy(values, 0, data, 0, size)
            this.values = data
        }
        System.arraycopy(list.values, 0, values, size, list.size)
        size += list.size
    }

    fun addAll(list: IntArrayList, startIndex: Int, endIndex: Int) {
        val listSize = endIndex - startIndex
        if (size + listSize >= values.size) {
            val data = IntArray(max(values.size + max(values.size, listSize), 16))
            System.arraycopy(values, 0, data, 0, size)
            this.values = data
        }
        System.arraycopy(list.values, startIndex, values, size, listSize)
        size += listSize
    }

    fun remove(index: Int): Int {
        val oldValue = values[index]
        System.arraycopy(values, index + 1, values, index, size - index - 1)
        size--
        return oldValue
    }

    @Suppress("MemberVisibilityCanBePrivate")
    fun indexOf(value: Int): Int {
        for (i in 0 until size) {
            if (values[i] == value) return i
        }
        return -1
    }

    fun contains(value: Int): Boolean {
        return indexOf(value) >= 0
    }

    fun clear() {
        size = 0
    }

    fun isEmpty() = size <= 0

    fun shrink(newSize: Int) {
        size = min(size, newSize)
    }

    fun subList(startIndex: Int, endIndex: Int): IntArrayList {
        val child = IntArrayList(endIndex - startIndex)
        System.arraycopy(values, startIndex, child.values, 0, endIndex - startIndex)
        child.size = endIndex - startIndex
        return child
    }

    companion object {
        val empty = IntArrayList(0)
    }

}