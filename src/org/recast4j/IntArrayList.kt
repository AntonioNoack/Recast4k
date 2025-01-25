package org.recast4j

import kotlin.math.max

class IntArrayList(var values: IntArray) {

    var size = 0

    constructor(cap: Int = 16) : this(IntArray(cap))
    constructor(src: IntArrayList) : this(src.size) {
        System.arraycopy(src.values, 0, values, 0, src.size)
        size = src.size
    }

    fun add(v: Int) {
        ensureExtra(1)
        values[size++] = v
    }

    fun add(index: Int, value: Int) {
        add(value)
        System.arraycopy(values, index, values, index + 1, size - index)
        values[index] = value
    }

    operator fun get(index: Int) = values[index]
    operator fun set(index: Int, value: Int) {
        values[index] = value
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

    fun ensureExtra(extra: Int) {
        ensureCapacity(size + extra)
    }

    private fun ensureCapacity(size: Int) {
        if (values.size < size) {
            values = values.copyOf(max(values.size * 2, max(size, 16)))
        }
    }

    fun isEmpty() = size <= 0

    fun subList(startIndex: Int, endIndex: Int): IntArrayList {
        return IntArrayList(values.copyOfRange(startIndex, endIndex))
    }

    companion object {
        val empty = IntArrayList(0)
    }

}