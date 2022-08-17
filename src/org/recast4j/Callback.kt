package org.recast4j

interface Callback<V> {
    fun call(v: V)
}