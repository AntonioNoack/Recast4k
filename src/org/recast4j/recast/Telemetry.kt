/*
recast4j copyright (c) 2021 Piotr Piastucki piotr@jtilia.org

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

import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.atomic.AtomicLong

class Telemetry {
    private val timerStart = ThreadLocal.withInitial<MutableMap<String, AtomicLong>> { HashMap() }
    private val timerAccum: MutableMap<String, AtomicLong> = ConcurrentHashMap()
    fun startTimer(name: String) {
        timerStart.get()[name] = AtomicLong(System.nanoTime())
    }

    fun stopTimer(name: String) {
        timerAccum.computeIfAbsent(name) { AtomicLong() }
            .addAndGet(System.nanoTime() - timerStart.get()[name]!!.get())
    }

    fun warn(string: String?) {
        System.err.println(string)
    }

    fun print() {
        timerAccum.forEach { (n: String, v: AtomicLong) -> println(n + ": " + v.get() / 1000000) }
    }
}