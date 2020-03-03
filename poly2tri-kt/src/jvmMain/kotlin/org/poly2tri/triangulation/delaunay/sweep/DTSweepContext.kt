/* Poly2Tri
 * Copyright (c) 2009-2010, Poly2Tri Contributors
 * http://code.google.com/p/poly2tri/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Poly2Tri nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.poly2tri.triangulation.delaunay.sweep

import org.poly2tri.triangulation.delaunay.DelaunayTriangle
import java.util.*

/**
 * @author Thomas Еhlen, thahlen@gmail.com
 */
actual class DTSweepContext : DTSweepContextCommon() {

    actual override fun meshClean(triangle: DelaunayTriangle?) {
        var t1: DelaunayTriangle
        var t2: DelaunayTriangle?
        if (triangle != null) {
            val deque = ArrayDeque<DelaunayTriangle>()
            deque.addFirst(triangle)
            triangle.isInterior = true
            while (!deque.isEmpty()) {
                t1 = deque.removeFirst()
                triUnit.addTriangle(t1)
                for (i in 0..2) {
                    if (!t1.cEdge[i]) {
                        t2 = t1.neighbors[i]
                        if (t2 != null && !t2.isInterior) {
                            t2.isInterior = true
                            deque.addLast(t2)
                        }
                    }
                }
            }
        }
    }

}