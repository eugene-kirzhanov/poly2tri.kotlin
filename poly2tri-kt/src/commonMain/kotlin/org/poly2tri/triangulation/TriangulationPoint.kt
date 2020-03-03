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
package org.poly2tri.triangulation

import org.poly2tri.geometry.primitives.Point
import org.poly2tri.triangulation.delaunay.sweep.DTSweepConstraint

abstract class TriangulationPoint : Point() {

    // List of edges this point constitutes an upper ending point (CDT)
    val edges = ArrayList<DTSweepConstraint>()

    fun addEdge(e: DTSweepConstraint) {
        edges.add(e)
    }

    override fun toString(): String = "[$x,$y]"

    override fun equals(other: Any?): Boolean {
        if (other is TriangulationPoint) {
            return x == other.x && y == other.y
        }
        return super.equals(other)
    }

    override fun hashCode(): Int {
        var result = x.hashCode()
        result = 31 * result + y.hashCode()
        return result
    }

    companion object {
        /**
         * Replace points in ptList for all equals object in uniquePts.
         *
         * @param uniquePts Map of triangulation points
         * @param ptList    Point list, updated, but always the same size.
         */
        fun mergeInstances(uniquePts: MutableMap<TriangulationPoint, TriangulationPoint>, ptList: MutableList<TriangulationPoint>) {
            for (idPoint in ptList.indices) {
                val pt = ptList[idPoint]
                val uniquePt = uniquePts[pt]
                if (uniquePt == null) {
                    uniquePts[pt] = pt
                } else {
                    // Duplicate point
                    ptList[idPoint] = uniquePt
                }
            }
        }
    }
}