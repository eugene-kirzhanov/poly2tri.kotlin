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
package org.poly2tri.triangulation.sets

import org.poly2tri.triangulation.TriangulationContext
import org.poly2tri.triangulation.TriangulationMode
import org.poly2tri.triangulation.TriangulationPoint

/**
 * Extends the PointSet by adding some Constraints on how it will be triangulated<br></br>
 * A constraint defines an edge between two points in the set, these edges can not
 * be crossed. They will be enforced triangle edges after a triangulation.
 *
 * @author Thomas Еhlen, thahlen@gmail.com
 */
class ConstrainedPointSet : PointSet {

    private var index: IntArray? = null
    private var constrainedPointList: MutableList<TriangulationPoint>? = null

    constructor(points: List<TriangulationPoint>, index: IntArray) : super(points) {
        this.index = index
    }

    /**
     * @param points      - A list of all points in PointSet
     * @param constraints - Pairs of two points defining a constraint, all points **must** be part of given PointSet!
     */
    constructor(points: List<TriangulationPoint>, constraints: List<TriangulationPoint>) : super(points) {
        constrainedPointList = ArrayList(constraints)
    }

    override fun getTriangulationMode(): TriangulationMode = TriangulationMode.CONSTRAINED

    override fun prepareTriangulation(tcx: TriangulationContext) {
        super.prepareTriangulation(tcx)
        constrainedPointList?.let {
            val uniquePts = HashMap<TriangulationPoint, TriangulationPoint>(points.size)
            // Enforce same coordinates means same instance of TriangulationPoint
            TriangulationPoint.mergeInstances(uniquePts, points)
            TriangulationPoint.mergeInstances(uniquePts, it)
            var p1: TriangulationPoint
            var p2: TriangulationPoint
            val iterator: Iterator<TriangulationPoint> = it.iterator()
            while (iterator.hasNext()) {
                p1 = iterator.next()
                p2 = iterator.next()
                tcx.newConstraint(p1, p2)
            }
        } ?: run {
            index?.let {
                var i = 0
                while (i < it.size) {
                    tcx.newConstraint(points[it[i]], points[it[i + 1]])
                    i += 2
                }
            }
        }
    }

}