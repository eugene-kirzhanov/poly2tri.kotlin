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

import org.poly2tri.triangulation.Triangulatable
import org.poly2tri.triangulation.TriangulationAlgorithm
import org.poly2tri.triangulation.TriangulationContext
import org.poly2tri.triangulation.TriangulationPoint
import org.poly2tri.triangulation.delaunay.DelaunayTriangle
import org.poly2tri.triangulation.point.TPoint

/**
 * @author Thomas Еhlen, thahlen@gmail.com
 */
abstract class DTSweepContextCommon : TriangulationContext() {

    companion object {
        // Inital triangle factor, seed triangle will extend 30% of PointSet width to both left and right.
        private const val ALPHA = 0.3f
    }

    lateinit var aFront: AdvancingFront

    private lateinit var head: TriangulationPoint
    private lateinit var tail: TriangulationPoint

    var basin = Basin()
    var edgeEvent = EdgeEvent()

    private val comparator = DTSweepPointComparator()

    init {
        clear()
    }

    fun removeFromList(triangle: DelaunayTriangle?) {
        triList.remove(triangle)
    }

    internal abstract fun meshClean(triangle: DelaunayTriangle?)

    final override fun clear() {
        super.clear()
        triList.clear()
    }

    fun locateNode(point: TriangulationPoint): AdvancingFrontNode? {
        return aFront.locateNode(point)
    }

    fun createAdvancingFront() {
        // Initial triangle
        val iTriangle = DelaunayTriangle(points[0], this.tail, this.head)
        addToList(iTriangle)

        val head = AdvancingFrontNode(this.tail, iTriangle)
        val middle = AdvancingFrontNode(points[0], iTriangle)
        val tail = AdvancingFrontNode(this.head, null)

        aFront = AdvancingFront(head, tail)

        aFront.head.next = middle
        middle.next = aFront.tail
        middle.prev = aFront.head
        aFront.tail.prev = middle
    }

    class Basin {
        var leftNode: AdvancingFrontNode? = null
        var bottomNode: AdvancingFrontNode? = null
        var rightNode: AdvancingFrontNode? = null
        var width = 0.0
        var leftHighest = false
    }

    class EdgeEvent {
        var constrainedEdge: DTSweepConstraint? = null
        var right = false
    }

    /**
     * Try to map a node to all sides of this triangle that don't have
     * a neighbor.
     */
    fun mapTriangleToNodes(t: DelaunayTriangle) {
        for (i in 0..2) {
            if (t.neighbors[i] == null) {
                aFront.locatePoint(t.pointCW(t.points[i]))?.let { node ->
                    node.triangle = t
                }
            }
        }
    }

    override fun prepareTriangulation(t: Triangulatable) {
        super.prepareTriangulation(t)
        var xmax: Double
        var xmin: Double
        var ymax: Double
        var ymin: Double
        xmin = points[0].x
        xmax = xmin
        ymin = points[0].y
        ymax = ymin
        // Calculate bounds. Should be combined with the sorting
        for (p in points) {
            if (p.x > xmax) xmax = p.x
            if (p.x < xmin) xmin = p.x
            if (p.y > ymax) ymax = p.y
            if (p.y < ymin) ymin = p.y
        }
        val deltaX = ALPHA * (xmax - xmin)
        val deltaY = ALPHA * (ymax - ymin)
        val p1 = TPoint(xmax + deltaX, ymin - deltaY)
        val p2 = TPoint(xmin - deltaX, ymin - deltaY)
        head = p1
        tail = p2
        points.sortWith(comparator)
    }

    fun finalizeTriangulation() {
        triUnit.addTriangles(triList)
        triList.clear()
    }

    override fun newConstraint(a: TriangulationPoint, b: TriangulationPoint) {
        DTSweepConstraint(a, b)
    }

    override fun algorithm(): TriangulationAlgorithm = TriangulationAlgorithm.DTSweep

}