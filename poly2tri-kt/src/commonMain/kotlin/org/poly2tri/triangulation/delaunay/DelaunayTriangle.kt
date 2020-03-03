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
package org.poly2tri.triangulation.delaunay

import org.poly2tri.triangulation.TriangulationPoint

class DelaunayTriangle(p1: TriangulationPoint, p2: TriangulationPoint, p3: TriangulationPoint) {

    /**
     * Neighbor pointers
     */
    val neighbors = arrayOfNulls<DelaunayTriangle>(3)

    /**
     * Flags to determine if an edge is a Constrained edge
     */
    val cEdge = booleanArrayOf(false, false, false)

    /**
     * Flags to determine if an edge is a Delauney edge
     */
    val dEdge = booleanArrayOf(false, false, false)

    /**
     * Has this triangle been marked as an interior triangle?
     */
    var isInterior = false

    val points = arrayOf(p1, p2, p3)

    fun index(p: TriangulationPoint): Int {
        return when {
            p === points[0] -> 0
            p === points[1] -> 1
            p === points[2] -> 2
            else -> throw RuntimeException("Calling index with a point that doesn't exist in triangle")
        }
    }

    fun contains(p: TriangulationPoint?): Boolean {
        return p === points[0] || p === points[1] || p === points[2]
    }

    fun contains(p: TriangulationPoint?, q: TriangulationPoint?): Boolean {
        return contains(p) && contains(q)
    }

    // Update neighbor pointers
    private fun markNeighbor(p1: TriangulationPoint?, p2: TriangulationPoint?, t: DelaunayTriangle) {
        if (p1 === points[2] && p2 === points[1] || p1 === points[1] && p2 === points[2]) {
            neighbors[0] = t
        } else if (p1 === points[0] && p2 === points[2] || p1 === points[2] && p2 === points[0]) {
            neighbors[1] = t
        } else if (p1 === points[0] && p2 === points[1] || p1 === points[1] && p2 === points[0]) {
            neighbors[2] = t
        }
    }

    /* Exhaustive search to update neighbor pointers */
    fun markNeighbor(t: DelaunayTriangle) {
        when {
            t.contains(points[1], points[2]) -> {
                neighbors[0] = t
                t.markNeighbor(points[1], points[2], this)
            }
            t.contains(points[0], points[2]) -> {
                neighbors[1] = t
                t.markNeighbor(points[0], points[2], this)
            }
            t.contains(points[0], points[1]) -> {
                neighbors[2] = t
                t.markNeighbor(points[0], points[1], this)
            }
            else -> println("Error: markNeighbor failed")
        }
    }

    fun clearNeighbors() {
        neighbors[0] = null
        neighbors[1] = null
        neighbors[2] = null
    }

    private fun clearNeighbor(triangle: DelaunayTriangle) {
        when {
            neighbors[0] === triangle -> neighbors[0] = null
            neighbors[1] === triangle -> neighbors[1] = null
            else -> neighbors[2] = null
        }
    }

    /**
     * Clears all references to all other triangles and points
     */
    fun clear() {
        neighbors.forEach { it?.clearNeighbor(this) }
        clearNeighbors()
    }

    /**
     * @param t - opposite triangle
     * @param p - the point in t that isn't shared between the triangles
     */
    fun oppositePoint(t: DelaunayTriangle, p: TriangulationPoint): TriangulationPoint {
        if (t === this) throw AssertionError("Self-pointer error")
        return pointCW(t.pointCW(p))
    }

    // The neighbor clockwise to given point
    fun neighborCW(point: TriangulationPoint): DelaunayTriangle? {
        return when {
            point === points[0] -> neighbors[1]
            point === points[1] -> neighbors[2]
            else -> neighbors[0]
        }
    }

    // The neighbor counter-clockwise to given point
    fun neighborCCW(point: TriangulationPoint): DelaunayTriangle? {
        return when {
            point === points[0] -> neighbors[2]
            point === points[1] -> neighbors[0]
            else -> neighbors[1]
        }
    }

    // The neighbor across to given point
    fun neighborAcross(opoint: TriangulationPoint): DelaunayTriangle? {
        return when {
            opoint === points[0] -> neighbors[0]
            opoint === points[1] -> neighbors[1]
            else -> neighbors[2]
        }
    }

    // The point counter-clockwise to given point
    fun pointCCW(point: TriangulationPoint): TriangulationPoint {
        return when {
            point === points[0] -> points[1]
            point === points[1] -> points[2]
            point === points[2] -> points[0]
            else -> {
                println("Error: point location error")
                throw RuntimeException("[FIXME] point location error")
            }
        }
    }

    // The point counter-clockwise to given point
    fun pointCW(point: TriangulationPoint): TriangulationPoint {
        return when {
            point === points[0] -> points[2]
            point === points[1] -> points[0]
            point === points[2] -> points[1]
            else -> {
                println("Error: point location error")
                throw RuntimeException("[FIXME] point location error")
            }
        }
    }

    // Legalize triangle by rotating clockwise around oPoint
    fun legalize(oPoint: TriangulationPoint, nPoint: TriangulationPoint) {
        when {
            oPoint === points[0] -> {
                points[1] = points[0]
                points[0] = points[2]
                points[2] = nPoint
            }
            oPoint === points[1] -> {
                points[2] = points[1]
                points[1] = points[0]
                points[0] = nPoint
            }
            oPoint === points[2] -> {
                points[0] = points[2]
                points[2] = points[1]
                points[1] = nPoint
            }
            else -> {
                println("Error: legalization error")
                throw RuntimeException("legalization bug")
            }
        }
    }

    override fun toString(): String = "${points[0]},${points[1]},${points[2]}"

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is DelaunayTriangle) return false
        return points.contentEquals(other.points)
    }

    override fun hashCode(): Int = points.contentHashCode()

    fun markConstrainedEdge(index: Int) {
        cEdge[index] = true
    }

    // Mark edge as constrained
    fun markConstrainedEdge(p: TriangulationPoint, q: TriangulationPoint) {
        if (q === points[0] && p === points[1] || q === points[1] && p === points[0]) {
            cEdge[2] = true
        } else if (q === points[0] && p === points[2] || q === points[2] && p === points[0]) {
            cEdge[1] = true
        } else if (q === points[1] && p === points[2] || q === points[2] && p === points[1]) {
            cEdge[0] = true
        }
    }

    /**
     * Get the neighbor that share this edge
     *
     * @return index of the shared edge or -1 if edge isn't shared
     */
    fun edgeIndex(p1: TriangulationPoint, p2: TriangulationPoint): Int {
        if (points[0] === p1) {
            if (points[1] === p2) {
                return 2
            } else if (points[2] === p2) {
                return 1
            }
        } else if (points[1] === p1) {
            if (points[2] === p2) {
                return 0
            } else if (points[0] === p2) {
                return 2
            }
        } else if (points[2] === p1) {
            if (points[0] === p2) {
                return 1
            } else if (points[1] === p2) {
                return 0
            }
        }
        return -1
    }

    fun getConstrainedEdgeCCW(p: TriangulationPoint): Boolean {
        return when {
            p === points[0] -> cEdge[2]
            p === points[1] -> cEdge[0]
            else -> cEdge[1]
        }
    }

    fun getConstrainedEdgeCW(p: TriangulationPoint): Boolean {
        return when {
            p === points[0] -> cEdge[1]
            p === points[1] -> cEdge[2]
            else -> cEdge[0]
        }
    }

    fun getConstrainedEdgeAcross(p: TriangulationPoint): Boolean {
        return when {
            p === points[0] -> cEdge[0]
            p === points[1] -> cEdge[1]
            else -> cEdge[2]
        }
    }

    fun setConstrainedEdgeCCW(p: TriangulationPoint, ce: Boolean) {
        when {
            p === points[0] -> cEdge[2] = ce
            p === points[1] -> cEdge[0] = ce
            else -> cEdge[1] = ce
        }
    }

    fun setConstrainedEdgeCW(p: TriangulationPoint, ce: Boolean) {
        when {
            p === points[0] -> cEdge[1] = ce
            p === points[1] -> cEdge[2] = ce
            else -> cEdge[0] = ce
        }
    }

    fun getDelunayEdgeCCW(p: TriangulationPoint): Boolean {
        return when {
            p === points[0] -> dEdge[2]
            p === points[1] -> dEdge[0]
            else -> dEdge[1]
        }
    }

    fun getDelunayEdgeCW(p: TriangulationPoint): Boolean {
        return when {
            p === points[0] -> dEdge[1]
            p === points[1] -> dEdge[2]
            else -> dEdge[0]
        }
    }

    fun setDelunayEdgeCCW(p: TriangulationPoint, e: Boolean) {
        when {
            p === points[0] -> dEdge[2] = e
            p === points[1] -> dEdge[0] = e
            else -> dEdge[1] = e
        }
    }

    fun setDelunayEdgeCW(p: TriangulationPoint, e: Boolean) {
        when {
            p === points[0] -> dEdge[1] = e
            p === points[1] -> dEdge[2] = e
            else -> dEdge[0] = e
        }
    }

    fun clearDelunayEdges() {
        dEdge[0] = false
        dEdge[1] = false
        dEdge[2] = false
    }

}