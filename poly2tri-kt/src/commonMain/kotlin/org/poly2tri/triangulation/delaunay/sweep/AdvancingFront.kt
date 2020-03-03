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

import org.poly2tri.triangulation.TriangulationPoint

/**
 * @author Thomas Еhlen (thahlen@gmail.com)
 */
class AdvancingFront(var head: AdvancingFrontNode, var tail: AdvancingFrontNode) {

    private var search: AdvancingFrontNode? = head

    override fun toString(): String {
        val sb = StringBuilder()
        var node: AdvancingFrontNode? = head
        while (node != tail && node != null) {
            sb.append(node.point.x.toString()).append("->")
            node = node.next
        }
        sb.append(tail.point.x)
        return sb.toString()
    }

    private fun findSearchNode(): AdvancingFrontNode? {
        return search
    }

    /**
     * We use a balancing tree to locate a node smaller or equal to
     * given key value
     */
    fun locateNode(point: TriangulationPoint): AdvancingFrontNode? {
        return locateNode(point.x)
    }

    private fun locateNode(x: Double): AdvancingFrontNode? {
        var node = findSearchNode()
        if (node != null) {
            if (x < node.value) {
                node = node.prev
                while (node != null) {
                    if (x >= node.value) {
                        search = node
                        return node
                    }
                    node = node.prev
                }
            } else {
                node = node.next
                while (node != null) {
                    if (x < node.value) {
                        search = node.prev
                        return node.prev
                    }
                    node = node.next
                }
            }
        }
        return null
    }

    /**
     * This implementation will use simple node traversal algorithm to find a point on the front
     */
    fun locatePoint(point: TriangulationPoint?): AdvancingFrontNode? {
        point ?: return null
        val px = point.x
        var node = findSearchNode()
        val nx = node!!.point.x
        if (px == nx) {
            if (point !== node.point) { // We might have two nodes with same x value for a short time
                node = when {
                    point === node.prev!!.point -> node.prev
                    point === node.next!!.point -> node.next
                    else -> throw RuntimeException("Failed to find Node for given afront point")
                }
            }
        } else if (px < nx) {
            while (node!!.prev.also { node = it } != null) {
                if (point === node!!.point) {
                    break
                }
            }
        } else {
            while (node!!.next.also { node = it } != null) {
                if (point === node!!.point) {
                    break
                }
            }
        }
        search = node
        return node
    }

}