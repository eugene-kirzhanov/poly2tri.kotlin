package org.poly2tri.triangulation.delaunay.sweep

import org.poly2tri.triangulation.TriangulationPoint

class DTSweepPointComparator : Comparator<TriangulationPoint> {

    override fun compare(a: TriangulationPoint, b: TriangulationPoint): Int {
        return when {
            a.y < b.y -> -1
            a.y > b.y -> 1
            else -> a.x.compareTo(b.x)
        }
    }

}