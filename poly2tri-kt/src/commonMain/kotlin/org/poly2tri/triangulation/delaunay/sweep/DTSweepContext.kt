package org.poly2tri.triangulation.delaunay.sweep

import org.poly2tri.triangulation.delaunay.DelaunayTriangle

expect class DTSweepContext() : DTSweepContextCommon {

    override fun meshClean(triangle: DelaunayTriangle?)

}