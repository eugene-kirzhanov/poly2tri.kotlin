package org.poly2tri.triangulation

import org.poly2tri.triangulation.delaunay.DelaunayTriangle

interface Triangulatable {

    /**
     * Preparations needed before triangulation start should be handled here
     */
    fun prepareTriangulation(tcx: TriangulationContext)

    fun getTriangles(): List<DelaunayTriangle>

    fun addTriangle(t: DelaunayTriangle)

    fun addTriangles(list: List<DelaunayTriangle>)

    fun getTriangulationMode(): TriangulationMode

}