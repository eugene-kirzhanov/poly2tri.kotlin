package org.poly2tri.geometry.polygon

import org.poly2tri.triangulation.Triangulatable
import org.poly2tri.triangulation.TriangulationContext
import org.poly2tri.triangulation.TriangulationMode
import org.poly2tri.triangulation.TriangulationPoint
import org.poly2tri.triangulation.TriangulationPoint.Companion.mergeInstances
import org.poly2tri.triangulation.delaunay.DelaunayTriangle

class Polygon : Triangulatable {

    private val points: MutableList<TriangulationPoint> = ArrayList()
    private val steinerPoints: MutableList<TriangulationPoint> = ArrayList()

    private val holes: MutableList<Polygon> = ArrayList()
    private val triangles: MutableList<DelaunayTriangle> = ArrayList()

    private var point: PolygonPoint? = null

    /**
     * To create a polygon we need atleast 3 separate points
     */
    constructor(p1: PolygonPoint, p2: PolygonPoint, p3: PolygonPoint) {
        p1.next = p2
        p2.next = p3
        p3.next = p1
        p1.previous = p3
        p2.previous = p1
        p3.previous = p2
        points.add(p1)
        points.add(p2)
        points.add(p3)
    }

    /**
     * Requires atleast 3 points
     *
     * @param points - ordered list of points forming the polygon. No duplicates are allowed
     */
    constructor(points: List<PolygonPoint>) {
        // Lets do one sanity check that first and last point hasn't got same position
        // Its something that often happen when importing polygon data from other formats
        val list = if (points[0] == points[points.size - 1]) {
            points.toMutableList().apply {
                removeAt(points.size - 1)
            }
        } else {
            points
        }
        this.points.addAll(list)
    }

    /**
     * Requires atleast 3 points
     */
    constructor(points: Array<PolygonPoint>) : this(points.toMutableList())

    override fun getTriangulationMode(): TriangulationMode {
        return TriangulationMode.POLYGON
    }

    private fun pointCount(): Int = points.size + steinerPoints.size

    fun addSteinerPoint(point: TriangulationPoint) {
        steinerPoints.add(point)
    }

    fun addSteinerPoints(points: List<TriangulationPoint>) {
        steinerPoints.addAll(points)
    }

    fun clearSteinerPoints() {
        steinerPoints.clear()
    }

    /**
     * Assumes: that given polygon is fully inside the current polygon
     *
     * @param poly - a subtraction polygon
     */
    fun addHole(poly: Polygon) {
        holes.add(poly)
    }

    /**
     * Will insert a point in the polygon after given point
     */
    fun insertPointAfter(a: PolygonPoint, newPoint: PolygonPoint) { // Validate that
        val index = points.indexOf(a)
        if (index != -1) {
            newPoint.next = a.next
            newPoint.previous = a
            a.next?.previous = newPoint
            a.next = newPoint
            points.add(index + 1, newPoint)
        } else {
            throw RuntimeException("Tried to insert a point into a Polygon after a point not belonging to the Polygon")
        }
    }

    fun addPoints(list: List<PolygonPoint>) {
        for (p in list) {
            p.previous = point
            if (point != null) {
                p.next = point?.next
                point?.next = p
            }
            point = p
            points.add(p)
        }
        val first = points[0] as PolygonPoint
        point?.next = first
        first.previous = point
    }

    /**
     * Will add a point after the last point added
     */
    fun addPoint(p: PolygonPoint) {
        p.previous = point
        p.next = point?.next
        point?.next = p
        points.add(p)
    }

    fun removePoint(p: PolygonPoint) {
        val next: PolygonPoint? = p.next
        val prev: PolygonPoint? = p.previous
        prev?.next = next
        next?.previous = prev
        points.remove(p)
    }

    override fun getTriangles(): List<DelaunayTriangle> = triangles

    override fun addTriangle(t: DelaunayTriangle) {
        triangles.add(t)
    }

    override fun addTriangles(list: List<DelaunayTriangle>) {
        triangles.addAll(list)
    }

    fun clearTriangulation() {
        triangles.clear()
    }

    /**
     * Merge equals points.
     * Creates constraints and populates the context with points.
     */
    override fun prepareTriangulation(tcx: TriangulationContext) {
        var hint = points.size + steinerPoints.size
        holes.forEach { hint += it.pointCount() }

        val uniquePts = HashMap<TriangulationPoint, TriangulationPoint>(hint)
        mergeInstances(uniquePts, points)
        if (steinerPoints.isNotEmpty()) {
            mergeInstances(uniquePts, steinerPoints)
        }
        holes.forEach {
            mergeInstances(uniquePts, it.points)
        }

        triangles.clear()

        // Outer constraints
        for (i in 0 until points.size - 1) {
            tcx.newConstraint(points[i], points[i + 1])
        }
        tcx.newConstraint(points[0], points[points.size - 1])

        // Hole constraints
        holes.forEach {
            for (i in 0 until it.points.size - 1) {
                tcx.newConstraint(it.points[i], it.points[i + 1])
            }
            tcx.newConstraint(it.points[0], it.points[it.points.size - 1])
        }
        tcx.addPoints(uniquePts.keys)
    }

}