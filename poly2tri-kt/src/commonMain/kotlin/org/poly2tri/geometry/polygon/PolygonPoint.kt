package org.poly2tri.geometry.polygon

import org.poly2tri.triangulation.point.TPoint

class PolygonPoint : TPoint {

    var next: PolygonPoint? = null
    var previous: PolygonPoint? = null

    constructor(x: Double, y: Double) : super(x, y)
    constructor(x: Double, y: Double, z: Double) : super(x, y, z)

}