package org.poly2tri.triangulation.point

import org.poly2tri.triangulation.TriangulationPoint

open class TPoint(override var x: Double,
                  override var y: Double,
                  override var z: Double = 0.0) : TriangulationPoint() {

    override fun set(x: Double, y: Double, z: Double) {
        this.x = x
        this.y = y
        this.z = z
    }

}