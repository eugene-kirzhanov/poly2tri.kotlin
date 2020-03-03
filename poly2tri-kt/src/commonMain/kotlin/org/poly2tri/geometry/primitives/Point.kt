package org.poly2tri.geometry.primitives

abstract class Point {

    abstract val x: Double
    abstract val y: Double
    abstract val z: Double

    abstract fun set(x: Double, y: Double, z: Double)

}