import org.poly2tri.Poly2Tri
import org.poly2tri.geometry.polygon.Polygon
import org.poly2tri.geometry.polygon.PolygonPoint

fun main() {
    val points = listOf(
            PolygonPoint(-371.182, 307.381),
            PolygonPoint(-365.909, 310.721),
            PolygonPoint(-369.425, 318.455),
            PolygonPoint(-373.468, 317.048),
            PolygonPoint(-375.401, 315.818),
            PolygonPoint(-376.28, 314.588),
            PolygonPoint(-376.28, 313.357),
            PolygonPoint(-375.753, 311.599),
            PolygonPoint(-373.643, 307.733),
            PolygonPoint(-372.764, 307.029),
            PolygonPoint(-372.061, 307.029)
    )
    val polygon = Polygon(points)
    Poly2Tri.triangulate(polygon)
    println("triangles: ${polygon.getTriangles().size}")
}
