package navigation;

import java.util.LinkedList;
import java.util.List;

public class Shape {

	public final List<Polygon> polygons;
	public final BoundingBox boundingBox;

	public Shape(List<Polygon> polygons) { 
		this.polygons = polygons;
		this.boundingBox = BoundingBox.polygonsToBoundingBox(polygons);
	}
	
	public Shape translate(double x, double y) {
		List<Polygon> translatedPolygons = new LinkedList<Polygon>();
		
		for (Polygon polygon : polygons) {
			translatedPolygons.add(polygon.translate(x, y));
		}
		return new Shape(translatedPolygons);
	}
	
	public Shape rotate(double theta, Point reference) {
		List<Polygon> rotatedPolygons = new LinkedList<Polygon>();
		
		for (Polygon polygon : polygons) {
			rotatedPolygons.add(polygon.rotate(theta, reference));
		}
		return new Shape(rotatedPolygons);
	}
	
	public Shape pose(Configuration c) { //Assumes currently at <0, 0, 0>
		List<Polygon> transformedPolygons = new LinkedList<Polygon>();
		
		for (Polygon polygon : polygons) {
			transformedPolygons.add(polygon.pose(c));
		}
		return new Shape(transformedPolygons);
	}

	public boolean collides(Polygon other) {
		if (!boundingBox.collides(other.boundingBox)) {
			return false;
		} else if (contains(other.points.get(0))
				|| other.contains(polygons.get(0).points.get(0))) {
			return true;
		}
		
		for (Polygon polygon : polygons) {
			if (polygon.collides(other)) {
				return true;
			}
		}
		return false;
	}
	
	public boolean collides(Shape other) {
		if (!boundingBox.collides(other.boundingBox)) {
			return false;
		} else if (contains(other.polygons.get(0).points.get(0))
				|| other.contains(polygons.get(0).points.get(0))) {
			return true;
		}
		
		for (Polygon poly1 : polygons) {
			for (Polygon poly2 : other.polygons) {
				if (poly1.collides(poly2)) {
					return true;
				}
			}
		}
		return false;
	}

	public boolean contains(Point point) {
		if (!boundingBox.contains(point)) {
			return false;
		}
		
		for (Polygon polygon : polygons) {
			if (polygon.contains(point)) {
				return true;
			}
		}
		return false;
	}
	
	//public boolean contains(Polygon polygon);
	//public boolean contains(Shape shape);
}
