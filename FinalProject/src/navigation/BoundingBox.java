package navigation;

import java.util.LinkedList;
import java.util.List;

public class BoundingBox {
	public final Point min, max;

	public BoundingBox(Point min, Point max) {
		this.min = min;
		this.max = max;
	}

	public boolean collides(BoundingBox other) {
		return !(min.x > other.max.x || max.x < other.min.x
				|| min.y > other.max.y || max.y < other.min.y);
	}

	public boolean contains(Point point) {
		return min.x < point.x && max.x > point.x && min.y < point.y
				&& max.y > point.y;
	}
	
	public boolean contains(BoundingBox other) {
		return (min.x <= other.min.x && max.x >= other.max.x
				&& min.y <= other.min.y && max.y >= other.max.y);
	}
	
	public boolean contains(Polygon polygon) {
		if (!contains(polygon.boundingBox)) {
			return false;
		}
		
		for (Point point : polygon.points) {
			if (!contains(point)) {
				return false;
			}
		}
		return true;
	}
	
	public boolean contains(Shape shape) {
		if (!contains(shape.boundingBox)) {
			return false;
		}
		
		for (Polygon polygon : shape.polygons) {
			if (!contains(polygon)) {
				return false;
			}
		}
		return true;
	}
	
	public Polygon getPolygon() {
		List<Point> points = new LinkedList<Point>();
		points.add(min);
		points.add(new Point(max.x, min.y));
		points.add(max);
		points.add(new Point(min.x, max.y));
		return new Polygon(points);
	}

	public static BoundingBox pointsToBoundingBox(List<Point> points) {
		if (points.size() == 0) {
			throw new RuntimeException();
		}

		double minX, minY, maxX, maxY;
		minX = maxX = points.get(0).x;
		minY = maxY = points.get(0).y;

		for (int i = 1; i < points.size(); i++) {
			Point point = points.get(i);
			minX = Math.min(minX, point.x);
			minY = Math.min(minY, point.y);
			maxX = Math.max(maxX, point.x);
			maxY = Math.max(maxY, point.y);
		}
		return new BoundingBox(new Point(minX, minY), new Point(maxX, maxY));
	}
	
	public static BoundingBox polygonsToBoundingBox(List<Polygon> polygons) {
		if (polygons.size() == 0) {
			throw new RuntimeException();
		}

		double minX, minY, maxX, maxY;
		BoundingBox box = polygons.get(0).boundingBox;
		minX = box.min.x;
		maxX = box.max.x;
		minY = box.min.y;
		maxY = box.max.y;

		for (int i = 1; i < polygons.size(); i++) {
			box = polygons.get(i).boundingBox;
			minX = Math.min(minX, box.min.x);
			minY = Math.min(minY, box.min.y);
			maxX = Math.max(maxX, box.max.x);
			maxY = Math.max(maxY, box.max.y);
		}
		return new BoundingBox(new Point(minX, minY), new Point(maxX, maxY));
	}
}