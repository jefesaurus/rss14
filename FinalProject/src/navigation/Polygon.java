package navigation;

import java.awt.geom.Line2D;
import java.util.LinkedList;
import java.util.List;

public class Polygon {
	public final List<Point> points;
	public final BoundingBox boundingBox;
	private List<Line2D.Double> lines;

	public Polygon(List<Point> points) { // Assumes vertices in increasing angle order for a convex polygon
		this.points = points;
		this.boundingBox = BoundingBox.pointsToBoundingBox(points);
		this.lines = null; //TODO - lazily compute more things
	}
	
	public List<Line2D.Double> getLines() {
		if (lines == null) {
			lines = Util.getLines(points);
		}
		return lines;
	}
	
	public Polygon translate(double x, double y) {
		List<Point> translatedPoints = new LinkedList<Point>();
		for (Point untranslated : points) {
			translatedPoints.add(new Point(untranslated.x + x, untranslated.y + y));
		}
		return new Polygon(translatedPoints);
	}
	
	public Polygon rotate(double theta, Point reference) {
		List<Point> rotatedPoints = new LinkedList<Point>();
		double cosTheta = Math.cos(theta);
		double sinTheta = Math.sin(theta);

		for (Point unrotated : points) {
			rotatedPoints.add(new Point(
					cosTheta*(unrotated.x - reference.x) - sinTheta*(unrotated.y - reference.y) + reference.x,
					sinTheta*(unrotated.x - reference.x) + cosTheta*(unrotated.y - reference.y) + reference.y));
		}
		return new Polygon(rotatedPoints);
	}
	
	public Polygon pose(Configuration c) { //Assumes currently at <0, 0, 0>
		List<Point> transformedPoints = new LinkedList<Point>();
		double cosTheta = Math.cos(c.theta);
		double sinTheta = Math.sin(c.theta);

		for (Point untransformed : points) {
			transformedPoints.add(new Point(
					cosTheta*(untransformed.x) - sinTheta*(untransformed.y) + c.x,
					sinTheta*(untransformed.x) + cosTheta*(untransformed.y) + c.y));
		}
		return new Polygon(transformedPoints);
	}

	public boolean collides(Polygon other) {
		if (!boundingBox.collides(other.boundingBox)) {
			return false;
		} else if (contains(other.points.get(0))
				|| other.contains(points.get(0))) {
			return true;
		}

		for (Line2D.Double line1 : getLines()) {
			for (Line2D.Double line2 : other.getLines()) {
				if (line1.intersectsLine(line2)) {
					return true;
				}
			}
		}
		return false;
	}
	
	public boolean collides(Shape other) {
		if (!boundingBox.collides(other.boundingBox)) {
			return false;
		} else if (contains(other.polygons.get(0).points.get(0))
				|| other.contains(points.get(0))) {
			return true;
		}

		for (Polygon polygon : other.polygons) {
			if (collides(polygon)) {
				return true;
			}
		}
		return false;
	}

	public boolean contains(Point point) {
		if (!boundingBox.contains(point)) {
			return false;
		}
		
		Point previous = points.get(0);
		for (int i = 1; i < points.size(); i++) {
			Point current = points.get(i);
			if (Util.rightTurn(previous, current, point)) {
				return false;
			}
			previous = current;
		}
		return !Util.rightTurn(previous, points.get(0), point);
	}
	
	public boolean contains(Polygon other) {
		if (!boundingBox.contains(other.boundingBox)) {
			return false;
		}
		
		for (Point point : other.points) {
			Point previous = points.get(0);
			for (int i = 1; i < points.size(); i++) {
				Point current = points.get(i);
				if (Util.rightTurn(previous, current, point)) {
					return false;
				}
				previous = current;
			}
		}
		return true;
	}
}