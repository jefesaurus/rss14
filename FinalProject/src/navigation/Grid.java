package navigation;

import java.util.LinkedList;
import java.util.List;

public class Grid {
	public final BoundingBox box;
	public final double[][] matrix;
	public final int width;
	public final int height;

	//TODO - average observations over time
	
	public Grid(BoundingBox box) { //TODO cache the boxes
		this.box = box;
		this.width = (int)Math.ceil((box.max.x - box.min.x)/Constants.gridResolution);
		this.height = (int)Math.ceil((box.max.y - box.min.y)/Constants.gridResolution);
		matrix = new double[height][width];
		
		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height; y++) {
				setValue(x, y, 0.0);
			}
		}
	}
	
	public Double getValue(int x, int y) {
		return matrix[height - 1 - y][x];
	}
	
	public void setValue(int x, int y, double d) {
		matrix[height - 1 - y][x] = d;
	}
	
	public Double getValue(Point p) {
		if (!box.contains(p)) {
			return null;
		}
		return getValue(getX(p), getY(p));
	}
	
	public void setValue(Point p, double d) {
		if (!box.contains(p)) {
			return;
		}
		setValue(getX(p), getY(p), d);
	}
	
	public int getX(Point p) {		
		return (int)((p.x - box.min.x)/Constants.gridResolution);
	}
	
	public int getY(Point p) {
		return (int)((p.y - box.min.y)/Constants.gridResolution);
	}
	
	public boolean valid(int x, int y) {
		return x >= 0 && x < width && y >= 0 && y < height;
	}
	
	public BoundingBox getBox(int x, int y) {
		return new BoundingBox(new Point(box.min.x + x*Constants.gridResolution, box.min.y + y*Constants.gridResolution),
				new Point(Math.min(box.max.x, box.min.x + (x+1)*Constants.gridResolution), Math.min(box.max.y, box.min.y + (y+1)*Constants.gridResolution)));	
	}
	
	public List<BoundingBox> getAboveThreshold(double threshold) {
		List<BoundingBox> boxes = new LinkedList<BoundingBox>();
		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height; y++) {
				if (getValue(x, y) > threshold) {
					boxes.add(getBox(x, y));
				}
			}
		}		
		return boxes;
	}
	
	public List<BoundingBox> getBelowThreshold(double threshold) {
		List<BoundingBox> boxes = new LinkedList<BoundingBox>();
		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height; y++) {
				if (getValue(x, y) <= threshold) {
					boxes.add(getBox(x, y));
				}
			}
		}		
		return boxes;
	}

	public void markColliding(Polygon poly) {
		for (int x = getX(poly.boundingBox.min); x <= getX(poly.boundingBox.max); x++) {
			for (int y = getY(poly.boundingBox.min); y <= getY(poly.boundingBox.max); y++) {
				if (valid(x, y) && poly.collides(getBox(x, y).getPolygon())) {
					setValue(x, y, getValue(x, y) + 1);
				}
			}
		}
		
	}
	
	public boolean collides(Polygon poly, double threshold) {
		for (int x = getX(poly.boundingBox.min); x <= getX(poly.boundingBox.max); x++) {
			for (int y = getY(poly.boundingBox.min); y <= getY(poly.boundingBox.max); y++) {
				if (valid(x, y) && getValue(x, y) > threshold && poly.collides(getBox(x, y).getPolygon())) {
					return true;
				}
			}
		}
		return false;
	}
	
	public boolean collides(Shape shape, double threshold) {
		for (int x = getX(shape.boundingBox.min); x <= getX(shape.boundingBox.max); x++) {
			for (int y = getY(shape.boundingBox.min); y <= getY(shape.boundingBox.max); y++) {
				if (valid(x, y) && getValue(x, y) > threshold && shape.collides(getBox(x, y).getPolygon())) {
					return true;
				}
			}
		}
		return false;
	}
}