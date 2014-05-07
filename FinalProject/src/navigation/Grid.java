package navigation;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.ConcurrentSkipListSet;

public class Grid {
	public final BoundingBox box;
	public final GridCell[][] matrix;
	public final int width;
	public final int height;
	public final ConcurrentSkipListSet<GridCell> occupied;
	
	public static final int MAX_COLLISIONS = 5;
	public static final long REFRESH_TIME = 10*1000;
	
	public Grid(BoundingBox box) { //TODO cache the boxes
		this.box = box;
		this.width = (int)Math.ceil((box.max.x - box.min.x)/Constants.GRID_RESOLUTION);
		this.height = (int)Math.ceil((box.max.y - box.min.y)/Constants.GRID_RESOLUTION);
		matrix = new GridCell[height][width];
		
		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height; y++) {
				int index = y*width + x;
				Point point = new Point(((box.min.x + x*Constants.GRID_RESOLUTION) + (Math.min(box.max.x, box.min.x + (x+1)*Constants.GRID_RESOLUTION)))/2., 
						((box.min.y + y*Constants.GRID_RESOLUTION) + (Math.min(box.max.y, box.min.y + (y+1)*Constants.GRID_RESOLUTION)))/2.);
				matrix[height - 1 - y][x] = new GridCell(x, y, index, point);
			}
		}
		
		//this.occupied = new HashSet<GridCell>();
		this.occupied = new ConcurrentSkipListSet<GridCell>();
		//this.occupied = new ConcurrentHashSet<GridCell>();
	}
	
	public GridCell cell(int x, int y) {
		return matrix[height - 1 - y][x];
	}
	
	public GridCell cell(Point p) {
		int x = getX(p);
		int y = getY(p);
		if (!valid(x, y)) {
			return null;
		}
		return cell(x, y);
	}
	
	public int getX(Point p) {		
		return (int)((p.x - box.min.x)/Constants.GRID_RESOLUTION);
	}
	
	public int getY(Point p) {
		return (int)((p.y - box.min.y)/Constants.GRID_RESOLUTION);
	}
	
	public boolean valid(int x, int y) {
		return x >= 0 && x < width && y >= 0 && y < height;
	}
	
	/*public List<Point> getColliding() {
		List<Point> points = new LinkedList<Point>();
		
		//for (GridCell cell : occupied) {
		//	points.add(cell.point);
		//
		//	if (System.currentTimeMillis() - cell.lastUpdate > 5*1000) {
		//		points.add(cell.point);
		//	} else {
		//		
		//	}
		//}
		
		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height; y++) {
				if (occupied(cell(x, y))) {
					points.add(cell(x, y).point);
				}
			}
		}
				
		
		return points;
	}*/	
	
	public void markColliding(Point point) {
		GridCell cell = cell(point);
		if (cell != null) {
			cell.incrementValue();
			occupied.add(cell);
		}
	}

	public void markColliding(Polygon poly) {
		for (int x = getX(poly.boundingBox.min); x <= getX(poly.boundingBox.max); x++) {
			for (int y = getY(poly.boundingBox.min); y <= getY(poly.boundingBox.max); y++) {
				if (valid(x, y) && poly.contains(cell(x, y).point)) {
					cell(x, y).incrementValue();
					occupied.add(cell(x, y));
				}
			}
		}
	}
	
	public void markColliding(Shape shape) {
		for (Polygon poly : shape.polygons) {
			markColliding(poly);
		}
	}
	
	public boolean occupied(GridCell cell) {
		if (cell.getValue() > 0.0) {
			if (System.currentTimeMillis() - cell.getLastUpdate() < REFRESH_TIME) {
				return true;
			} else {
				cell.resetValue();
				occupied.remove(cell);
			}
		} 
		return false;
	}
	
	public boolean collides(Polygon poly) {	
		int collisions = 0;
		for (int x = getX(poly.boundingBox.min); x <= getX(poly.boundingBox.max); x++) {
			for (int y = getY(poly.boundingBox.min); y <= getY(poly.boundingBox.max); y++) {
				if (!valid(x, y) || (occupied(cell(x, y)) && poly.contains(cell(x, y).point))) {
					collisions ++;
					if (collisions > MAX_COLLISIONS) {
						return true;
					}
				}
			}
		}
		return false;
	}
	
	public boolean collides(Shape shape) {
		int collisions = 0;
		for (int x = getX(shape.boundingBox.min); x <= getX(shape.boundingBox.max); x++) {
			for (int y = getY(shape.boundingBox.min); y <= getY(shape.boundingBox.max); y++) {
				if (!valid(x, y) || (occupied(cell(x, y)) && shape.contains(cell(x, y).point))) {
					collisions ++;
					if (collisions > MAX_COLLISIONS) {
						return true;
					}
				}
			}
		}
		return false;
	}
}