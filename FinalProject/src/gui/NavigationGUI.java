package gui;

import java.awt.Color;
import java.util.LinkedList;
import java.util.List;

import navigation.*;

import org.ros.node.NodeMain;

import GlobalNavigation.MapGUI;

public class NavigationGUI extends MapGUI implements NodeMain {

	public World world;
	public NavigationGUI(World world){
		this.world = world; //TODO overwrite ROBOT_SHAPE in VisionGUI to draw new robot
	}
	
	public void draw(Point point, Color color) {
		double radius = .025;
		draw(point, radius, true, color);
	}
	
	public void draw(BoundingBox box, boolean filled, Color color) {
		addRect(box.min.x, box.min.y, box.max.x - box.min.x, box.max.y - box.min.y, filled, color);
	}
	
	public void draw(Polygon polygon, boolean filled, Color color) {
		addPoly(Util.convertToPoint2D(polygon.points), true, filled, color);
	}
	
	public void draw(Point center, double radius, boolean filled, Color color) {
		int circleApproximation = 100;
		LinkedList<Point> points = new LinkedList<Point>();
		for (double angle = 0; angle < 2*Math.PI; angle += (2*Math.PI)/circleApproximation) {
			points.add(new Point(center.x + radius*Math.cos(angle), center.y + radius*Math.sin(angle)));
		}
		addPoly(Util.convertToPoint2D(points), true, filled, color);
	}
	
	public void draw(Fiducial fid){
		draw(fid.center, fid.radius, true, fid.color);
	}
	
	public void draw(Shape shape, boolean filled, Color color) {
		for (Polygon polygon : shape.polygons) {
			draw(polygon, filled, color);
		}		
	}
	
	public void draw(Grid grid, Color color) {
		for (BoundingBox box : grid.getAboveThreshold(0.0)) {
			draw(box, false, color);
		}
	}
	
	public void draw() { //TODO Draw order
		draw(world.getRegion(), false, Color.BLACK);
		
		for (Fiducial fid : world.getFiducials()) {
			draw(fid);
		}
		for (Polygon obstacle : world.getObstacles()) {
			draw(obstacle, true, Color.BLACK);
		}
		for (Point block : world.getBlocks()) {
			draw(block, Color.BLACK);
		}
		draw(world.getStart(), Color.BLUE);
		draw(world.getGoal(), Color.RED);
	}
	
	public void draw(List<Configuration> path, double grow, Color color) {
		if (path == null || path.size() < 1) return;
		for (int i = 0; i < path.size(); i ++) {
			//addSegment(path.get(i-1).x, path.get(i-1).y, path.get(i).x, path.get(i).y, color);
			draw(world.getRobot(path.get(i), grow), false, color);
		}
	}
	
	public void draw(TreeNode node, Color color) {
		for (TreeNode child : node.children) {
			addSegment(node.config.x, node.config.y, child.config.x, child.config.y, color);
			draw(child, color);
		}
	}	
	
	public void clear() {
		eraseLine();
		eraseSegments();
		erasePoints();
		eraseRects();
		erasePolys();
	}
	
}
