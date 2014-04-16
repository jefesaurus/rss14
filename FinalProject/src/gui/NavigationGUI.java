package gui;

import java.awt.Color;
import java.util.List;

import navigation.*;

import org.ros.node.NodeMain;

import GlobalNavigation.MapGUI;

public class NavigationGUI extends MapGUI implements NodeMain {

	public NavigationGUI(){
		super();
	}
	
	public void draw(BoundingBox box, boolean filled, Color color) {
		addRect(box.min.x, box.min.y, box.max.x - box.min.x, box.max.y - box.min.y, filled, color);
	}
	
	public void draw(Polygon polygon, boolean filled, Color color) {
		addPoly(Util.convertToPoint2D(polygon.points), true, filled, color);
	}
	
	public void draw(Shape shape, boolean filled, Color color) {
		for (Polygon polygon : shape.polygons) {
			draw(polygon, filled, color);
		}		
	}
	
	public void draw(World world) {
		draw(world.getRegion(), false, Color.BLACK);
		for (Polygon obstacle : world.getObstacles()) {
			draw(obstacle, true, Color.BLACK);
		}
	}
	
	public void draw(List<Configuration> path) {
		if (path == null || path.size() < 1) return;
		for (int i = 1; i < path.size(); i ++) {
			addSegment(path.get(i-1).x, path.get(i-1).y, path.get(i).x, path.get(i).y, Color.GREEN);
		}
	}	
	
	public void draw(World world, List<Configuration> path) {
		if (path == null || path.size() < 1) return;
		for (int i = 0; i < path.size(); i ++) {
			draw(world.getRobot(path.get(i)), false, Color.GREEN);
		}
	}
	
	public void draw(TreeNode node, Color color) {
		for (TreeNode child : node.children) {
			addSegment(node.config.x, node.config.y, child.config.x, child.config.y, color);
			draw(child, color);
		}
	}	
	
	public void draw(World world, TreeNode node, Color color) {
		for (TreeNode child : node.children) {
			addSegment(node.config.x, node.config.y, child.config.x, child.config.y, color);
			draw(world, child, color);
		}
		if (node.children.size() == 0) {
			draw(world.getRobot(node.config), false, color);
		}
	}	
}
