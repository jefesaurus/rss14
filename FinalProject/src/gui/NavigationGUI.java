package gui;

import java.awt.Color;
import java.util.List;
import navigation.*;
import org.ros.node.NodeMain;

public class NavigationGUI extends ChallengeGUI implements NodeMain {

	public World world;
	public NavigationGUI(World world){
		this.world = world; //TODO overwrite ROBOT_SHAPE in VisionGUI to draw new robot
	}

	public void draw(Point point, Color color) {
		double radius = .025;
		addEllipse(point.x, point.y, 2*radius, 2*radius, true, color);
	}
	
	public void draw(Point point, double radius, Color color) {
		addEllipse(point.x, point.y, 2*radius, 2*radius, true, color);
	}

	public void draw(BoundingBox box, boolean filled, Color color) {
		addRect(box.min.x, box.min.y, box.max.x - box.min.x, box.max.y - box.min.y, filled, color);
	}

	public void draw(Polygon polygon, boolean filled, Color color) {
		addPoly(Util.convertToPoint2D(polygon.points), true, filled, color);
	}

	public void draw(Fiducial fiducial){
		addEllipse(fiducial.position.x, fiducial.position.y, 2*(fiducial.bottomRadius + .01), 2*(fiducial.bottomRadius + .01), true, fiducial.bottomColor);
		addEllipse(fiducial.position.x, fiducial.position.y, 2*(fiducial.topRadius - .01), 2*(fiducial.topRadius - .01), true, fiducial.topColor);
	}

	public void draw(Block block){
		if (block.isKnownSize() && block.size > 1) {
			addRect(block.position.x - block.LENGTH, block.position.y - block.LENGTH/2, 2*block.LENGTH, block.LENGTH, true, block.color);
		} else {
			addRect(block.position.x - block.LENGTH/2., block.position.y - block.LENGTH/2, block.LENGTH, block.LENGTH, true, block.color);
		}
	}

	public void draw(Shape shape, boolean filled, Color color) {
		for (Polygon polygon : shape.polygons) {
			draw(polygon, filled, color);
		}		
	}

	public void draw(Grid grid, Color color) {
		for (Point point : grid.getColliding()) {
			draw(point, Constants.GRID_RESOLUTION/2., color);
		}
	}

	public void draw() { //TODO Draw order
		draw(world.getRegion(), false, Color.BLACK);
		for (Fiducial fid : world.getFiducials()) {
			draw(fid);
		}
		for (Block block : world.getBlocks()) {
			draw(block);
		}
		for (Polygon obstacle : world.getObstacles()) {
			draw(obstacle, true, Color.BLACK);
		}
		draw(world.getStart(), Color.BLUE);
		draw(world.getGoal(), Color.YELLOW);
		
		draw(world.getOccupancyGrid(), Color.RED);
	}
	
	public void draw(Configuration robot) { //TODO Draw order
		draw(world.getRegion(), false, Color.BLACK);
		for (Fiducial fid : world.getFiducials()) {
			draw(fid);
		}
		for (Block block : world.getBlocks()) {
			draw(block);
		}
		for (Polygon obstacle : world.getObstacles()) {
			draw(obstacle, true, Color.BLACK);
		}
		draw(world.getStart(), Color.BLUE);
		draw(world.getGoal(), Color.YELLOW);
		
		draw(world.getOccupancyGrid(), Color.RED);
		
		draw(world.getRobot(robot), false, Color.GREEN);
		draw(world.getViewCone(robot), false, Color.GREEN);
	}
	

	public void draw(List<Waypoint> path, Color color) {
		if (path == null || path.size() < 1) return;
		for (Waypoint waypoint : path) {
			draw(world.getRobot(waypoint.config, waypoint.grow), false, color);
		}
	}

	public void draw(RRT tree, Color color) {
		if (tree != null) {
			draw(tree.root, color);
		}
	}
	
	public void draw(TreeNode node, Color color) {
		for (TreeNode child : node.children) {
			if (node.config != null) {
				addSegment(node.config.x, node.config.y, child.config.x, child.config.y, color);
			}
			draw(child, color);
		}
	}	

	public void clear() {
		eraseLine();
		eraseSegments();
		erasePoints();
		eraseRects();
		erasePolys();
		eraseEllipses();
		eraseStrings();
	}

}
