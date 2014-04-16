package GlobalNavigation;

import java.awt.Color;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Point2D.Double;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class GridBasedPlanner implements MotionPlanner {
	
	protected List<Point2D.Double> path = null;
	private Grid gridMap = null;
	private CSpace cspace = null;
	
	/**
	 * 
	 * @param cspace
	 * @param worldRect
	 * @param resolution in meters
	 */
	public GridBasedPlanner(CSpace cspace, Rectangle2D.Double worldRect, double resolution) {
		gridMap = new Grid(worldRect, resolution);
		this.cspace = cspace;
		populateGrid();
	}
	
	// use cspace to choose which cells are occupied and which aren't
	private void populateGrid() {
		for (List<PolygonObstacle> obstacles : cspace.rotationLayers()) {
			for (PolygonObstacle obstacle : obstacles) {
				int marked = gridMap.markObstacle(obstacle);
				System.out.println("Marked cells: " + marked);
			}
		}
		/*for (PolygonObstacle obstacle : cspace.getObstacles()) {
			int marked = gridMap.markObstacle(obstacle);
			System.out.println("Marked cells: " );
		}*/
	}
	
	@Override
	public List<Point2D.Double> computePath(Point2D.Double start, Point2D.Double goal) {
		double distance = gridMap.computeShortestPaths(goal); // already present grid method for shortest path
		Grid.Cell cell = gridMap.getCell(start).toGoalNext;
		if (cell == null) {return null;}
		Grid.Cell goalCell = gridMap.getCell(goal);
		// walk through cells until we reach the goal cell
		List<Point2D.Double> answer = new ArrayList<Point2D.Double>();
		while (cell != goalCell) {
			answer.add(cell.makeCenterPoint());
			cell = cell.toGoalNext;
		}
		path = answer;
		return answer;
	}

	@Override
	public void draw(MapGUI gui) { 
		/*Iterator<Grid.Cell> iter = gridMap.iterator();
		Grid.Cell cell;
		while (iter.hasNext()) {
			cell = iter.next();
			if (cell.free) {
				gui.addRect(cell.getCenterX(), cell.getCenterY(), .05, .05, true, Color.green);
			} else {
				gui.addRect(cell.getCenterX(), cell.getCenterY(), .05, .05, true, Color.blue);
			}
		}*/
		
		if (path == null) {
			// put red square
			gui.addRect(-1, -1, 2, 2, true, Color.pink);
			return;
		}
		for (Point2D.Double point : path) {
			gui.addRect(point.x, point.y, .05, .05, true, Color.green);
		}
	}

}
