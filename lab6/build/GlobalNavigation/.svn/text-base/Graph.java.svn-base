package GlobalNavigation;

import java.awt.Color;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Point2D.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;
import java.lang.Double;
import java.util.Map;


public class Graph {
	PolygonMap map;
	//List<Line2D.Double> edges;
	List<Point2D.Double> vertices;
	int resolution;
	PolygonObstacle robot;
	Map<Double, CSpaceSingle> cspaces;
	Map<Integer, List<Integer>> graph;
	
	public Graph(PolygonMap map, int resolution, PolygonObstacle robot) {
		this.map = map;
		this.resolution = resolution;
		//this.edges = new ArrayList<Line2D.Double>();
		this.vertices = new ArrayList<Point2D.Double>();
		this.robot = robot;
		this.cspaces = new HashMap<Double, CSpaceSingle>();
		this.graph = new HashMap<Integer, List<Integer>>();
		computeGraph();
	}
	
	public void computeGraph() {
		ArrayList<Point2D.Double> points = new ArrayList<Point2D.Double>();
		for (PolygonObstacle obstacle : this.map.obstacles) {
			for (Point2D.Double vert : obstacle.getVertices()) {
				points.add(vert);
			}
		}
		int currentVertex = 0;
		for (int s = 0; s < points.size(); s ++) {
			for (int e = 0; e < points.size(); e ++) {
				Point2D.Double start = points.get(s);
				Point2D.Double end = points.get(e);
				if (canSeeMap(start, end)) {
					Line2D.Double line = new Line2D.Double(start.x, start.y, end.x, end.y);
					//this.edges.add(line);
					double incrementX = (line.x2 - line.x1)/(double)this.resolution;
					double incrementY = (line.y2 - line.y1)/(double)this.resolution;
					double currX = line.x1 + incrementX;
					double currY = line.y1 + incrementY;
					for (int step = 0; step < this.resolution; step ++) {
						Point2D.Double newVertex = new Point2D.Double(currX, currY);
						this.vertices.add(newVertex);
						currX += incrementX;
						currY += incrementY;
						this.graph.put(currentVertex, new ArrayList<Integer>());
						currentVertex ++;
					}
				}
			}
		}
		
		for (int s = 0; s < this.vertices.size(); s ++) {
			for (int e = s + 1; e < this.vertices.size(); e ++) {
				Point2D.Double start = this.vertices.get(s);
				Point2D.Double end = this.vertices.get(e);
				if (canSeeCSpace(start, end)) {
					this.graph.get(s).add(e);
					this.graph.get(e).add(s);
				}
			}
		}
	}
	
	public int addVertex(Point2D.Double vertex) {
		int indexToInsert = this.vertices.size();
		this.vertices.add(vertex);
		this.graph.put(indexToInsert, new ArrayList<Integer>());

		for (int e = 0; e < indexToInsert - 1; e ++) {
			Point2D.Double testVertex = this.vertices.get(e);
			if (canSeeCSpace(vertex, testVertex)) {
				this.graph.get(indexToInsert).add(e);
				this.graph.get(e).add(indexToInsert);
			}
		}
		return indexToInsert;
	}
	
	public void drawMap(MapGUI gui) {
		for (PolygonObstacle obstacle : this.map.obstacles) {
			gui.addPoly(obstacle.getVertices(), true, true, Color.BLACK);
		}
	}
	
	public void drawGraph(MapGUI gui) {
		for (Integer vIndex : graph.keySet()) {
			Point2D.Double vertex = this.vertices.get(vIndex);
			gui.addPoint(vertex.x, vertex.y, 1, Color.BLUE);
			List<Integer> neighbors = this.graph.get(vIndex);
			for (Integer nIndex : neighbors) {
				Point2D.Double neighbor = this.vertices.get(nIndex);
				//gui.addSegment(vertex.x, vertex.y, neighbor.x, neighbor.y, Color.ORANGE);
			}
		}
	}
	
	public void drawCSpace(MapGUI gui, List<Integer> path) {
		
		int prevIndex = path.get(0);
		for (int i = 1; i < path.size(); i ++) {
			Integer currIndex = path.get(i);
			Point2D.Double curr = this.vertices.get(currIndex);
			Point2D.Double prev = this.vertices.get(prevIndex);
			gui.addSegment(prev.x, prev.y, curr.x, curr.y, Color.RED);
			
			CSpaceSingle cspace = getCSpace(prev, curr);
			cspace.draw(gui);
			cspace.draw(gui);
			
			prevIndex = currIndex;
		}
		
	}
	
	public void drawCSpace(MapGUI gui, List<Integer> path, int section) {
		int prevIndex = path.get(section);
		Integer currIndex = path.get(section+1);
		Point2D.Double curr = this.vertices.get(currIndex);
		Point2D.Double prev = this.vertices.get(prevIndex);
		double angle = Math.atan2(curr.x - prev.x, curr.y - prev.y);
		gui.setRobotPose(curr.x, curr.y, Math.PI/2-angle);
		gui.setRobotPose(prev.x, prev.y, Math.PI/2-angle);

		CSpaceSingle cspace = getCSpace(prev, curr);
		cspace.draw(gui);
	}
	
	public CSpaceSingle getCSpace(Point2D.Double start, Point2D.Double end) {
		double angle = Math.atan2(end.x - start.x, end.y - start.y);
		Double roundedAngle = round(angle, 2); // Rounds the angle to some number of places to aid with hashing

		if (!cspaces.containsKey(roundedAngle)) {
			CSpaceSingle cspace = new CSpaceSingle(map.getObstacles(), robot, angle);
			cspaces.put(roundedAngle, cspace);
		}
		return cspaces.get(roundedAngle);
	}
	
	private boolean canSeeCSpace(Point2D.Double start, Point2D.Double end){
		CSpaceSingle cspace = getCSpace(start, end);
		return canSee(start, end, cspace.getObstacles());
	}
	
	private boolean canSeeMap(Point2D.Double start, Point2D.Double end){
		return canSee(start, end, this.map.getObstacles());
	}
	
	private boolean canSee(Point2D.Double start, Point2D.Double end, List<PolygonObstacle> obstacles){
		Line2D.Double startToEndLine = new Line2D.Double(start,end);
		for(PolygonObstacle po : obstacles){
			if(isMidpointInObstacle(startToEndLine,po)){
				return false;
			}
			ArrayList<Line2D.Double> lines = new ArrayList<Line2D.Double>();
			Point2D.Double last = po.getVertices().get(po.getVertices().size()-1);
			for(Point2D.Double vertex : po.getVertices()){
				lines.add(new Line2D.Double(last,vertex));
				last = vertex;
			}
			for(Line2D.Double otherLine : lines){
				if(startToEndLine.intersectsLine(otherLine) && !startToEndLine.equals(otherLine) &&
						!otherLine.getP1().equals(start) && !otherLine.getP2().equals(start) &&
						!otherLine.getP1().equals(end) && !otherLine.getP2().equals(end)){
					return false;
				}
			}
		}
		return true;
	}
	
	
	private boolean isMidpointInObstacle(Line2D.Double line, PolygonObstacle po){
		double midx = (line.getP1().getX() + line.getP2().getX()) / 2.0;
		double midy = (line.getP1().getY() + line.getP2().getY()) / 2.0;
		double delta = 0.01;//1 cm offset
		return (po.contains(midx-delta,midy-delta) &&
				po.contains(midx-delta,midy+delta) &&
				po.contains(midx+delta,midy-delta) &&
				po.contains(midx+delta,midy+delta));
	}
	
	public static Double round(double value, int places) {
	    if (places < 0) throw new IllegalArgumentException();

	    long factor = (long) Math.pow(10, places);
	    value = value * factor;
	    long tmp = Math.round(value);
	    return new Double((double)tmp / factor);
	}
	
	private double euclideanDistance(Integer s, Integer e){
		Point2D.Double start = this.vertices.get(s);
		Point2D.Double end = this.vertices.get(e);
		return Math.sqrt(Math.pow(start.getX()-end.getX(),2)+Math.pow(start.getY()-end.getY(),2));
	}
	
	
	public class QueueElement implements Comparable<QueueElement> {
		public Integer lastPoint;
		public ArrayList<Integer> path;
		public double distanceToGoal;
		public double distanceFromStart;
		public QueueElement(ArrayList<Integer> p, double distFromStart, double distToGoal, Integer last){
			path = new ArrayList<Integer>();
			path = p;
			distanceToGoal = distToGoal;
			distanceFromStart = distFromStart;
			lastPoint = last;
		}

		@Override
		public int compareTo(QueueElement obj){
			if (this.distanceToGoal+this.distanceFromStart < obj.distanceToGoal+obj.distanceFromStart) {
				return -1;
			}
			if (this.distanceToGoal+this.distanceFromStart > obj.distanceToGoal+obj.distanceFromStart) {
				return 1;
			}
			return 0;
		}
	}
	
	public List<Integer> computeShortestPath(Point2D.Double start, Point2D.Double goal){
		Integer startIndex = addVertex(start);
		Integer goalIndex = addVertex(goal);

		PriorityQueue<QueueElement> queue = new PriorityQueue<QueueElement>();

		HashSet<Integer> processed = new HashSet<Integer>();
		HashMap<Integer, Double> visited = new HashMap<Integer, Double>(); // vert -> distance
		
		ArrayList<Integer> initialPath = new ArrayList<Integer>();
		initialPath.add(startIndex);
		queue.add(new QueueElement(initialPath,0.0,euclideanDistance(startIndex,goalIndex), startIndex));
		while(!queue.isEmpty()){
			QueueElement current = queue.poll();
			Integer currentPointIndex = current.lastPoint;
			if(processed.contains(currentPointIndex)) {
				continue;
			}
			processed.add(currentPointIndex);

			if(currentPointIndex.equals(goalIndex)) {
				return current.path;
			}

			for(Integer neighbor : graph.get(currentPointIndex)){
				double newDistance = current.distanceFromStart+euclideanDistance(currentPointIndex,neighbor);
				
				if(visited.containsKey(neighbor) && visited.get(neighbor) <= newDistance) {
					continue;
				}
				
				ArrayList<Integer> newPath = softCopy(current.path);
				newPath.add(neighbor);
				visited.put(neighbor, newDistance);
				queue.add(new QueueElement(newPath,current.distanceFromStart+euclideanDistance(currentPointIndex,neighbor),
					euclideanDistance(neighbor,goalIndex), neighbor));
			}
		}
		return null;
	}
	
	public void test() {
		Integer x = new Integer(5);
		Integer y = new Integer(5);
		System.out.println(x.equals(y));
		System.out.println(x == y);
	}
	
	public ArrayList<Integer> softCopy(ArrayList<Integer> toCopy) {
		ArrayList<Integer> toReturn = new ArrayList<Integer>();
		for (Integer point : toCopy) {
			toReturn.add(point);
		}
		return toReturn;
	}
	
	public void drawPath(MapGUI gui, List<Integer> path) {
		if (path == null || path.size() < 1) return;
		int prevIndex = path.get(0);
		for (int i = 1; i < path.size(); i ++) {
			Integer currIndex = path.get(i);
			Point2D.Double curr = this.vertices.get(currIndex);
			Point2D.Double prev = this.vertices.get(prevIndex);
			gui.addSegment(prev.x, prev.y, curr.x, curr.y, Color.RED);
			prevIndex = currIndex;
		}
	}
	
	public List<Point2D.Double> cookPath(List<Integer> path) {
		List<Point2D.Double> rasterPath = new ArrayList<Point2D.Double>();
		for (Integer waypointIndex : path) {
			rasterPath.add(this.vertices.get(waypointIndex));
		}
		return rasterPath;
	}
}
