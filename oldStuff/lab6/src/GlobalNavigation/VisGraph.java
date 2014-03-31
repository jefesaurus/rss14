package GlobalNavigation;

import java.awt.Color;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

class VisGraphNode {
	Point2D.Double loc;
	double angle;
	List<String> neighbors;
	public VisGraphNode(Point2D.Double loc, double angle) {
		this.loc = loc;
		this.angle = angle;
	}
}

class VisGraphLayer {
	int index;
	double angle;
	public List<Point2D.Double> endpoints;
	public List<Line2D.Double> lines;
	
	// These are a subset of the above that are actually traversable
	public List<Point2D.Double> vertices;
	public List<Line2D.Double> edges;
	
	public VisGraphLayer(int index, double angle) {
		this.index = index;
		this.angle = angle;
		this.endpoints = new ArrayList<Point2D.Double>();
		this.lines = new ArrayList<Line2D.Double>();
		
		this.vertices = new ArrayList<Point2D.Double>();
		this.edges = new ArrayList<Line2D.Double>();
	}
}

public class VisGraph {
	int numLayers;
	  List<VisGraphLayer> layers;

	  List<Double> angles;
	  double segLen = 3.0;
	  CSpace cspace;
	
	public VisGraph(CSpace cspace) {
		this.layers = new ArrayList<VisGraphLayer>();
		this.numLayers = cspace.angles.size();
		this.cspace = cspace;
		  
		for (int i = 0; i < numLayers; i ++) {
			List<PolygonObstacle> cspaceLayer = cspace.rotLayers.get(i);
			double angle = cspace.angles.get(i);
			VisGraphLayer layer = new VisGraphLayer(i, angle);

			for (PolygonObstacle obstacle : cspaceLayer) {
				List<Point2D.Double> verts = obstacle.getVertices();
				for (Point2D.Double vert : verts) {
					
					// First level of pruning gets rid of points in obstacles that intersected with other polys
					// in the same cspace layer
					if (!cspace.intersects(vert, i)) {
						layer.endpoints.add(vert);
						layer.lines.add(new Line2D.Double(vert.x - Math.cos(angle)*segLen, vert.y - Math.sin(angle)*segLen, 
								vert.x + Math.cos(angle)*segLen, vert.y + Math.sin(angle)*segLen));
					}
				}
			}
			this.layers.add(layer);
		}
	}
	
	public void makeGraph() {
		for (VisGraphLayer baseLayer : this.layers) {
			for (VisGraphLayer testLayer : this.layers) {
				if (baseLayer.index == testLayer.index) continue;
				
				for (int i = 0; i < baseLayer.lines.size(); i ++) {
					Line2D.Double baseLine = baseLayer.lines.get(i);
					Point2D.Double basePoint = baseLayer.endpoints.get(i);

					for (Line2D.Double testLine : testLayer.lines) {
						Point2D.Double intersection = GeomUtils.getIntersection(baseLine, testLine);
						if (intersection != null && canSee(basePoint, intersection)) {
							baseLayer.vertices.add(intersection);
						}
					}
				}
			}
		}
	}
	
	public void draw(MapGUI gui) {
		for (VisGraphLayer layer : this.layers) {
			for (Point2D.Double point : layer.vertices) {
				gui.addPoint(point.x, point.y, 0, Color.BLUE);
			}
		}
	}
	
	public void testIntersect(MapGUI gui, CSpace cspace) {
		double segLen = 1;
		for (int i = 0; i < cspace.rotLayers.size(); i ++) {
			List<PolygonObstacle> cspaceLayer = cspace.rotLayers.get(i);
			double angle = cspace.angles.get(i);
			for (PolygonObstacle obstacle : cspaceLayer) {
				List<Point2D.Double> verts = obstacle.getVertices();
				for (Point2D.Double vert : verts) {
					// First level of pruning gets rid of points in obstacles that intersected with other polys
					// in the same cspace layer
					if (!cspace.intersects(vert, i)) {
						gui.addPoint(vert.x, vert.y, 1, Color.GREEN);
						gui.addSegment(vert.x, vert.y, vert.x + Math.cos(angle)*segLen, vert.y + Math.sin(angle)*segLen, Color.BLACK);
						gui.addSegment(vert.x, vert.y, vert.x - Math.cos(angle)*segLen, vert.y - Math.sin(angle)*segLen, Color.BLACK);

					}
				}
			}
		}
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
	
	private boolean canSee(Point2D.Double start, Point2D.Double end){
		Line2D.Double startToEndLine = new Line2D.Double(start,end);

		for(PolygonObstacle po : this.cspace.getObstacles()){
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
}
