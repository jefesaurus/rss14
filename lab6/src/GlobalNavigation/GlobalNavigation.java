package GlobalNavigation;

import java.awt.Color;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.List;

import org.ros.message.lab6_msgs.GUIPolyMsg;
import org.ros.message.lab6_msgs.GUIRectMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Subscriber;

public class GlobalNavigation implements NodeMain {

	public PolygonMap map;
	public BotGeometry botgeo;
	public CSpace cspace;
	public MotionPlanner planner;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	public Node node;
	public WaypointNavigator nav;

	private MapGUI gui = new MapGUI();

	public GlobalNavigation() {
		botgeo = new BotGeometry();
		
	}

	@Override
	public void onStart(Node node) {
		ParameterTree paramTree = node.newParameterTree();
		String mapFileName = paramTree.getString(node
				.resolveName("~/mapFileName"));
		try {
			map = new PolygonMap(mapFileName);
		} catch (Exception e) {
			e.printStackTrace();
		}
		this.node = node;
		
		Graph g = new Graph(map, 2, botgeo.getBotPoly(0.0));
		Point2D.Double start = map.robotStart;
		Point2D.Double end = map.robotGoal;
		gui.addPoint(start.x, start.y, 1, Color.GREEN);
		gui.addPoint(end.x, end.y, 1, Color.BLUE);
		List<Integer> path = g.computeShortestPath(start, end);
		if (path == null) {
			System.out.println("Couldn't find a path");
			return;
		}
		List<Point2D.Double> pointPath = g.cookPath(path);
		//nav = new WaypointNavigator(node, pointPath, gui);
		System.out.println(path.size());
		g.drawMap(gui);
		g.drawGraph(gui);
		//g.drawPath(gui, path);

		//g.drawCSpace(gui, path, 0);
		
		//Thread runningStuff = new Thread(nav);
		//runningStuff.start();
	}

	public void testMinkSum() {
		cspace.compute(2);
		cspace.draw(gui);
		Point2D.Double punto = new Point2D.Double(1., 1.);
		gui.addPoint(punto.x, punto.y, 0);
		System.out.println(cspace.intersects(punto, 0));
		System.out.println(cspace.intersects(punto, 1));
	}

	// Finished the test convex hull part in part 2
	public void testConvexHull() {
		int NUM_POINTS = 20;
		double MIN_X = -2;
		double MAX_X = 2;
		double MIN_Y = -2;
		double MAX_Y = 2;
		ArrayList<Point2D.Double> points = new ArrayList<Point2D.Double>();
		System.out.println("Points in cloud:");
		for (int i = 0; i < NUM_POINTS; i++) {
			Point2D.Double punto = new Point2D.Double(Math.random()
					* (MAX_X - MIN_X) + MIN_X, Math.random() * (MAX_Y - MIN_Y)
					+ MIN_Y);
			points.add(punto);
			gui.addPoint(punto.x, punto.y, 1);
			System.out.println(punto);
		}
		PolygonObstacle hull = GeomUtils.convexHull(points);
		gui.addPoly(hull.getVertices(), true, false, Color.BLUE);
	}
	
	@Override
	public void onShutdown(Node node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onShutdownComplete(Node node) {
		// TODO Auto-generated method stub

	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}

	public static void fillRectMsg(GUIRectMsg rectMsg,
			java.awt.geom.Rectangle2D.Double worldRect, Object object, boolean b) {
		// TODO Auto-generated method stub

	}
}
