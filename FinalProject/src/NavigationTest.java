

import gui.*;
import navigation.*;

import java.awt.Color;
import java.awt.geom.Point2D;

import org.ros.message.lab6_msgs.GUIPolyMsg;
import org.ros.message.lab6_msgs.GUIRectMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Subscriber;

public class NavigationTest implements NodeMain {

	public PolygonMap map;
	//public BotGeometry botgeo;
	public MotionPlanner planner;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	public Node node;
	public WaypointNavigator nav;

	private GUI gui = new GUI();

	public NavigationTest() {
		//botgeo = new BotGeometry();
		
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
		
		//Graph g = new Graph(map, 2, botgeo.getBotPoly(0.0));
		Point2D.Double start = map.robotStart;
		Point2D.Double end = map.robotGoal;
		gui.addPoint(start.x, start.y, 1, Color.GREEN);
		gui.addPoint(end.x, end.y, 1, Color.BLUE);
		//List<Integer> path = g.computeShortestPath(start, end);
		//if (path == null) {
		//	System.out.println("Couldn't find a path");
		//	return;
		//}
		//List<Point2D.Double> pointPath = g.cookPath(path);
		//nav = new WaypointNavigator(node, pointPath, gui);
		//System.out.println(path.size());
		//g.drawMap(gui);
		//g.drawGraph(gui);
		//g.drawPath(gui, path);

		//g.drawCSpace(gui, path, 0);
		
		//Thread runningStuff = new Thread(nav);
		//runningStuff.start();
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
