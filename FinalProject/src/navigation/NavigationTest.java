package navigation;

import gui.*;
import java.awt.Color;
import java.util.List;

import org.ros.message.lab6_msgs.GUIRectMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Subscriber;

public class NavigationTest implements NodeMain {

	public World world;
	public MotionPlanner planner;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	public Node node;

	private NavigationGUI gui;
	private WaypointNavigator nav;

	public NavigationTest() {		
		gui = new NavigationGUI();
	}

	@Override
	public void onStart(Node node) {
		ParameterTree paramTree = node.newParameterTree();
		String mapFileName = paramTree.getString(node
				.resolveName("~/mapFileName"));
		try {
			world = new World(mapFileName);
		} catch (Exception e) {
			e.printStackTrace();
		}
		this.node = node;
		planner = new MotionPlanner(world);
		nav = new WaypointNavigator(node, gui);
		
		Configuration start = world.getStart().configuration(0);
		Configuration goal = world.getGoal().configuration(3*Math.PI/2);
		//Configuration goal = new Configuration(2, 3.85, 0);
		//Configuration goal = new Configuration(2, 2.5, 0);

		System.out.println("Start collision?: " + world.robotCollision(start));
		System.out.println("Goal collision?: " + world.robotCollision(goal));
		
		gui.draw(world);
		gui.draw(world.getRobot(start), true, Color.BLUE);
		gui.draw(world.getViewCone(start), false, Color.BLUE);
		gui.draw(world.getRobot(goal), true, Color.RED);
		
		List<Configuration> path = planner.findPath(start, goal);
		if (path != null) {
			System.out.println("Found path of length " + path.size() + " in " + planner.iterations + " iterations");
			//gui.draw(path);
			//gui.draw(world, path);
			nav.setPath(path);
		} else {
			System.out.println("Could not find path in " + planner.iterations + " iterations");
		}
		
		//gui.draw(world, planner.tree1.root, Color.RED);
		//gui.draw(world, planner.tree2.root, Color.BLUE);
		gui.draw(planner.tree1.root, Color.RED);
		gui.draw(planner.tree2.root, Color.BLUE);
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
