package navigation;

import gui.*;
import java.awt.Color;
import java.util.LinkedList;
import java.util.List;

import org.ros.message.lab6_msgs.GUIRectMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Subscriber;

public class NavigationTest implements NodeMain {

	public Node node;
	public World world;
	private NavigationGUI gui;
	private Navigator navigator;

	public NavigationTest() {		
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
		gui = new NavigationGUI(world);
		Configuration start = world.getStart().configuration(0);
		Configuration goal = world.getGoal().configuration(3*Math.PI/2);

		gui.clear();
		gui.draw();
		gui.draw(world.getRobot(start), true, Color.BLUE);
		gui.draw(world.getRobot(goal), true, Color.RED);
		gui.draw(world.getViewCone(start), false, Color.BLUE);
		gui.draw(world.getOccupancyGrid(), Color.RED);
		gui.draw(world.getVisibilityGrid(), Color.GREEN);
		
		navigator = new Navigator(node, gui, world);

		//switchControlDemo(goal);
		blocksDemo(goal);
	}
	
	private void switchControlDemo(Configuration goal) {
		navigator.setGoal(goal);
		Util.pause(10000);
		navigator.freeze();
		Util.pause(10000);
		navigator.resume();
	}
	
	private void blocksDemo(Configuration goal) {
		long startTime = System.currentTimeMillis();
		long collectionTime = 120*1000; //TODO Global clock 
		List<Point> collected = new LinkedList<Point>();
		for (Point block : world.getBlocks()) { //TODO sort by distance of RRT paths
			Configuration config = world.sampleConfigurationForPoint(block);
			if (config == null) {
				continue;
			}
			navigator.setGoal(config);
			
			while (navigator.getGoal() != null && (System.currentTimeMillis() - startTime) < collectionTime) {
				Util.pause(100);
			}
			if (navigator.getGoal() == null ) {
				collected.add(block); //TODO adds failed RRT paths too
			} 
			if ((System.currentTimeMillis() - startTime) >= collectionTime) {
				break;
			} 
		}
		//Done collecting, return to goal
		navigator.setGoal(goal); 
		System.out.println("Collected " + collected.size() + " blocks");
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
