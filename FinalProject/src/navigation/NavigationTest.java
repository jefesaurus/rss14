package navigation;

import gui.*;
import java.awt.Color;
import java.util.LinkedList;
import java.util.List;

import kinect.KinectData;

import navigation.Constants.CollisionCheck;
import navigation.Constants.DriveSystem;

import org.ros.message.lab6_msgs.GUIRectMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;

public class NavigationTest implements NodeMain {

	public Node node;
	public World world;
	private NavigationGUI gui;
	private Navigator navigator;
	private KinectData kinecter;

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
		Configuration start = world.getStart().configuration(0.);
		Configuration goal = world.getGoal().configuration(Math.PI);

		gui.clear();
		gui.draw();
		gui.draw(world.getRobot(start), true, Color.BLUE);
		gui.draw(world.getRobot(goal), true, Color.YELLOW);
		//gui.draw(world.getOccupancyGrid(), Color.RED);

		//gui.draw(world.getViewCone(start), false, Color.BLUE);
		//gui.draw(world.getVisibilityGrid(), Color.GREEN);
		
		/*MotionPlanner planner = new MotionPlanner(world);
		List<Waypoint> newPath = planner.plan(start, goal, new LinearGrowthParameters(DriveSystem.FORWARD, CollisionCheck.GRIDONLY, .0, .0));
		gui.draw(newPath, Color.GREEN);
		gui.draw(planner.tree1, Color.BLUE);
		gui.draw(planner.tree2, Color.RED);
		if (newPath != null) {
			System.out.println("Found path of length " + newPath.size() + " from " + start + " to " + goal + "\nTook " + planner.attempts + " attempts, " +
				planner.iterations + " iterations, " + planner.time/1000.0 + " seconds");
		} else {
			System.out.println("Could not find path from " + start + " to " + goal);
		}*/
		
		navigator = new Navigator(node, gui, world);
		//navigator.newGoal(goal);

		int divideScale = 4;
		kinecter = new KinectData(world, divideScale);
		kinecter.onStart(node);
		
		
		//switchControlDemo(goal);
		//blocksDemo(goal);
		//blocksDemoAddAll(goal);
	}
	
	private void switchControlDemo(Configuration goal) {
		navigator.newGoal(goal);
		Util.pause(10000);
		navigator.freeze();
		Util.pause(10000);
		navigator.resume();
	}
	
	private void blocksDemo(Configuration goal) {
		long startTime = System.currentTimeMillis();
		long collectionTime = 120*1000; //TODO Global clock 
		List<Block> collected = new LinkedList<Block>();
		for (Block block : world.getBlocks()) { //TODO sort by distance of RRT paths
			//Configuration config = world.sampleConfigurationForPoint(block.position);
			//if (config == null) {
			//	continue;
			//}
			navigator.newGoal(block.position);
			
			while (navigator.currentGoal() == block.position && (System.currentTimeMillis() - startTime) < collectionTime) {
				Util.pause(100);
			}
			if (navigator.accomplished.size() != 0 && navigator.accomplished.get(navigator.accomplished.size()-1) == block.position) {
				collected.add(block); 
			} 
			if ((System.currentTimeMillis() - startTime) >= collectionTime) {
				break;
			} 
		}
		//Done collecting, return to goal
		navigator.clearGoals();
		navigator.newGoal(goal);
		//navigator.newGoal(backward);

		System.out.println("Collected " + collected.size() + " blocks");
	}
	
	private void blocksDemoAddAll(Configuration goal) {
		long startTime = System.currentTimeMillis();
		long collectionTime = 300*1000; //TODO Global clock 		
		for (Block block : world.getBlocks()) {
			navigator.newGoal(block.position);
		}
		while ((System.currentTimeMillis() - startTime) < collectionTime && navigator.currentGoal() != null) {
			Util.pause(100);
		}
		
		//Done collecting, return to goal
		navigator.clearGoals();
		Util.pause(10);
		navigator.newGoal(goal);
		//navigator.newGoal(world.getStart().configuration(Math.PI));
		//Configuration backward = world.sampleBackwardsConfiguration(goal);
		//if (backward != null) {
		//	navigator.newGoal(backward);
		//}
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
