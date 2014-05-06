package master;

import gui.NavigationGUI;

import java.awt.Color;

import kinect.KinectData;

import navigation.Block;
import navigation.Configuration;
import navigation.Navigator;
import navigation.Util;
import navigation.World;

import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;

import collectBlocks.BlockInfo;
import collectBlocks.BlockCollector;


public class Main implements NodeMain, Runnable {

	private BlockCollector blockCol;
	private DrivingMaster driveMaster;
	private KinectData kinecter;
	public GatesController gates;
	public Node node;
	public World world;
	private NavigationGUI gui;
	private Navigator navigator;
	
	public Main() {
		int divideScale = 4;
		driveMaster = new DrivingMaster();
		gates = new GatesController();
		kinecter = new KinectData(divideScale);
		blockCol = new BlockCollector(driveMaster, kinecter, gates, divideScale);
	}
	
	@Override
	public void run() {
		while (true) {
			// make decisions about which modules should be activated based on information they have
			// if less than 30 seconds left
			boolean runningOutOfTime = false;
			if (runningOutOfTime) {
				System.out.println("running out of time");
				// prioritize shelter building
			} else {
				// block in the area
				if (blockCol.blockImminent()) {
					System.out.println("collecting");
					// do block collection
//					navigator.freeze();
					blockCol.takeOverDriving(true);
				} else {
					System.out.println("navigating");
					// do navigation
//					navigator.resume();
					blockCol.takeOverDriving(false);
				}
			}
			// only run this every second. It's not necessary to always be making these decisions
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
				continue;
			}
		}
		
	}
	
	@Override
	public void onShutdown(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onStart(Node node) {
		// create navigator
		System.out.println("creating navigator");
		ParameterTree paramTree = node.newParameterTree();
		String mapFileName = paramTree.getString(node
				.resolveName("~/mapFileName"));
		try {
			world = new World(mapFileName);
		} catch (Exception e) {
			e.printStackTrace();
		}
		this.node = node;
//		gui = new NavigationGUI(world);
//		Configuration start = world.getStart().configuration(0);
//		Configuration goal = world.getGoal().configuration(Math.PI);
//
//		gui.clear();
//		gui.draw();
//		gui.draw(world.getRobot(start), true, Color.BLUE);
//		gui.draw(world.getRobot(goal), true, Color.RED);
//		gui.draw(world.getViewCone(start), false, Color.BLUE);
//		gui.draw(world.getOccupancyGrid(), Color.RED);
//		gui.draw(world.getVisibilityGrid(), Color.GREEN);
//		
//		navigator = new Navigator(node, gui, world);
		
		System.out.println("Creating the rest");
		// set up driving module
		driveMaster.onStart(node);
		// set up kinect
		kinecter.onStart(node);
		// set up camera processing
		blockCol.onStart(node);
		blockCol.setProcessing(true);
		blockCol.takeOverDriving(false);
		// set up gates controller
		gates.onStart(node);
		
//		for (Block block : world.getBlocks()) {
//			navigator.newGoal(block.position);
//		}
//		navigator.newGoal(goal);
		
		Thread runThis = new Thread(this);
		runThis.start();
	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}

	

}
