package master;

import gui.NavigationGUI;

import java.awt.Color;

import kinect.KinectData;

import navigation.Configuration;
import navigation.Navigator;
import navigation.World;

import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;

import collectBlocks.BlockInfo;
import collectBlocks.BlockCollector;


public class Main implements NodeMain, Runnable {

	private BlockCollector camProc;
	private DrivingMaster driveMaster;
	private KinectData kinecter;
	public Node node;
	public World world;
	private NavigationGUI gui;
	private Navigator navigator;
	
	public Main() {
		int divideScale = 4;
		driveMaster = new DrivingMaster();
		kinecter = new KinectData(divideScale);
		camProc = new BlockCollector(driveMaster, kinecter, divideScale);
	}
	
	@Override
	public void run() {
		while (true) {
			// make decisions about which modules should be activated based on information they have
			// if less than 30 seconds left
			boolean runningOutOfTime = false;
			if (runningOutOfTime) {
				// prioritize shelter building
			} else {
				// go around and collect blocks
				BlockInfo b = camProc.largestBlob();
				// block of big enough size
				if (b.size > 50) {
					// do block collection
					navigator.freeze();
					camProc.takeOverDriving(true);
				} else {
					// do navigation
					navigator.resume();
					camProc.takeOverDriving(false);
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
		
		// TODO Auto-generated method stub
		// set up driving module
		driveMaster.onStart(node);
		// set up kinect
		kinecter.onStart(node);
		// set up camera processing
		camProc.onStart(node);
		camProc.setProcessing(true);
		camProc.takeOverDriving(true);
		
		navigator.setGoal(goal);
		
		Thread runThis = new Thread(this);
		runThis.start();
	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}

	

}
