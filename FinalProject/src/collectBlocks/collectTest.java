package collectBlocks;

import java.io.IOException;
import java.text.ParseException;

import kinect.KinectData;
import master.DrivingMaster;
import master.GatesController;
import navigation.Block;
import navigation.World;

import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;

public class collectTest implements NodeMain, Runnable {

	private BlockCollector camProc;
	private DrivingMaster driveMaster;
	private KinectData kinecter;
	public GatesController gates;
	
	public collectTest() {
		
	}
	
	public void run() {
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		camProc.takeOverDriving(true);
	}
	
	public void onStart(Node node) {
		
		
		
		// set up driving module
		driveMaster = new DrivingMaster();
		driveMaster.onStart(node);
		// set up kinect
		ParameterTree paramTree = node.newParameterTree();
		String mapFileName = paramTree.getString(node
				.resolveName("~/mapFileName"));
		int divideScale = 4;
		try {
			kinecter = new KinectData(new World(mapFileName), 4);
		} catch (Exception e) {
			e.printStackTrace();
		}
		kinecter.onStart(node);
		// set up gates controller
				gates = new GatesController();
				gates.onStart(node);
		// set up camera processing
		camProc = new BlockCollector(driveMaster, kinecter, gates, divideScale);
		camProc.onStart(node);
		camProc.setProcessing(true);
		camProc.takeOverDriving(false);
		
		
		Thread runThis = new Thread(this);
		runThis.start();
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
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}
}
