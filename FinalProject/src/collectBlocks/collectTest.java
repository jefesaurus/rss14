package collectBlocks;

import kinect.KinectData;
import master.DrivingMaster;
import master.GatesController;
import navigation.Block;

import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;

public class collectTest implements NodeMain, Runnable {

	private BlockCollector camProc;
	private DrivingMaster driveMaster;
	private KinectData kinecter;
	public GatesController gates;
	
	public collectTest() {
		int divideScale = 4;
		driveMaster = new DrivingMaster();
		gates = new GatesController();
		kinecter = new KinectData(divideScale);
		camProc = new BlockCollector(driveMaster, kinecter, gates, divideScale);
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
		driveMaster.onStart(node);
		// set up kinect
		kinecter.onStart(node);
		// set up camera processing
		camProc.onStart(node);
		camProc.setProcessing(true);
		camProc.takeOverDriving(false);
		// set up gates controller
		gates.onStart(node);
		
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
