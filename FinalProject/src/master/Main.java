package master;

import kinect.KinectData;

import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;

import collectBlocks.BlockInfo;
import collectBlocks.BlockCollector;


public class Main implements NodeMain, Runnable {

	private BlockCollector camProc;
	private DrivingMaster driveMaster;
	private KinectData kinecter;
	
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
				if (b.size > 100) {
					// do block collection
				} else {
					// do navigation
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
		// TODO Auto-generated method stub
		// set up driving module
		driveMaster.onStart(node);
		// set up kinect
		kinecter.onStart(node);
		// set up camera processing
		camProc.onStart(node);
		camProc.setProcessing(true);
		camProc.takeOverDriving(true);
		
		Thread runThis = new Thread(this);
		runThis.start();
	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}

	

}
