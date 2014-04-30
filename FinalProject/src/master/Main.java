package master;

import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;

import collectBlocks.BlockInfo;
import collectBlocks.BlockCollector;


public class Main implements NodeMain, Runnable {

	private BlockCollector camProc;
	private DrivingMaster driveMaster;
	
	public Main() {
		driveMaster = new DrivingMaster();
		camProc = new BlockCollector(driveMaster);
	}
	
	@Override
	public void run() {
		while (true) {
			// make decisions about which modules should be activated based on information they have
			BlockInfo b = camProc.largestBlob();
			if (b.size > 100) {
				// do something
			}
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
		driveMaster = new DrivingMaster();
		driveMaster.onStart(node);
		// set up camera processing
		camProc.onStart(node);
		camProc.setProcessing(true);
		camProc.takeOverDriving(false);
		
		Thread runThis = new Thread(this);
		runThis.start();
	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}

	

}
