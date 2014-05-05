package master;

import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;

public class GateTest implements NodeMain, Runnable {

	public GatesController gates;
	
	public GateTest() {
		gates = new GatesController();
	}
	
	@Override
	public void onStart(Node node) {
		gates.onStart(node);
		new Thread(this).start();
	}

	@Override
	public void run() {
		try {
			// open front gate
			System.out.println("opening front gate");
			gates.openFrontGate();
			Thread.sleep(3000);
			// close front gate
			System.out.println("Closing front gate");
			gates.closeFrontGate();
			Thread.sleep(3000);
			// open front gate
			System.out.println("Opening front gate again");
			gates.openFrontGate();
			Thread.sleep(3000);
		} catch (InterruptedException e1) {
			e1.printStackTrace();
		}
		
	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}
	
	@Override
	public void onShutdown(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}

}
