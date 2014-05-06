package digitalIO;

import orc.DigitalInput;
import orc.Orc;

import org.ros.message.rss_msgs.BumpMsg;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

public class BackBumperPublisher implements Runnable {

	public static final String BUMP_MSG = "rss_msgs/BumpMsg";
	public static final String BUMP_CHANNEL = "rss/BackBumpSensors";

	private DigitalInput left;
	private DigitalInput right;
	private BumpMsg msg;
	private Publisher<BumpMsg> pub;
	private Object lock;

	public BackBumperPublisher(Node node, Orc orc, Object lock){
		this.lock = lock;
		left = new DigitalInput(orc, 2, true, true);
		right = new DigitalInput(orc, 3, true, true);
		pub = node.newPublisher(BUMP_CHANNEL, BUMP_MSG);
	}

	@Override public void run() {
		msg = new BumpMsg();
		while(true){
			synchronized(lock) {
				msg.left = left.getValue();
				msg.right = right.getValue();
			}
			pub.publish(msg);
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
}
