package cameraProcessing;

import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

/**
 * 
 * @author previous TA's, prentice, vona
 *
 */
public class VisualServo implements NodeMain, Runnable {

	private static final int width = 160*4;

	private static final int height = 120*4;


	/**
	 * <p>The Connected Components thing.</p>
	 **/
	private ConnComp cct = null;


	private VisionGUI gui;
	private ArrayBlockingQueue<byte[]> visionImage = new ArrayBlockingQueue<byte[]>(
			1);

	protected boolean firstUpdate = true;

	public Subscriber<org.ros.message.sensor_msgs.Image> vidSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;

	public Publisher<MotionMsg> motionPub;


	/**
	 * <p>Create a new VisualServo object.</p>
	 */
	public VisualServo() {
		setInitialParams();

		gui = new VisionGUI();
	}
	
	Image src = null;

	protected void setInitialParams() {

	}

	/**
	 * <p>Handle a CameraMessage. Perform blob tracking and
	 * servo robot towards target.</p>
	 * 
	 * @param rawImage a received camera message
	 */
	public void handle(byte[] rawImage) {

		visionImage.offer(rawImage);
	}

    public double RANGE_GAIN = .005;
    public double BEARING_GAIN = .005;

    public double desiredRange = 441; // desired centroid.y in pixels
    public double desiredBearing = 391; //desired centroid.x in pixels

	@Override
	public void run() {


		
		while (true) {
			try {
				src = new Image(visionImage.take(), width, height);
			} catch (InterruptedException e) {
				e.printStackTrace();
				continue;
			}

			Image dest = new Image(src);
			//blobTrack.apply(src, dest);
			List<BlockInfo> blockInfos = cct.visualize(src, dest);

			// update newly formed vision message
			gui.setVisionImage(dest.toArray(), width, height);


			// publish velocity messages to move the robot towards the target
            MotionMsg msg = new MotionMsg();
            double tv = 0.0;
            double rv = 0.0;

            if (!blockInfos.isEmpty()) {
            	BlockInfo b = blockInfos.get(0);
                // Very basic proportional controller
            	System.out.println("size : " + b.size + " | cx : " + b.centroid.x + " | cy : " + b.centroid.y);
	            double rangeError = -(b.centroid.y - desiredRange); // negative because y pixels increase downward
	            double bearingError = b.centroid.x - desiredBearing;
	            tv = rangeError*RANGE_GAIN;
	            rv = bearingError*BEARING_GAIN;
            }
            
            tv = 0.0; rv = 0.0;

			msg.translationalVelocity = tv;
			msg.rotationalVelocity = rv;
            System.out.println("TV: " + tv + ", RV: " + rv);
			motionPub.publish(msg);
		}
	}

	/**
	 * <p>
	 * Run the VisualServo process
	 * </p>
	 * 
	 * @param node optional command-line argument containing hostname
	 */
	@Override
	public void onStart(Node node) {
		cct = new ConnComp();

		motionPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");

		// Begin Student Code

		// set parameters on blobTrack as you desire


		// initialize the ROS publication to command/Motors


		// End Student Code


		final boolean reverseRGB = node.newParameterTree().getBoolean("reverse_rgb", true);

		vidSub = node.newSubscriber("/rss/video", "sensor_msgs/Image");
		vidSub
		.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.Image>() {
			@Override
			public void onNewMessage(
					org.ros.message.sensor_msgs.Image message) {
				byte[] rgbData;
				if (reverseRGB) {
					rgbData = Image.RGB2BGR(message.data,
							(int) message.width, (int) message.height);
				}
				else {
					rgbData = message.data;
				}
				assert ((int) message.width == width);
				assert ((int) message.height == height);
				handle(rgbData);
			}
		});

		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		odoSub
		.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(
					org.ros.message.rss_msgs.OdometryMsg message) {
				if (firstUpdate) {
					firstUpdate = false;
					gui.resetWorldToView(message.x, message.y);
				}
				gui.setRobotPose(message.x, message.y, message.theta);
			}
		});
		Thread runningStuff = new Thread(this);
		runningStuff.start();
	}

	@Override
	public void onShutdown(Node node) {
		if (node != null) {
			node.shutdown();
		}
	}

	@Override
	public void onShutdownComplete(Node node) {
	}

	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("rss/visualservo");
	}
}
