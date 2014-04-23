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

import cameraProcessing.FiducialFinder.FidPattern;

public class cameraProcessor implements NodeMain, Runnable {

	private static final int width = 160*4;
	private static final int height = 120*4;
	
	private ArrayBlockingQueue<byte[]> visionImage = new ArrayBlockingQueue<byte[]>(1);
	protected boolean firstUpdate = true;

	public Subscriber<org.ros.message.sensor_msgs.Image> vidSub;
	public Publisher<MotionMsg> motionPub;
	
	private ConnComp cct;
	private FiducialFinder fidFind;
	
	private Image src;
	public boolean guiOn = true;
	private boolean processBool = true;
	private boolean visualServoBool = true;
	private VisionGUI gui;
	
	private List<BlockInfo> binfos;
	private List<FidPattern> finfos;
	
	public cameraProcessor() {
		gui = new VisionGUI();
	}
	
	public void setProcessing (boolean toggle) {
		this.processBool = toggle;
	}
	
	public void takeOverDriving (boolean toggle) {
		this.visualServoBool = toggle;
	}
	
	public boolean isProcessing() {
		return this.processBool;
	}
	
	public boolean isDriving() {
		return this.visualServoBool;
	}
	/**
	 * Should be called in the run command if you want to process data
	 */
	private void processData() {
		try {
			src = new Image(visionImage.take(), width, height);
		} catch (InterruptedException e) {
			e.printStackTrace();
			return;
		}
		Image dest = new Image(src);
		
		// process here
		if (guiOn){
			dest = new Image(src);
			binfos = cct.visualize(src, dest);
		} else {
			binfos = cct.getBlockInfosForFrame(src);
		}
		finfos = fidFind.findFids(binfos);
			
		cct.calibrateHelp(src, dest);
//		cct.debugHelp(src, dest);
		
		if (guiOn) {
			gui.setVisionImage(dest.toArray(), width, height);
		}
	}
	
	
	public double RANGE_GAIN = .005;
    public double BEARING_GAIN = .005;
    public double desiredRange = 441; // desired centroid.y in pixels
    public double desiredBearing = 391; //desired centroid.x in pixels
	/**
	 * Should be called in the run command if you want the robot to servo
	 * to the closest object.
	 */
	private void doViusalServo() {
		// publish velocity messages to move the robot towards the target
        MotionMsg msg = new MotionMsg();
        double tv = 0.0;
        double rv = 0.0;

        if (!binfos.isEmpty()) {
        	BlockInfo b = binfos.get(0);
            // Very basic proportional controller
        	System.out.println("size : " + b.size + " | cx : " + b.centroid.x + " | cy : " + b.centroid.y);
            double rangeError = desiredRange - b.centroid.y; // negative because y pixels increase downward
            double bearingError = desiredBearing - b.centroid.x;
            tv = rangeError*RANGE_GAIN;
            rv = bearingError*BEARING_GAIN;
        }

		msg.translationalVelocity = tv;
		msg.rotationalVelocity = rv;
        System.out.println("TV: " + tv + ", RV: " + rv);
		motionPub.publish(msg);
	}
	
	@Override
	public void run() {
		while (true) {
			if (this.processBool)
				processData();
			if (this.visualServoBool)
				doViusalServo();
		}
	}

	@Override
	public void onStart(Node node) {
		System.out.println("Starting camera processor");
		cct = new ConnComp();
		fidFind = new FiducialFinder();
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
		
		motionPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
		
		Thread runThis = new Thread(this);
		runThis.start();
	}
	
	public void handle(byte[] rawImage) {
		visionImage.offer(rawImage);
	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}	
	
	@Override
	public void onShutdown(Node node) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onShutdownComplete(Node node) {
		// TODO Auto-generated method stub
		
	}

}
