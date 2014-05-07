package collectBlocks;

import java.awt.Point;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;

import kinect.KinectData;

import master.DrivingMaster;
import master.GatesController;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import collectBlocks.FiducialFinder.FidPattern;

public class BlockCollector implements NodeMain, Runnable {

	private int width = 640;
	private int height = 480;
	
	private ArrayBlockingQueue<byte[]> visionImage = new ArrayBlockingQueue<byte[]>(1);
	protected boolean firstUpdate = true;

	public Subscriber<org.ros.message.sensor_msgs.Image> vidSub;
	public Publisher<MotionMsg> motionPub;
	
	private ConnComp cct;
	private FiducialFinder fidFind;
	private DrivingMaster driveMaster;
	private KinectData kinecter;
	private GatesController gates;
	
	private Image srcImage;
	private int[][][] srcArray;
	public boolean guiOn = true;
	private boolean processBool = true;
	private boolean collectBool = true;
	private String state; // Looking, Found, Eating
	private VisionGUI gui;
	
	private List<BlockInfo> binfos;
	private List<FidPattern> finfos;
	
	public BlockCollector(DrivingMaster driveMaster, KinectData kinecter, GatesController gates, int divideScale) {
		width /= divideScale;
		height /= divideScale;
		if (guiOn) {
			gui = new VisionGUI();
		}
		this.driveMaster = driveMaster;
		this.kinecter = kinecter;
		this.gates = gates;
		state = "Looking";
		binfos = new ArrayList<BlockInfo>();
		finfos = new ArrayList<FidPattern>();
	}
	
	public boolean blockImminent() {
		return largestBlob().size > 90;
	}
	
	public void setProcessing (boolean toggle) {
		this.processBool = toggle;
	}
	
	public void takeOverDriving (boolean toggle) {
		this.collectBool = toggle;
		this.driveMaster.setActive(toggle);
		if (toggle) {
			gates.openFrontGate();
		} else {
			gates.closeFrontGate();
		}
	}
	
	public boolean isProcessing() {
		return this.processBool;
	}
	
	public boolean isDriving() {
		return this.collectBool;
	}
	
	public BlockInfo largestBlob() {
		if (binfos == null || binfos.isEmpty()) 
			return new BlockInfo(new Point(0,0), 0, "red");
		return binfos.get(0);
	}
	
	/**
	 * Should be called in the run command if you want to process data
	 */
	private void processData() {
		srcArray = kinecter.getNewRGBArray();
		if (srcArray == null)
			return;
		srcArray = kinecter.getRGBArray();
		boolean[][] wallMask = kinecter.getWallMask();
		boolean[][] blockMask = kinecter.getBlockMask();
//		process here
		if (guiOn){
			Image dest = kinecter.getImage();
			//blocks
			binfos = cct.visualize(srcArray, blockMask, 50, true, dest);
			//fiducials
			List<BlockInfo> fidbinfos = cct.visualize(srcArray, wallMask, 50, false, dest);
			finfos = fidFind.findFids(fidbinfos);
			
//			cct.calibrateHelp(srcArray, dest);
//			cct.debugHelp(srcArray, dest);
//			cct.maskHelp(srcArray, wallMask, dest);
			gui.setVisionImage(dest.toArray(), width, height);
		} else {
			binfos = cct.getBlockInfosForFrame(srcArray, blockMask, 50, true);
			// fiducials
			finfos = fidFind.findFids(srcArray, wallMask, 50);
		}
//		System.out.println("Kinect data processed: " + System.nanoTime()/1000000);
	}
	
	
	public double RANGE_GAIN = .005;
    public double BEARING_GAIN = 1.;
    public double desiredRange = 441/4; // desired centroid.y in pixels
    public double desiredBearing = 391/4; //desired centroid.x in pixels
    private long timer;
	/**
	 * Should be called in the run command if you want the robot to servo
	 * to the closest object.
	 */
	private void collect() {
//		System.out.println("state : " + state);
		// publish velocity messages to move the robot towards the target
        double tv = 0.0;
        double rv = 0.0;
        // driving stuff
        BlockInfo b = null;
        if (!binfos.isEmpty()) {
        	b = binfos.get(0);
        }
        if (state.equals("Looking") || state.equals("Found")) {
        	if (b != null) {
                // Very basic proportional controller
//            	System.out.println("size : " + b.size + " | cx : " + b.centroid.x + " | cy : " + b.centroid.y);
                double rangeError = desiredRange - b.centroid.y; // negative because y pixels increase downward
//                double bearingError = desiredBearing - b.centroid.x;
                double bearingError = ((double)(width/2 - b.centroid.x))/width;
                tv = .2;
                rv = bearingError*BEARING_GAIN;
            }
        } else if (state.equals("Eating")) {
        	tv = .3;
        } else {
        	System.out.println("Unknown state " + state);
        }
        
        long milli = System.nanoTime()/1000000;
        // next state decisions
        if (state.equals("Looking")) {
        	//next state is "Found" if block is within 2nd tenth from the bottom
            if (b != null && b.centroid.y > .9*height) {
            	state = "Eating";
            	timer = milli;
            	System.out.println("Changing state to Eating");
            }
        } else if (state.equals("Found")) {
        	if (b != null && b.centroid.y < .8*height) {
        		state = "Eating";
        		timer = milli;
        		System.out.println("Changing state to Eating");
        	}
        } else if (state.equals("Eating")) {
        	//2 seconds to make sure it's actually eaten the block.
        	if (Math.abs(milli - timer) > 5000) {
        		state = "Looking";
        		timer = milli;
        		System.out.println("Changing state to Looking");
        	}
        } else {
        	System.out.println("Unknown state again");
        }
//        System.out.println("tv : " +tv+" |rv : "+rv);

        driveMaster.setVelocity(tv, rv);
	}
	
	@Override
	public void run() {
		while (true) {
			if (this.processBool)
				processData();
			if (this.collectBool)
				collect();
		}
	}

	@Override
	public void onStart(Node node) {
		System.out.println("Starting block collector");
		cct = new ConnComp();
		fidFind = new FiducialFinder();
//		System.out.println("Getting parameter tree");
//		final boolean reverseRGB = node.newParameterTree().getBoolean("reverse_rgb", true);

//		System.out.println("about to video subscribe");
//		vidSub = node.newSubscriber("/rss/video", "sensor_msgs/Image");
//		System.out.println("Inserting video listener");
//		vidSub
//		.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.Image>() {
//			@Override
//			public void onNewMessage(
//					org.ros.message.sensor_msgs.Image message) {
//				System.out.println("received video frame");
//				byte[] rgbData;
//				if (reverseRGB) {
//					rgbData = Image.RGB2BGR(message.data,
//							(int) message.width, (int) message.height);
//				}
//				else {
//					rgbData = message.data;
//				}
//				assert ((int) message.width == width);
//				assert ((int) message.height == height);
//				handle(rgbData);
//			}
//		});
//		System.out.println("inserted video listener");
		
		Thread runThis = new Thread(this);
		runThis.start();
		System.out.println("running camera processing thread");
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
