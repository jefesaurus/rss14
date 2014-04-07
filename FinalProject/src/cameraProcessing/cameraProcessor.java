package cameraProcessing;

import java.util.concurrent.ArrayBlockingQueue;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class cameraProcessor implements NodeMain, Runnable {

	private static final int width = 160*4;
	private static final int height = 120*4;
	
	private ArrayBlockingQueue<byte[]> visionImage = new ArrayBlockingQueue<byte[]>(1);
	protected boolean firstUpdate = true;

	public Subscriber<org.ros.message.sensor_msgs.Image> vidSub;
	
	private ConnCompTwo cct;
	private Image src;
	private VisionGUI gui;
	
	public float centroidX;
	public float centroidY;
	public float area;
	
	public cameraProcessor() {
		gui = new VisionGUI();
		
	}
	
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
			
			// process here
			cct.visualize(src, dest);
			
			gui.setVisionImage(dest.toArray(), width, height);
		}
		
	}

	@Override
	public void onStart(Node node) {
		System.out.println("Starting camera processor");
		cct = new ConnCompTwo();
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
