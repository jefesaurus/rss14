package kinect;

import java.awt.Color;
import java.util.ArrayList;

import navigation.IntTuple;
import navigation.Point3D;
import navigation.Pose3D;
import navigation.World;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.sensor_msgs.PointCloud2;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import collectBlocks.Image;

import java.awt.geom.Point2D;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.HashMap; 
import java.util.Map; 
import java.util.Iterator;
import java.awt.Point;

public class KinectData implements NodeMain {
	public Subscriber<org.ros.message.sensor_msgs.PointCloud2> kinSub;
	public boolean firstUpdate;
	private int[][][] rgb;
	private boolean[][] wallMask;
	private double[][] xyz;
	public Pose pose;
	public Pose3D kinectPose;
	HashMap<IntTuple, double[]> occupancy;
	public Image rep;

	int START_COL = 0;
	int END_COL = 640;
	int START_ROW = 0;
	int END_ROW = 480;
	private int divideScale;
	private final int rawWidth = 640;
	private final int width = END_COL - START_COL;
	private final int rawHeight = 480;
	private final int height = END_ROW - START_ROW;
	private boolean newData = false;

	private World world;

	public KinectData(World world, int divideScale) {
		System.out.println("Kinect node constructed");
		this.world = world;
		this.divideScale = divideScale;
		rgb = new int[width/divideScale][height/divideScale][3];
		xyz = new double[width/divideScale][height/divideScale];
		wallMask = new boolean[width/divideScale][height/divideScale];
		rep = new Image(width/divideScale, height/divideScale);

		this.kinectPose = new Pose3D(new Point3D(0, 0.67, 0.), Math.PI/2., -Math.PI/2. - .495, 0.);;

		this.occupancy = new HashMap<IntTuple, double[]>();
	}

	public int getWidth() {
		return rawWidth/divideScale;
	}

	public int getHeight() {
		return rawHeight/divideScale;
	}

	public int[] getColorPixel(int x, int y) {
		return rgb[x][y];
	}

	public Image getImage() {
		Image answer = new Image(this.getWidth(), this.getHeight());
		for (int x = 0; x < this.getWidth(); x++) {
			for (int y = 0; y < this.getHeight(); y++) {
				answer.setPixel(x, y, (byte)rgb[x][y][0], (byte)rgb[x][y][1], (byte)rgb[x][y][2]);
			}
		}
		return answer;
	}

	/**
	 * Return null if this method has been called already without the rgb array changing
	 * @return
	 */
	public Image getNewImage() {
		if (newData) {
			newData = false;
			return getImage();
		}
		return null;
	}

	public int[][][] getRGBArray() {
		synchronized (rgb) {
			return rgb;
		}
	}

	public int[][][] getNewRGBArray() {
		if (newData) {
			newData = false;
			return getRGBArray();
		}
		return null;
	}

	public boolean[][] getWallMask() {
		return wallMask;
	}

	public boolean[][] getBlockMask() {
		boolean[][] mask = new boolean[this.getWidth()][this.getHeight()];
		for (int x = 0; x < this.getWidth(); x++) {
			for (int y = 0; y < this.getHeight(); y++) {
				//TODO: Decide what is a block and what is not
				if (y > this.getHeight()/2){
					mask[x][y] = true;
				} else {
					mask[x][y] = false;
				}

			}
		}
		return mask;
	}

	@Override
	public void onStart(Node node) {
		System.out.println("KINECT NODE STARTED");
		kinSub = node.newSubscriber("/camera/depth_registered/points", "sensor_msgs/PointCloud2");
		kinSub.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.PointCloud2>() {
			@Override
			public synchronized void onNewMessage(org.ros.message.sensor_msgs.PointCloud2 message) {
				unpackPointCloudData((int)message.width, (int)message.height, (int)message.point_step, (int)message.row_step, message.data);
			}
		});
	}

	int X_OFFSET = 0;
	int Y_OFFSET = 4;
	int Z_OFFSET = 8;
	int R_OFFSET = 16;
	int G_OFFSET = 17;
	int B_OFFSET = 18;
	float OCCUPANCY_RESOLUTION = .02f;
	int OCCUPANCY_THRESHOLD = 3;
	private long lastTimeProcessed = -1;
	private int fps = 10;

	public void unpackPointCloudData(int width, int height, int pointStep, int rowStep, byte[] data) {
		if (lastTimeProcessed < 0)
			lastTimeProcessed = System.nanoTime();
		else {
			long elapsedMilli = (System.nanoTime() - lastTimeProcessed)/1000000;
			if (elapsedMilli > 1000/fps) {
				lastTimeProcessed = System.nanoTime();
			} else {
				return;
			}
		}
//			System.out.println("Kinect message received: " + System.nanoTime()/1000000);
		int offset, x_i, y_i, z_i, r_i, g_i, b_i;
		float x, y, z;
		int r, g, b;
		Point3D point;

		synchronized (rgb) {
			int stepSize = this.divideScale;
			for (int row = START_ROW; row < END_ROW; row += stepSize) {
				for (int col = START_COL; col < END_COL; col += stepSize) {
					offset = rowStep*row + pointStep*col;
					x_i = offset+X_OFFSET;
					y_i = offset+Y_OFFSET;
					z_i = offset+Z_OFFSET;
					r_i = offset+R_OFFSET;
					g_i = offset+G_OFFSET;
					b_i = offset+B_OFFSET;
					x = Float.intBitsToFloat((data[x_i+3] & 0xff) << 24 | (data[x_i+2] & 0xff) << 16 | (data[x_i+1] & 0xff) << 8 | (data[x_i] & 0xff)); 
					y = Float.intBitsToFloat((data[y_i+3] & 0xff) << 24 | (data[y_i+2] & 0xff) << 16 | (data[y_i+1] & 0xff) << 8 | (data[y_i] & 0xff)); 
					z = Float.intBitsToFloat((data[z_i+3] & 0xff) << 24 | (data[z_i+2] & 0xff) << 16 | (data[z_i+1] & 0xff) << 8 | (data[z_i] & 0xff)); 
					r = (data[r_i] & 0xff);
					g = (data[g_i] & 0xff);
					b = (data[b_i] & 0xff);
					float[] hsv = new float[3];

					// fill in rgb
					rgb[(col-START_COL)/divideScale][(row-START_ROW)/divideScale][2] = r;
					rgb[(col-START_COL)/divideScale][(row-START_ROW)/divideScale][1] = g;
					rgb[(col-START_COL)/divideScale][(row-START_ROW)/divideScale][0] = b;

					if (!Float.isNaN(x) && !Float.isNaN(y) && !Float.isNaN(z)) {
						point = kinectPose.fromFrame(new Point3D(x, y, z));
						if (point.z > 0.0) {
							IntTuple loc = new IntTuple((int)((point.x-.58)/OCCUPANCY_RESOLUTION), (int)(point.y/OCCUPANCY_RESOLUTION));
							double[] point_data = occupancy.get(loc);
							boolean isWall = false;
							if (point.z > .08) {
								isWall = true;
							} else {
								Color.RGBtoHSB(r, g, b, hsv);
								if (hsv[1] < .4) {
									isWall = true;
								}
							}
							if (isWall) {
								if (point_data == null) {
									occupancy.put(loc, new double[] {1,point.z,0,0,0});
								} else {
									point_data[0] ++;
									point_data[1] += point.z;
								}
							}
						}
						if (point.z > 0.15) {
							wallMask[(col-START_COL)/divideScale][(row-START_ROW)/divideScale] = true;
						} else {
							wallMask[(col-START_COL)/divideScale][(row-START_ROW)/divideScale] = false;
						}
						xyz[(col-START_COL)/divideScale][(row-START_ROW)/divideScale] = z;

					}
				}
			}
//			world.updateOccupancy(occupancy);
			newData = true;
		}
	}



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
		return new GraphName("rss/kinect");
	}
}
