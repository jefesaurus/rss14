package kinect;

import java.awt.Color;
import java.util.ArrayList;

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
	public int[][][] rgb;
	public float[][] xyz;
	public Pose pose;
	public Pose3D kinectPose;
	
	private int divideScale;
	private final int rawWidth = 640;
	private final int rawHeight = 480;
	private boolean newData = false;

	public KinectData(int divideScale) {
		System.out.println("Kinect node constructed");
		this.divideScale = divideScale;
		rgb = new int[rawWidth/divideScale][rawHeight/divideScale][3];
		xyz = new float[rawWidth/divideScale][rawHeight/divideScale];

	    kinectPose = new Pose3D(new Point3D(0., 0., 0.3), Math.PI/2., -Math.PI/2., 0.);
	
	    occupancy = new HashMap<Point2D.Float, Integer>();
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

  //<fffxxxxBBB
  //f = 4
  //x = 1
  //B = 1
  /*
  int START_COL = 640/3;
  int END_COL = 2*640/3;
  int START_ROW = 480/3;
  int END_ROW = 2*480/3;
  */
  int START_COL = 0;
  int END_COL = 640;
  int START_ROW = 190;
  int END_ROW = 290;
  /*
  int START_COL = 220;
  int END_COL = 420;
  int START_ROW = 180;
  int END_ROW = 300;
  */

  private long lastTimeProcessed = -1;
  private int fps = 5;
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
	int n = 1;
    int total = 1;
    float x_avg = 0.0f;
    float y_avg = 0.0f;
    float z_avg = 0.0f;
    float ryavg = 0.0f;
    ArrayList<Point3D> obstaclePoints = new ArrayList<Point3D>();
    float red = 0.0f;
    float blue = 0.0f;
    float green = 0.0f;
    
    int stepSize = this.divideScale;
    for (int row = 0; row < this.rawHeight; row += stepSize) {
      for (int col = 0; col < this.rawWidth; col += stepSize) {
        int offset = rowStep*row + pointStep*col;
        int x_i = offset+X_OFFSET;
        int y_i = offset+Y_OFFSET;
        int z_i = offset+Z_OFFSET;
        int r_i = offset+R_OFFSET;
        int g_i = offset+G_OFFSET;
        int b_i = offset+B_OFFSET;
        float x = Float.intBitsToFloat((data[x_i+3] & 0xff) << 24 | (data[x_i+2] & 0xff) << 16 | (data[x_i+1] & 0xff) << 8 | (data[x_i] & 0xff)); 
        float y = Float.intBitsToFloat((data[y_i+3] & 0xff) << 24 | (data[y_i+2] & 0xff) << 16 | (data[y_i+1] & 0xff) << 8 | (data[y_i] & 0xff)); 
        float z = Float.intBitsToFloat((data[z_i+3] & 0xff) << 24 | (data[z_i+2] & 0xff) << 16 | (data[z_i+1] & 0xff) << 8 | (data[z_i] & 0xff)); 
        int r = (data[r_i] & 0xff); 
        int g = (data[g_i] & 0xff); 
        int b = (data[b_i] & 0xff); 
        this.rgb[col/divideScale][row/divideScale][2] = r;
        this.rgb[col/divideScale][row/divideScale][1] = g;
        this.rgb[col/divideScale][row/divideScale][0] = b;
        
        this.xyz[col/divideScale][row/divideScale] = z;

        if (!Float.isNaN(x) && !Float.isNaN(y) && !Float.isNaN(z)) {
          Point3D realPoint = kinectPose.fromFrame(new Point3D(x, y, z));
          if (realPoint.z > 0) {
            obstaclePoints.add(realPoint); 
          } else {
            n++;
          }
          total ++;
          x_avg += realPoint.x;
          y_avg += realPoint.y;
          z_avg += realPoint.z;

          //red += r;
          //blue += b;
          //green += g;
        }
      }
    }
    newData = true;
  }

  HashMap<Point2D.Float, Integer> occupancy;


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
