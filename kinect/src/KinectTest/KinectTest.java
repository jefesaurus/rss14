package KinectTest;

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

public class KinectTest implements NodeMain, Runnable {
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	public Subscriber<org.ros.message.sensor_msgs.PointCloud2> kinSub;
  public KinectGUI gui;
  public boolean firstUpdate;
  public Pose pose;
  public Pose3D kinectPose;

	public KinectTest() {
		System.out.println("Constructed GUI");

		gui = new KinectGUI();
	  gui.setRobotPose(0., 0., 0.);
    kinectPose = new Pose3D(new Point3D(0.0, 0.3, 0.0), Math.PI/2., -Math.PI/2. - Math.PI/90., 0.);

    occupancy = new HashMap<Point2D.Float, Integer>();
	}

	@Override
	public void run() {
		System.out.println("Starting RUN!");
		//while (true) {
		//}
	}
	
	@Override
	public void onStart(Node node) {
		System.out.println("NODE STARTED");

    /*
		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		odoSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
				if (firstUpdate) {
					firstUpdate = false;
					gui.resetWorldToView(message.x, message.y);
				}
				gui.setRobotPose(message.x, message.y, message.theta);
				pose = new Pose(message.x, message.y, message.theta);
			}
		});

    */
		kinSub = node.newSubscriber("/camera/depth_registered/points", "sensor_msgs/PointCloud2");
		kinSub.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.PointCloud2>() {
			@Override
			public synchronized void onNewMessage(org.ros.message.sensor_msgs.PointCloud2 message) {
        unpackPointCloudData((int)message.width, (int)message.height, (int)message.point_step, (int)message.row_step, message.data);
			}
		});


		Thread runningStuff = new Thread(this);
		runningStuff.start();
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
  int START_COL = 210;
  int END_COL = 430;
  int START_ROW = 200;
  int END_ROW = 480;
  /*
  int START_COL = 220;
  int END_COL = 420;
  int START_ROW = 180;
  int END_ROW = 300;
  */

  public void unpackPointCloudData(int width, int height, int pointStep, int rowStep, byte[] data) {
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

    for (int row = START_ROW; row < END_ROW; row ++) {
      for (int col = START_COL; col < END_COL; col ++) {
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
        //int r = (data[r_i] & 0xff); 
        //int g = (data[g_i] & 0xff); 
        //int b = (data[b_i] & 0xff); 



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

    //Point3D p = new Point3D(x_avg/total, y_avg/total, z_avg/total);
    //System.out.println("Avg point: " + p + " percent filtered: " + n/(double)total);
    //System.out.println("R: " + red/total + " G: " + green/total + " B: " + blue/total);
    updateGUI(obstaclePoints);
  }

  HashMap<Point2D.Float, Integer> occupancy;

  float OCCUPANCY_RESOLUTION = .02f;
  int OCCUPANCY_THRESHOLD = 3;
  public void updateGUI(ArrayList<Point3D> obstaclePoints) {
    int npoints = 0;
    for (Point3D point : obstaclePoints) {
      float x = ((int)(point.x/OCCUPANCY_RESOLUTION))*OCCUPANCY_RESOLUTION;
      float y = ((int)(point.y/OCCUPANCY_RESOLUTION))*OCCUPANCY_RESOLUTION;
      Point2D.Float cell = new Point2D.Float(x, y);
      Integer count = occupancy.get(cell);
      npoints ++;
      if (count == null) {
        occupancy.put(cell, 1);
      } else {
        occupancy.put(cell, ++count);
      }
    }
    gui.erasePoints();
    //System.out.println("NUMS SQUARES: " + occupancy.entrySet().size() + " NUM POINTS: " + npoints);
    for (Map.Entry<Point2D.Float, Integer> cell : occupancy.entrySet()) {
      if (cell.getValue() > OCCUPANCY_THRESHOLD) {
        Point2D.Float point = cell.getKey();
        //System.out.println("DRAWING POINT: " + point.x + ", " + point.y);
        gui.addPoint(point.x, point.y, 1);
      }
    }
    occupancy.clear();
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
