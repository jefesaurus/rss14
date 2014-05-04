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

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;


public class KinectTest implements NodeMain, Runnable {
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	public Subscriber<org.ros.message.sensor_msgs.PointCloud2> kinSub;
  public KinectGUI gui;
  public boolean firstUpdate;
  public Pose pose;
  public Pose3D kinectPose;
  HashMap<IntTuple, double[]> occupancy;
  BufferedWriter bw;
  boolean WRITE_POINTS_TO_FILE = false;

	public KinectTest() {
		System.out.println("Constructed GUI");

		gui = new KinectGUI();
	  gui.setRobotPose(0., 0., 0.);
    kinectPose = new Pose3D(new Point3D(0.0, 0.67, 0.0), Math.PI/2., -Math.PI/2. - .571, 0.);

    occupancy = new HashMap<IntTuple, double[]>();

    if (WRITE_POINTS_TO_FILE) {
    try { 
      File pointFile = new File("/home/rss-student/RSS-I-group/kinect/src/KinectTest/points.txt");
      if (WRITE_POINTS_TO_FILE) {
        if (!pointFile.exists()) {
          pointFile.createNewFile();
        }
      }
      FileWriter fw = new FileWriter(pointFile.getAbsoluteFile());
      bw = new BufferedWriter(fw);
    } catch (IOException e) {
      e.printStackTrace();
    }
    }
	}

	@Override
	public void run() {
		System.out.println("Starting RUN!");
	}
	
	@Override
	public void onStart(Node node) {
		System.out.println("NODE STARTED");
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
  int START_COL = 100;
  int END_COL = 540;
  int START_ROW = 0;
  int END_ROW = 480;

  float OCCUPANCY_RESOLUTION = .02f;
  int OCCUPANCY_THRESHOLD = 3;
  public void unpackPointCloudData(int width, int height, int pointStep, int rowStep, byte[] data) {
    int offset, x_i, y_i, z_i, r_i, g_i, b_i;
    float x, y, z;
    int r, g, b;
    Point3D point;
    for (int row = START_ROW; row < END_ROW; row ++) {
      for (int col = START_COL; col < END_COL; col ++) {
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
        if (!Float.isNaN(x) && !Float.isNaN(y) && !Float.isNaN(z)) {
          point = kinectPose.fromFrame(new Point3D(x, y, z));
          if (point.z > 0) {
            IntTuple loc = new IntTuple((int)(point.x/OCCUPANCY_RESOLUTION), (int)(point.y/OCCUPANCY_RESOLUTION));
            double[] point_data = occupancy.get(loc);
            if (point_data == null) {
              occupancy.put(loc, new double[] {1,point.z,r,g,b});
            } else {
              point_data[0] ++;
              point_data[1] += point.z;
              point_data[2] += r;
              point_data[3] += g;
              point_data[4] += b;
            }
          }
        }
      }
    }
    gui.erasePoints();
    for (Map.Entry<IntTuple, double[]> cell : occupancy.entrySet()) {
      double[] point_data = cell.getValue();
      double num_points = point_data[0];
      if (point_data[0] > OCCUPANCY_THRESHOLD) {
        IntTuple loc = cell.getKey();
        int red = (int)(point_data[2]/num_points);
        int green = (int)(point_data[3]/num_points);
        int blue = (int)(point_data[4]/num_points);
        float[] hsv = new float[3];
        Color.RGBtoHSB(red,green,blue,hsv);
        //System.out.println(point_data[1]);


        if (point_data[1]/num_points > .05) {
          gui.addPoint(loc.x*OCCUPANCY_RESOLUTION, loc.y*OCCUPANCY_RESOLUTION, 1, Color.BLACK);
        } else {
          if (checkYellow(hsv[0], hsv[1], hsv[2])) {
            gui.addPoint(loc.x*OCCUPANCY_RESOLUTION, loc.y*OCCUPANCY_RESOLUTION, 1, Color.YELLOW);
          } else if (checkRed(hsv[0], hsv[1], hsv[2])) {
            gui.addPoint(loc.x*OCCUPANCY_RESOLUTION, loc.y*OCCUPANCY_RESOLUTION, 1, Color.RED);
          } else if (checkGreen(hsv[0], hsv[1], hsv[2])) {
            gui.addPoint(loc.x*OCCUPANCY_RESOLUTION, loc.y*OCCUPANCY_RESOLUTION, 1, Color.GREEN);
          } else if (checkBlue(hsv[0], hsv[1], hsv[2])) {
            gui.addPoint(loc.x*OCCUPANCY_RESOLUTION, loc.y*OCCUPANCY_RESOLUTION, 1, Color.BLUE);
          } else if (checkOrange(hsv[0], hsv[1], hsv[2])) {
            gui.addPoint(loc.x*OCCUPANCY_RESOLUTION, loc.y*OCCUPANCY_RESOLUTION, 1, Color.ORANGE);
          }
        }
      }
    }
    occupancy.clear();
  }

  //greenRange = new HSBRanges(.35f, .45f, .6f, 1.f, .3f, 1.f);
  public boolean checkGreen(float hue, float sat, float bright) {
    float hueMin = .35f; float hueMax = .45f;
    float satMin = .6f;float satMax = 1.f;
    float brightMin = .3f;float brightMax = 1.f;
    if (hue < hueMin || hue > hueMax) return false;
    if (sat < satMin || sat > satMax) return false;
    if (bright < brightMin || bright > brightMax) return false;
    return true;
  }

  //yellowRange = new HSBRanges(.1f, .2f, .4f, 1.f, .4f, 1.f);
  public boolean checkYellow(float hue, float sat, float bright) {
    float hueMin = .1f;float hueMax = .2f;
    float satMin = .4f;float satMax = 1.f;
    float brightMin = .4f;float brightMax = 1.f;
    if (hue < hueMin || hue > hueMax) return false;
    if (sat < satMin || sat > satMax) return false;
    if (bright < brightMin || bright > brightMax) return false;
    return true;
  }

  //redRange = new HSBRanges(.0f, .05f, .5f, 1.f, .5f, 1.f);
  public boolean checkRed(float hue, float sat, float bright) {
    float hueMin = .0f;float hueMax = .05f;
    float satMin = .5f;float satMax = 1.f;
    float brightMin = .5f;float brightMax = 1.f;
    if (hue < hueMin || hue > hueMax) return false;
    if (sat < satMin || sat > satMax) return false;
    if (bright < brightMin || bright > brightMax) return false;
    return true;
  }

  //orangeRange = new HSBRanges(.05f, .08f, .5f, 1.f, .4f, 1.f);
  public boolean checkOrange(float hue, float sat, float bright) {
    float hueMin = .05f; float hueMax = .08f;
    float satMin = .5f;float satMax = 1.f;
    float brightMin = .4f;float brightMax = 1.f;
    if (hue < hueMin || hue > hueMax) return false;
    if (sat < satMin || sat > satMax) return false;
    if (bright < brightMin || bright > brightMax) return false;
    return true;
  }


  //blueRange = new HSBRanges(.5f, .7f, .4f, 1.f, .2f, .4f);
  public boolean checkBlue(float hue, float sat, float bright) {
    float hueMin = .5f; float hueMax = .7f;
    float satMin = .4f; float satMax = 1.f;
    float brightMin = .2f; float brightMax = .4f;
    if (hue < hueMin || hue > hueMax) return false;
    if (sat < satMin || sat > satMax) return false;
    if (bright < brightMin || bright > brightMax) return false;
    return true;
  }

	public void onShutdown(Node node) {
    if (WRITE_POINTS_TO_FILE) {try {bw.close();} catch (IOException e) {e.printStackTrace();}}
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
