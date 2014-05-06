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
  public VisionGUI vgui;
  public boolean firstUpdate;
  public Pose pose;
  public Pose3D kinectPose;
  HashMap<IntTuple, double[]> occupancy;

  public boolean POINT_GUI = false;
  public boolean VISION_GUI = true;

	public KinectTest() {
		System.out.println("Constructed GUI");

    if (POINT_GUI) {
		  gui = new KinectGUI();
	    gui.setRobotPose(0., 0., 0.);
    }
    if (VISION_GUI) {
		  vgui = new VisionGUI();
    }
    kinectPose = new Pose3D(new Point3D(0.0, 0.67, 0.0), Math.PI/2., -Math.PI/2. - .571, 0.);
    //kinectPose = new Pose3D(new Point3D(0.0, 0.67, 0.0), Math.PI/2., -Math.PI/2. - .34, 0.);

    occupancy = new HashMap<IntTuple, double[]>();

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
  int R_OFFSET = 18;
  int G_OFFSET = 17;
  int B_OFFSET = 16;

  //<fffxxxxBBB
  //f = 4
  //x = 1
  //B = 1
  int START_COL = 100;
  int END_COL = 540;
  int START_ROW = 140;
  int END_ROW = 480;
  /*
  int START_COL = 200;
  int END_COL = 440;
  int START_ROW = 100;
  int END_ROW = 380;
  */


  float OCCUPANCY_RESOLUTION = .05f;
  int OCCUPANCY_THRESHOLD = 3;
  public void unpackPointCloudData(int width, int height, int pointStep, int rowStep, byte[] data) {
    int offset, x_i, y_i, z_i, r_i, g_i, b_i;
    float x, y, z;
    int r, g, b;
    Point3D point;
    double avg_h = 0;
    double avg_s = 0;
    double avg_v = 0;
    int pic_height = this.END_ROW - this.START_ROW;
    int pic_width = this.END_COL - this.START_COL;
    Image rep = new Image(pic_width, pic_height);
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
        float[] hsv = new float[3];
        if (!Float.isNaN(x) && !Float.isNaN(y) && !Float.isNaN(z)) {
          point = kinectPose.fromFrame(new Point3D(x, y, z));
          if (point.z > 0.0) {
            IntTuple loc = new IntTuple((int)(point.x/OCCUPANCY_RESOLUTION), (int)(point.y/OCCUPANCY_RESOLUTION));
            double[] point_data = occupancy.get(loc);
            Color.RGBtoHSB(r,g,b,hsv);
            //System.out.println("r: " + r + " g: " +g + " b: " + b);
            //System.out.println("h: " + hsv[0] + " s: " + hsv[1] + " v: " + hsv[2]);
            rep.setPixel(col - START_COL, row - START_ROW, data[r_i], data[g_i], data[b_i]);
            if (point_data == null) {
              if (checkRed(hsv[0], hsv[1], hsv[2])) {
                occupancy.put(loc, new double[] {1,point.z,0, 1,0});
              } else if (checkGreen(hsv[0], hsv[1], hsv[2])) {
                occupancy.put(loc, new double[] {1,point.z,0,0, 1});
              } else if (checkYellow(hsv[0], hsv[1], hsv[2])) {
                occupancy.put(loc, new double[] {1,point.z, 1,0,0});
              } else {
                occupancy.put(loc, new double[] {1,point.z,0,0,0});
              }
            } else {
              point_data[0] ++;
              point_data[1] += point.z;
              if (checkRed(hsv[0], hsv[1], hsv[2])) {
                point_data[3] ++;
              } else if (checkGreen(hsv[0], hsv[1], hsv[2])) {
                point_data[4] ++;
              } else if (checkYellow(hsv[0], hsv[1], hsv[2])) {
                point_data[2] ++;
              }
            }
          }
        }
      }
    }
    if (VISION_GUI) {
      vgui.setVisionImage(rep.toArray(), pic_width, pic_height);
    }
    if (POINT_GUI) {
      gui.erasePoints();
    }
    for (Map.Entry<IntTuple, double[]> cell : occupancy.entrySet()) {
      double[] point_data = cell.getValue();
      double num_points = point_data[0];
      if (point_data[0] > OCCUPANCY_THRESHOLD) {
        if (POINT_GUI) {
          IntTuple loc = cell.getKey();
          int yellow = (int)point_data[2];
          int red = (int)point_data[3];
          int green = (int)point_data[4];
          //System.out.println("r: " + red + " g: " + green + " y: " + yellow);
          if (point_data[1]/num_points > .05) {
            gui.addPoint(loc.x*OCCUPANCY_RESOLUTION, loc.y*OCCUPANCY_RESOLUTION, 1, Color.BLACK);
          } else {
            if (red > green) {
              if (red > yellow) {
                gui.addPoint(loc.x*OCCUPANCY_RESOLUTION, loc.y*OCCUPANCY_RESOLUTION, 1, Color.RED);
              } else {
                gui.addPoint(loc.x*OCCUPANCY_RESOLUTION, loc.y*OCCUPANCY_RESOLUTION, 1, Color.YELLOW);
              }
            } else {
              if (green > yellow) {
                gui.addPoint(loc.x*OCCUPANCY_RESOLUTION, loc.y*OCCUPANCY_RESOLUTION, 1, Color.GREEN);
              } else {
                gui.addPoint(loc.x*OCCUPANCY_RESOLUTION, loc.y*OCCUPANCY_RESOLUTION, 1, Color.YELLOW);
              }
            }
          }
        }
      }
    }
    occupancy.clear();
  }

  //greenRange = new HSBRanges(.35f, .45f, .6f, 1.f, .3f, 1.f);
  public boolean checkGreen(float hue, float sat, float bright) {
    //if (sat < .6f) return false;
    //if (bright < .3f || bright > 1.f) return false;
    return (hue > .2f && hue < .45f);
  }

  //yellowRange = new HSBRanges(.1f, .2f, .4f, 1.f, .4f, 1.f);
  public boolean checkYellow(float hue, float sat, float bright) {
    //if (sat < .5f) return false;
    //if (bright < .4f) return false;
    return (hue > .13f && hue < .19f);
  }

  //redRange = new HSBRanges(.0f, .05f, .5f, 1.f, .5f, 1.f);
  //349
  public boolean checkRed(float hue, float sat, float bright) {
    //if (sat < .5f) return false;
    //if (bright < .5f) return false;
    return (hue > .90f || hue < .1f);
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
