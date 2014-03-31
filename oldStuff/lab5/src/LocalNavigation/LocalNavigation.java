package LocalNavigation;

import java.awt.Color;
import java.util.ArrayList;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
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

public class LocalNavigation implements NodeMain, Runnable {

	public Subscriber<org.ros.message.rss_msgs.BumpMsg> bumperSub;
	public Subscriber<org.ros.message.rss_msgs.SonarMsg> frontSonarSub,
			backSonarSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	public Publisher<MotionMsg> motionPub;
	public Publisher<org.ros.message.std_msgs.String> statePub;

	// TODO do all of this with enums
	protected final String STOP_ON_BUMP = "STOP_ON_BUMP";
	protected final String STOPPED = "STOPPED";
	protected final String ALIGN_ON_BUMP = "ALIGN_ON_BUMP";
	protected final String ALIGNING = "ALIGNING";
	protected final String ALIGNED = "ALIGNED";
	protected final String ALIGNED_AND_ROTATED = "ALIGNED_AND_ROTATED";
	protected final String BACKING_UP = "BACKING_UP";
	protected final String FINDING_WALL = "FINDING_WALL";
	protected final String TRACKING_WALL = "TRACKING_WALL";
	protected final String WALL_ENDED = "WALL_ENDED";
	
	protected final double parallelK = .5;
	protected final double distanceK = .5;
	protected final double desiredDist = .4;
	
	public boolean saveErrors = true;
	protected BufferedWriter errorWriter;
	protected File errorFile;

	public final double y_left = 0.1905;
	public final double x_front = 0.0508;
	public final double x_back = -0.3302;

	protected final double objectThreshold = .66;

	// Start State
	protected String state = "ALIGN_ON_BUMP";

	protected boolean leftBumper, rightBumper;
	protected double frontRange, backRange;
	protected Pose pose = new Pose(0,0,0);

	protected boolean firstUpdate = true;

	/*
	 * LINE estimation stuff List of points along the current line
	 */
	// Estimated parameters
	double line_a = 1.0;
	double line_b = 1.0;
	double line_c = -1.0;
	
	double xSum = 0.0;
	double ySum = 0.0;
	double xSqrSum = 0.0;
	double ySqrSum = 0.0;
	double productSum = 0.0;
	
	Pose desiredPose = null;
	Pose wallPose = null;
	Pose wallStart = null;
	
	// Past Wall estimations
	ArrayList<Double> as = new ArrayList<Double>();
	ArrayList<Double> bs = new ArrayList<Double>();
	ArrayList<Double> cs = new ArrayList<Double>();


	
	
	private SonarGUI gui;

	public LocalNavigation() {
		System.out.println("Constructed GUI");
		gui = new SonarGUI();
		// Drawing the rectangle
		gui.addSegment(.5, .5, 1.5, .5, Color.black);
		gui.addSegment(1.5, .5, 1.5, 1.5, Color.black);
		gui.addSegment(.5, 1.5, 1.5, 1.5, Color.black);
		gui.addSegment(.5, .5, .5, 1.5, Color.black);
	}

	public synchronized void updateLineEstimate(Point2D.Double newPoint) {
		xSum += newPoint.x;
		ySum += newPoint.y;
		xSqrSum += newPoint.x * newPoint.x;
		ySqrSum += newPoint.y * newPoint.y;
		productSum += newPoint.x * newPoint.y;
		
		double determinant = xSqrSum * ySqrSum - productSum * productSum;
		if (determinant < .001) {
			return;
		}

		double a = (xSum * ySqrSum - ySum * productSum)/determinant;
		double b = (ySum * xSqrSum - xSum * productSum)/determinant;
		double ln = Math.sqrt(a*a + b*b);
		
		line_a = a/ln;
		line_b = b/ln;
		line_c = -1/ln;
	}

	private double euclideanDist(Pose pose1, Pose pose2) {
		return Math.sqrt((pose1.x - pose2.x)*(pose1.x - pose2.x) + (pose1.y - pose2.y)*(pose1.y - pose2.y));
	}
	
	/**
	 * @param actualAng
	 * @param desiredAng
	 * @param modulo, typically 360 degrees or 2pi radians
	 * @return angle error between -modulo/2 to modulo/2
	 */
	private double angleError(double actualAng, double desiredAng, double modulo) {
		double diff = actualAng - desiredAng;
		diff = diff - (int)(diff/modulo)*modulo;
		if (diff > modulo/2)
			return modulo - diff;
		else if (diff < -modulo/2)
			return modulo + diff;
		else
			return diff;
	}
	
	@Override
	public void run() {
		System.out.println("Starting RUN!");
		try {
			errorFile = new File("/home/rss-student/RSS-I-group/lab5/saveErrors.txt");
			
			if (!errorFile.exists()) 
				errorFile.createNewFile();
			
			FileWriter fw = new FileWriter(errorFile.getAbsoluteFile());
			errorWriter = new BufferedWriter(fw);
			
			errorWriter.write("timestamp translation_error rotation_error");		
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		while (true) {
			String currentState = getState();
			double tv = 0.;
			double rv = 0.;
			String newState = null;
			String additionalPrinting = "";
			if (currentState == STOP_ON_BUMP) {
				tv = .5;
				rv = 0;

				if (getLeftBumper() || getRightBumper())
					newState = STOPPED;
			} else if (currentState == STOPPED) {
				tv = 0;
				rv = 0;
			} else if (currentState == ALIGN_ON_BUMP) {
				tv = .5;
				rv = 0.2;

				if (getLeftBumper() || getRightBumper())
					newState = ALIGNING;
			} else if (currentState == ALIGNING) {
				tv = .25;
				rv = 0;
				boolean left = getLeftBumper();
				boolean right = getRightBumper();
				additionalPrinting += " | leftBump: " + left + " | rightBump: " + right;

				if (!left) {
					rv -= .25;
				}
				if (!right) {
					rv += .25;
				}

				if (left && right) {
					newState = ALIGNED;
					tv = 0;
					rv = 0;
					double distanceBack = .35;
					desiredPose = new Pose(pose.x - distanceBack*Math.cos(pose.theta), pose.y - distanceBack*Math.sin(pose.theta), (pose.theta - Math.PI/2));
					wallPose = new Pose(pose.x, pose.y, pose.theta);
				}
			} else if (currentState == ALIGNED) {	
				double angError = angleError(pose.theta, desiredPose.theta, 2*Math.PI);
				additionalPrinting += " | WANT ANGLE: " + desiredPose.theta + " | AM ANGLE: " + pose.theta + " | ERROR: " + angError;
				if (euclideanDist(pose, wallPose) < .25) {
					tv = -.25;
					rv = 0;
				} else if (angError > .25) {
					tv = 0;
					rv = -.1;
				} else {
					tv = 0;
					rv = 0;
					newState = ALIGNED_AND_ROTATED;
				}
			} else if (currentState == ALIGNED_AND_ROTATED) {
				tv = 0;
				rv = 0;
				newState = BACKING_UP;

			} else if (currentState == BACKING_UP) {	
				double gainK = 0.35;
				tv = -.25;
				double frontRange = getFrontRange();
				double backRange = getBackRange();
				if (backRange < objectThreshold && frontRange < objectThreshold) {
					rv = gainK*(frontRange - backRange);
				} else {
					rv = 0.0;
				}
				
				if (frontRange > objectThreshold && backRange > objectThreshold) {
					newState = FINDING_WALL;
				}
			} else if (currentState == FINDING_WALL) {
				if (getFrontRange() > objectThreshold || getBackRange() > objectThreshold) {
					tv = 0.25;
					rv = 0;
				} else {
					wallStart = new Pose(pose.x, pose.y, pose.theta);
					newState = TRACKING_WALL;	
				}
			} else if (currentState == TRACKING_WALL) {	
				startNewLine(); // Resets the state for the line estimation
				double gainK = 0.15;
				tv = .2;
				double sonarDiff = getFrontRange() - getBackRange();
				rv = gainK*sonarDiff;
				
				/*
				double dDesired = 0.2;
				double dCurrent =  -line_c - (line_a*pose.x+line_b*pose.y);
				//double dCurrent = ;
				rv = gainK*(dCurrent - dDesired);
				
				if (saveErrors) {
					double transError = dCurrent - dDesired;
					double angleError = Math.abs(Math.atan2(line_a, line_b) - pose.theta);
					additionalPrinting += " | transError: " + Double.toString(transError) + " | angleError: " + Double.toString(angleError);
					try {
						errorWriter.write(Long.toString(System.currentTimeMillis()) + " " + Double.toString(transError) + " " + Double.toString(angleError));
						errorWriter.flush();
					} catch (IOException e) {
						e.printStackTrace();
					}
				}
				*/
				if (getFrontRange() > objectThreshold && getBackRange() > objectThreshold) {
					tv = 0;
					rv = 0;
					newState = WALL_ENDED;
					storeOldLine(); // Stash the estimated line parameters
				}
			} else if (currentState == WALL_ENDED) {
				tv = 0;
				rv = 0;
				newState = ALIGN_ON_BUMP;
			} else {
				tv = 0;
				rv = 0;
				System.out.println("Unknown state: " + currentState);
			}

			System.out.println("Current State: " + currentState + additionalPrinting + " Angle: " + currentExternalAngle(.1) + " Num lines: " + as.size());

			MotionMsg msg = new MotionMsg();
			msg.translationalVelocity = tv;
			msg.rotationalVelocity = rv;
			// System.out.println("TV: " + tv + ", RV: " + rv);
			motionPub.publish(msg);

			if (newState != null) {
				setState(newState);
				org.ros.message.std_msgs.String str = new org.ros.message.std_msgs.String();
				str.data = getState();
				statePub.publish(str);
			}
		}
	}
	
	public double currentExternalAngle(double fuzziness) {
		double totalAngle = 0.0;
		if (as.size() > 0) {
			double lastAngle = Math.atan2(as.get(0), bs.get(0));
			for (int i = 1; i < as.size(); i ++) {
				double currentAngle = Math.atan2(as.get(i), bs.get(i));
				System.out.println("Last: " + lastAngle + " Current: " + currentAngle);
				totalAngle += currentAngle - lastAngle;
				lastAngle = currentAngle;
			}
		}
		//return Math.abs(totalAngle - Math.PI*2) < fuzziness;
		return totalAngle;
	}
	
	/*
	 * Stashes the old line parameters and resets all of the line tracking state for the next one
	 */
	public void startNewLine() {

		line_a = 1.0;
		line_b = 1.0;
		line_c = -1.0;
		
		xSum = 0.0;
		ySum = 0.0;
		xSqrSum = 0.0;
		ySqrSum = 0.0;
		productSum = 0.0;
		
		desiredPose = null;
		wallPose = null;
		wallStart = null;
	}
	
	public void storeOldLine() {
		as.add(line_a);
		bs.add(line_b);
		cs.add(line_c);
	}

	@Override
	public void onStart(Node node) {
		System.out.println("yolo");

		motionPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
		statePub = node.newPublisher("/rss/state", "std_msgs/String");

		bumperSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");
		bumperSub
				.addMessageListener(new MessageListener<org.ros.message.rss_msgs.BumpMsg>() {
					@Override
					public void onNewMessage(
							org.ros.message.rss_msgs.BumpMsg message) {
						boolean left = message.left;
						boolean right = message.right;
						System.out.println("Left: " + left + " | Right: "
								+ right);
						setLeftBumper(left);
						setRightBumper(right);
					}
				});

		frontSonarSub = node.newSubscriber("/rss/Sonars/Front",
				"rss_msgs/SonarMsg");
		frontSonarSub
				.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
					@Override
					public void onNewMessage(
							org.ros.message.rss_msgs.SonarMsg message) {
						double range = (double) message.range;
						System.out.println("Front Range: " + range);
						setFrontRange(range);

						double vector_x = x_front;
						double vector_y = range + y_left;
						double mag = Math.sqrt(vector_x * vector_x + vector_y
								* vector_y);
						double angle = Math.atan2(vector_y, vector_x);
						double sensor_x = pose.x + Math.cos(pose.theta + angle)
								* mag;
						double sensor_y = pose.y + Math.sin(pose.theta + angle)
								* mag;
						
						Color color = Color.green;
						if (range < objectThreshold) {
							if (getState() == TRACKING_WALL) {
								updateLineEstimate(new Point2D.Double(sensor_x, sensor_y));
							}
							color = Color.red;
						}
						gui.addPoint(sensor_x, sensor_y, SonarGUI.O_POINT,
								color);

						gui.setLine(line_a, line_b, line_c, Color.cyan);
					}
				});

		backSonarSub = node.newSubscriber("/rss/Sonars/Back",
				"rss_msgs/SonarMsg");
		backSonarSub
				.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
					@Override
					public void onNewMessage(
							org.ros.message.rss_msgs.SonarMsg message) {
						double range = (double) message.range;

						System.out.println("Back Range: " + range);
						setBackRange(range);



						double vector_x = x_back;
						double vector_y = range + y_left;
						double mag = Math.sqrt(vector_x * vector_x + vector_y
								* vector_y);
						double angle = Math.atan2(vector_y, vector_x);
						double sensor_x = pose.x + Math.cos(pose.theta + angle)
								* mag;
						double sensor_y = pose.y + Math.sin(pose.theta + angle)
								* mag;
						
						Color color = Color.green;
						if (range < objectThreshold) {
							if (getState() == TRACKING_WALL) {
								updateLineEstimate(new Point2D.Double(sensor_x, sensor_y));
							}
							color = Color.red;
						}
						gui.addPoint(sensor_x, sensor_y, SonarGUI.X_POINT,
								color);
						gui.setLine(line_a, line_b, line_c, Color.cyan);
					}
				});

		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		// Publish reset message to odoSub to reset inital position
		odoSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(
					org.ros.message.rss_msgs.OdometryMsg message) {
				if (firstUpdate) {
					firstUpdate = false;
					gui.resetWorldToView(message.x, message.y);
				}
				gui.setRobotPose(message.x, message.y, message.theta);
				pose = new Pose(message.x, message.y, message.theta); // TODO
																		// synchronize
			}
		});
		Thread runningStuff = new Thread(this);
		runningStuff.start();
	}

	public synchronized String getState() {
		return state;
	}

	public synchronized void setState(String s) {
		state = s;
	}

	public synchronized boolean getLeftBumper() {
		return leftBumper;
	}

	public synchronized void setLeftBumper(boolean b) {
		leftBumper = b;
	}

	public synchronized boolean getRightBumper() {
		return rightBumper;
	}

	public synchronized void setRightBumper(boolean b) {
		rightBumper = b;
	}

	public synchronized double getFrontRange() {
		return frontRange;
	}

	public synchronized void setFrontRange(double d) {
		frontRange = d;
	}

	public synchronized double getBackRange() {
		return backRange;
	}

	public synchronized void setBackRange(double d) {
		backRange = d;
	}

	@Override
	public void onShutdown(Node node) {
		try {
			errorWriter.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		if (node != null) {
			node.shutdown();
		}
	}

	@Override
	public void onShutdownComplete(Node node) {
	}

	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("rss/localnavigation");
	}
}
