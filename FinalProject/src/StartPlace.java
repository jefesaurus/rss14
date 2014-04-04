import gui.GUI;

import java.awt.Color;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.List;

import org.ros.message.lab6_msgs.GUIPolyMsg;
import org.ros.message.lab6_msgs.GUIRectMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Subscriber;

public class StartPlace implements NodeMain, Runnable {

	public StartPlace() {
		//GUI gui = new GUI();
	}

	@Override
	public void run() {

	}

	@Override
	public void onStart(Node node) {
		System.out.println("I'm working");
	}

	@Override
	public void onShutdown(Node node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onShutdownComplete(Node node) {
		// TODO Auto-generated method stub

	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}

	public static void fillRectMsg(GUIRectMsg rectMsg,
			java.awt.geom.Rectangle2D.Double worldRect, Object object, boolean b) {
		// TODO Auto-generated method stub

	}
}
