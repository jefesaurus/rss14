package navigation;

import java.util.LinkedList;
import java.util.List;

import navigation.Constants.DriveSystem;

public class TreeNode {
	public Configuration config;
	public List<TreeNode> children;
	public TreeNode parent;
	public DriveSystem drive;
	public double grow;
	public double distance;

	public TreeNode(Configuration config, DriveSystem drive, double grow) {
		this.config = config;
		this.children = new LinkedList<TreeNode>();
		this.parent = null;
		this.drive = drive;
		this.distance = 0.;
		this.grow = grow;
	}

	public TreeNode(Configuration config, TreeNode parent, DriveSystem drive, double distance, double grow) {
		this.config = config;
		this.children = new LinkedList<TreeNode>();
		this.parent = parent;
		this.drive = drive;
		this.distance = distance;
		this.grow = grow;
	}
}