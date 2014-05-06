package navigation;

import navigation.Constants.DriveSystem;

public class Waypoint {
	public final Configuration config;
	public final DriveSystem drive;
	public final double distance;
	public final double grow;
	
	public Waypoint(Configuration config, DriveSystem drive, double distance, double grow) {
		this.config = config;
		this.drive = drive;
		this.distance = distance;
		this.grow = grow;
	}
}
