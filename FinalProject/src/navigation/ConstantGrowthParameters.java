package navigation;

import navigation.Constants.CollisionCheck;
import navigation.Constants.DriveSystem;

public class ConstantGrowthParameters extends PlanningParameters {
	public double grow;
	
	public ConstantGrowthParameters(DriveSystem drive, CollisionCheck check, double grow) {
		super(drive, check);
		this.grow = grow;
	}
	
	public double computeGrow(double distance) {
		return grow;
	}
}
