package navigation;

import navigation.Constants.CollisionCheck;
import navigation.Constants.DriveSystem;

public class LinearGrowthParameters extends PlanningParameters {
	public double growSlope, maxGrowth;
	
	public LinearGrowthParameters(DriveSystem drive, CollisionCheck check, double growSlope, double maxGrowth) {
		super(drive, check);
		this.growSlope = growSlope;
		this.maxGrowth = maxGrowth;
	}
	
	public double computeGrow(double distance) {
		return Math.min(maxGrowth, growSlope*distance);
	}
	
	public PlanningParameters changeDrive(DriveSystem newDrive) {
		return new LinearGrowthParameters(newDrive, check, growSlope, maxGrowth);
	}
}
