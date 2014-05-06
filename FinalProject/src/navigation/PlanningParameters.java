package navigation;

import navigation.Constants.CollisionCheck;
import navigation.Constants.DriveSystem;

public abstract class PlanningParameters {
	public final DriveSystem drive;
	public final CollisionCheck check;

	public PlanningParameters(DriveSystem drive, CollisionCheck check) {
		this.drive = drive;
		this.check = check;
	}
	
	public abstract double computeGrow(double distance);
}
