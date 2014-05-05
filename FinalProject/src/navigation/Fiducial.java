package navigation;

import java.awt.Color;

public class Fiducial {
	public Point position;
	public Color topColor, bottomColor;
	public double topRadius, bottomRadius;
    
    public Fiducial(){
		this.position = null;
		this.topColor = null;
		this.bottomColor = null;
		this.topRadius = -1.;
		this.bottomRadius = -1.;
    }
    
	public boolean isKnownPosition(){
		return (position!=null);
	}
	
	public boolean isKnownTopColor(){
		return (topColor!=null);
	}
	
	public boolean isKnownBottomColor(){
		return (bottomColor!=null);
	}
	
	public boolean isKnownTopRadius(){
		return (topRadius>0);
	}

	public boolean isKnownBottomRadius(){
		return (bottomRadius>0);
	}
}