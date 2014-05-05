package navigation;

import java.awt.Color;

public class Block{
	public Point position;
	public Color color;
	public int size;
	
	public static final double LENGTH = 0.05;

	public Block(){
		this.position = null;
		this.color = null;
		this.size = -1;
	}
	
	public boolean isKnownColor(){
		return (color!=null);
	}
	    
	public boolean isKnownSize(){
		return (size>0);
	}

	public boolean isKnownPosition(){
		return (position!=null);
	}
}
