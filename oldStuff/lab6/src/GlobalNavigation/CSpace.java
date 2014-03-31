package GlobalNavigation;

import java.util.*;
import java.awt.Color;
import java.awt.geom.*;
import java.awt.geom.Point2D.*;
import java.lang.Double;

public class CSpace {

  PolygonObstacle bot;
  Point2D.Double refPoint;
  List<PolygonObstacle> realObstacles;
  List<List<PolygonObstacle>> rotLayers; // Rotation layers
  List<Double> angles; // Angles that correspond this the cspace layers
 

  public CSpace(List<PolygonObstacle> realObstacles, PolygonObstacle bot, Point2D.Double refPoint) {
	  this.bot = bot;
	  this.refPoint = refPoint;
	  this.realObstacles = realObstacles;
	  this.rotLayers = new ArrayList<List<PolygonObstacle>>();
	  this.angles = new ArrayList<Double>();
  }
  
  
  /*
   * This calculates the cspace for a given number of rotation increments and 
   * adds it to the layers list.
   * This needs to be called once, and then you can access the cspaces for the layers
   * 
   */
  public void compute(int numRotationSteps) {
	  clear();
	  double angleStep = Math.PI*2./numRotationSteps;
	  double currentAngle = 0.0;
	  List<PolygonObstacle> firstLayer = new LinkedList<PolygonObstacle>();
	  for (PolygonObstacle realObstacle : realObstacles) {
		  firstLayer.add(makeCSObstacle(realObstacle, bot, refPoint));
	  }
	  rotLayers.add(firstLayer);
	  this.angles.add(new Double(currentAngle));

	  
	  for (int i = 1; i < numRotationSteps; i ++) {
		currentAngle += angleStep;
		PolygonObstacle rotatedBot = GeomUtils.RotatePoints(this.bot, currentAngle, this.refPoint);
		
		List<PolygonObstacle> newLayer = new LinkedList<PolygonObstacle>();
		
	    for (PolygonObstacle realObstacle : realObstacles) {
	    	newLayer.add(makeCSObstacle(realObstacle, rotatedBot, refPoint));
	    }
		rotLayers.add(newLayer);
		this.angles.add(new Double(currentAngle));
	  }
  }
  
  public void clear() {
	  this.rotLayers = new LinkedList<List<PolygonObstacle>>();
	  this.angles = new LinkedList<Double>();
  }
  
  public void draw(MapGUI gui) {
		for (List<PolygonObstacle> obstacles : this.rotLayers) {
			for (PolygonObstacle obstacle : obstacles) {
				gui.addPoly(obstacle.getVertices(), true, false, Color.RED);
			}
		}
  }

  public List<PolygonObstacle> getObstacles() {
    return realObstacles;
  }
  
  public List<List<PolygonObstacle>> rotationLayers() {
	  return this.rotLayers;
  }
  
  public static PolygonObstacle makeCSObstacle(PolygonObstacle baseObj, PolygonObstacle toAddObj, Point2D.Double refPoint) {
		PolygonObstacle FlippedObstacle = GeomUtils.RotatePoints(toAddObj, Math.PI, refPoint);
		return GeomUtils.MSum(baseObj, FlippedObstacle, refPoint);
  }
  
  public boolean intersects(Point2D.Double point, int rotIndex) {
	  for (PolygonObstacle obstacle : this.rotLayers.get(rotIndex)) {
		  if (GeomUtils.isPointInPolyFuzzy(point, obstacle)) {
			  return true;
		  }
	  }
	  return false;
  }
}