package GlobalNavigation;

import java.util.*;
import java.awt.Color;
import java.awt.geom.*;
import java.awt.geom.Point2D.*;
import java.lang.Double;

public class CSpaceSingle {

  PolygonObstacle bot;
  Point2D.Double refPoint;
  List<PolygonObstacle> realObstacles;
  List<PolygonObstacle> cSpaceObs; // Rotation layers
  double angle;

  public CSpaceSingle(List<PolygonObstacle> realObstacles, PolygonObstacle bot, Point2D.Double refPoint, double angle) {
	  this.refPoint = refPoint;
	  this.bot = GeomUtils.RotatePoints(bot, angle, this.refPoint );
	  this.realObstacles = realObstacles;
	  this.cSpaceObs = new ArrayList<PolygonObstacle>();
	  compute();
  }
  
  public CSpaceSingle(List<PolygonObstacle> realObstacles, PolygonObstacle bot, double angle) {
	  this.refPoint = new Point2D.Double(0.,0.);
	  this.bot = GeomUtils.RotatePoints(bot, -angle, this.refPoint );
	  this.realObstacles = realObstacles;
	  this.cSpaceObs = new ArrayList<PolygonObstacle>();
	  compute();
  }
  
  
  public void compute() {
	  for (PolygonObstacle realObstacle : realObstacles) {
		  this.cSpaceObs.add(makeCSObstacle(realObstacle, bot, refPoint));
	  }
  }
  
  public List<PolygonObstacle> getObstacles() {
	  return this.cSpaceObs;
  }
 
  
  public void draw(MapGUI gui) {
		for (PolygonObstacle obstacle : cSpaceObs) {
			gui.addPoly(obstacle.getVertices(), true, false, Color.RED);
		}
  }
  
  public static PolygonObstacle makeCSObstacle(PolygonObstacle baseObj, PolygonObstacle toAddObj, Point2D.Double refPoint) {
		PolygonObstacle FlippedObstacle = GeomUtils.RotatePoints(toAddObj, Math.PI, refPoint);
		return GeomUtils.MSum(baseObj, FlippedObstacle, refPoint);
  }
  
  public boolean intersects(Point2D.Double point) {
	  for (PolygonObstacle obstacle : this.cSpaceObs) {
		  if (GeomUtils.isPointInPolyFuzzy(point, obstacle)) {
			  return true;
		  }
	  }
	  return false;
  }
}