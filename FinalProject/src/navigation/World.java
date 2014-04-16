package navigation;

import java.awt.geom.Point2D;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.text.ParseException;
import java.util.LinkedList;
import java.util.List;

public class World {
	protected BoundingBox region;
	protected LinkedList<Polygon> obstacles = new LinkedList<Polygon>();
	protected Shape robot;

	public World(File mapFile) throws IOException, ParseException {
		if (mapFile != null)
			parse(mapFile);
		robot = Constants.createRobot();
	}
	
	public World(String mapFile) throws IOException, ParseException {
		this((mapFile != null) ? new File(mapFile) : null);
	}
	
	protected double parseDouble(BufferedReader br, String name, int lineNumber) throws IOException, ParseException, NumberFormatException {
		String line = br.readLine();
		if (line == null)
			throw new ParseException(name + " (line " + lineNumber + ")",
					lineNumber);

		return Double.parseDouble(line);
	}

	protected void parsePoint(Point2D.Double point, BufferedReader br, String name, int lineNumber) throws IOException, ParseException, NumberFormatException {
		String line = br.readLine();
		String[] tok = (line != null) ? line.split("\\s+") : null;

		if ((tok == null) || (tok.length < 2)){
			throw new ParseException(name + " (line " + lineNumber + ")",
					lineNumber);
		}

		point.x = Double.parseDouble(tok[0]);
		point.y = Double.parseDouble(tok[1]);
	}

	protected BoundingBox parseRect(BufferedReader br, String name, int lineNumber) throws IOException, ParseException, NumberFormatException {
		String line = br.readLine();
		String[] tok = (line != null) ? line.split("\\s+") : null;

		if ((tok == null) || (tok.length < 4))
			throw new ParseException(name + " (line " + lineNumber + ")",
					lineNumber);

		double x = Double.parseDouble(tok[0]);
		double y = Double.parseDouble(tok[1]);
		double width = Double.parseDouble(tok[2]);
		double height = Double.parseDouble(tok[3]);
		
		return new BoundingBox(new Point(x, y), new Point(x + width, y + height));	
	}

	protected Polygon parseObs(BufferedReader br, String name, int lineNumber) throws IOException, ParseException, NumberFormatException {
		String line = br.readLine();

		if (line == null)
			return null;

		String[] tok = line.trim().split("\\s+");

		if (tok.length == 0)
			return null;

		if (tok.length%2 != 0)
			throw new ParseException(name + " (line " + lineNumber + ")",
					lineNumber);

		LinkedList<Point> points = new LinkedList<Point>();	
		for (int i = 0; i < tok.length/2; i++)
			points.add(new Point(Double.parseDouble(tok[2*i]), Double.parseDouble(tok[2*i+1])));

		return new Polygon(points);
	}

	protected void parse(File mapFile) throws IOException, ParseException {
		int lineNumber = 1;
		try {

			BufferedReader br = new BufferedReader(new FileReader(mapFile));

			br.readLine(); br.readLine(); //Skipping start and goal
			//parsePoint(robotStart, br, "robot start", lineNumber++);
			//parsePoint(robotGoal, br, "robot goal", lineNumber++);
			region = parseRect(br, "region", lineNumber++);

			for (int obstacleNumber = 0; ; obstacleNumber++) {
				Polygon poly = parseObs(br, "obstacle " + obstacleNumber, lineNumber++);
				if (poly != null) {
					obstacles.add(poly);
				}
				else
					break;
			}

		} catch (NumberFormatException e) {
			throw new ParseException("malformed number on line " + lineNumber,
					lineNumber);
		}
	}

	public Shape getRobot(Configuration c){
		return robot.pose(c);
	}
	
	public BoundingBox getRegion() {
		return region;
	}

	public List<Polygon> getObstacles() {
		return obstacles;
	}
	
	public boolean robotCollision(Configuration c) {
		Shape placedRobot = getRobot(c);
		for (Polygon obst : obstacles) {
			if (placedRobot.collides(obst)) {
				return true;
			}
		}
		return !region.contains(placedRobot);
	}

	@Override
	public String toString() {
		StringBuffer sb = new StringBuffer();
		
		/*
		sb.append("\nworld rect: x=");
		sb.append(Double.toString(region.x));
		sb.append(" y=");
		sb.append(Double.toString(region.y));
		sb.append(" width=");
		sb.append(Double.toString(region.width));
		sb.append(" height=");
		sb.append(Double.toString(region.height));
		
		sb.append("\n" + obstacles.size() + " obstacles:");
		for (Polygon obstacle : obstacles) {
			sb.append("\n ");
			obstacle.toStringBuffer(sb);
		}
		*/

		return sb.toString();
	}
}
