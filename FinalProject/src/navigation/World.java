package navigation;

import java.awt.Color;
import java.awt.geom.Point2D;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.text.ParseException;
import java.util.LinkedList;
import java.util.List;

import navigation.Constants.CollisionCheck;

public class World {
	protected Point start;
	protected Point goal;
	protected BoundingBox region;
	protected LinkedList<Fiducial> fiducials = new LinkedList<Fiducial>();
	protected LinkedList<Point> blocks = new LinkedList<Point>();
	protected LinkedList<Polygon> obstacles = new LinkedList<Polygon>();
	protected Polygon viewCone;
	protected Grid occupancyGrid;
	protected Grid visibilityGrid;

	public World(File mapFile) throws IOException, ParseException {
		viewCone = Constants.createViewCone();
		if (mapFile != null)
			parseChallenge(mapFile);
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
	
	protected Integer readInt(BufferedReader br) throws IOException {
		String line = nextLine(br);
		String[] tok = line.split("\\s+");
		
		return Integer.parseInt(tok[1]);
	}
	
	protected Double readDouble(BufferedReader br) throws IOException {
		String line = nextLine(br);
		String[] tok = line.split("\\s+");

		return Double.parseDouble(tok[1]);
	}
	
	protected Color readColor(BufferedReader br) throws Exception {
		String line = nextLine(br);
		String[] tok = line.split("\\s+");
	
		return (Color)Class.forName("java.awt.Color").getField(tok[1]).get(null);
	}
	
	protected Point readPoint(BufferedReader br) throws IOException {
		String line = nextLine(br);
		String[] tok = line.split("\\s+");

		return new Point(Double.parseDouble(tok[2]), Double.parseDouble(tok[3]));
	}
	
	protected String nextLine(BufferedReader br) throws IOException{
		String line = br.readLine();
		while (line != null) {
			String trimmed = line.trim();
			if (!(trimmed.length() == 0 || trimmed.charAt(0) == '#' || trimmed.charAt(0) == '{' || trimmed.charAt(0) == '}')){
				return trimmed;
			} 			
			line = br.readLine();
		}
		return null;
	}
	
	protected void parseChallenge(File mapFile) throws ParseException {
		try {
			BufferedReader br = new BufferedReader(new FileReader(mapFile));
			
			String line = nextLine(br);
			while (line != null) {
				if (line.length() >= 3 && line.substring(0, 3).equals("map")) {
					Point bottomLeft = readPoint(br);
					Point topRight = readPoint(br);
					region = new BoundingBox(bottomLeft, topRight);
					start = readPoint(br);
					goal = readPoint(br);
				} else if (line.length() >= 9 && line.substring(0, 9).equals("fiducials")) {
					int numFiducials = readInt(br);
					for (int i = 0; i < numFiducials; i++) {
						nextLine(br);
						Point position = readPoint(br);
						Color topColor = readColor(br);
						Color bottomColor = readColor(br);
						Double topRadius = readDouble(br);
						Double bottomRadius = readDouble(br);
						fiducials.add(new Fiducial(position, topRadius, topColor));
					}
				} else if (line.length() >= 20 && line.substring(0, 20).equals("construction_objects")) {
					int numBlocks = readInt(br); //TODO blocks may have additional syntax
					for (int i = 0; i < numBlocks; i++) {
						nextLine(br);
						blocks.add(readPoint(br));
					}
				} else if (line.length() >= 9 && line.substring(0, 9).equals("obstacles")) {
					int numObstacles = readInt(br);
					for (int i = 0; i < numObstacles; i++) {
						nextLine(br);
						LinkedList<Point> points = new LinkedList<Point>();
						int numPoints = readInt(br);
						for (int p = 0; p < numPoints; p++) {
							points.add(readPoint(br));
						}
						obstacles.add(new Polygon(points));
					}
				} else {
					throw new Exception();
				}
				line = nextLine(br);
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
		occupancyGrid = new Grid(region);
		visibilityGrid = new Grid(region);
		
		for (Polygon obst : obstacles) {
			occupancyGrid.markColliding(obst);
		}
		
		visibilityGrid.markColliding(getViewCone(start.configuration(0)));
	}

	public Shape getRobot(Configuration c){
		return Constants.createRobot(0.).pose(c);
	}
	
	public Shape getRobot(Configuration c, double grow){
		return Constants.createRobot(grow).pose(c);
	}
	
	public Polygon getViewCone(Configuration c){
		return viewCone.pose(c);
	}
	
	public BoundingBox getRegion() {
		return region;
	}
	
	public Point getStart() {
		return start;
	}
	
	public Point getGoal() {
		return goal;
	}

	public List<Polygon> getObstacles() {
		return obstacles;
	}
	
	public List<Fiducial> getFiducials() {
		return fiducials;
	}
	
	public List<Point> getBlocks() {
		return blocks;
	}
	
	public Grid getOccupancyGrid() {
		return occupancyGrid;
	}
	
	public Grid getVisibilityGrid() {
		return visibilityGrid;
	}
	
	public boolean robotMapOnlyCollision(Configuration c, double grow) {
		Shape placedRobot = getRobot(c, grow); //TODO lazily place robot
		for (Polygon obst : obstacles) {
			if (placedRobot.collides(obst)) {
				return true;
			}
		}
		return !region.contains(placedRobot);
	}
	
	public boolean robotGridOnlyCollision(Configuration c, double grow) {
		Shape placedRobot = getRobot(c, grow); //TODO lazily place robot
		return occupancyGrid.collides(placedRobot, .1); //TODO adjust threshold
	}
	
	public boolean robotMapAndGridCollision(Configuration c, double grow) {
		Shape placedRobot = getRobot(c, grow); //TODO lazily place robot
		for (Polygon obst : obstacles) {
			if (placedRobot.collides(obst)) {
				return true;
			}
		}		
		return !region.contains(placedRobot) || occupancyGrid.collides(placedRobot, .1); //TODO adjust threshold
	}
	
	public boolean robotCollision(Configuration c, double grow, CollisionCheck check) {
		switch (check) {
		case MAPONLY: return robotMapOnlyCollision(c, grow);
		case GRIDONLY: return robotGridOnlyCollision(c, grow);
		case MAPANDGRID: return robotMapAndGridCollision(c, grow);
		default: return true;
		}
	}
	
	public Configuration sampleConfigurationForPoint(Point p) {
		int attempts = 30;
		for (int i = 0; i < attempts; i++) {
			Configuration config = p.configuration(2 * Math.PI * Math.random());
			if (!robotMapOnlyCollision(config, 0.)) {
				return config;
			}
		}
		return null;
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
