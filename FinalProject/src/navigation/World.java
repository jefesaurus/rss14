package navigation;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.text.ParseException;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import navigation.Constants.CollisionCheck;

public class World {
	protected Point start;
	protected Point goal;
	protected BoundingBox region;
	protected List<Fiducial> fiducials;
	protected List<Block> blocks;
	protected List<Polygon> obstacles;
	protected Polygon viewCone;
	protected Grid occupancyGrid;
	protected Grid visibilityGrid;
	
	public Navigator navigator;

	public World(File mapFile) throws IOException, ParseException {
		if (mapFile != null) {
			loadWorld(mapFile);
		}
	}
	
	public World(String mapFile) throws IOException, ParseException {
		this((mapFile != null) ? new File(mapFile) : null);
	}
	
	private void loadWorld(File file) throws IOException, ParseException{
		parseFile(file);
		
		start = new Point(2.6 , 4.38); //TODO 
		
		occupancyGrid = new Grid(region);
		
		//for (Polygon obst : obstacles) {
		//	occupancyGrid.markColliding(obst);
		//}
		
		//visibilityGrid = new Grid(region);
		viewCone = Constants.createViewCone();
		//visibilityGrid.markColliding(getViewCone(start.configuration(0)));
	}
	
	
	private static final String COMMENT = "#";
    private static final String SECTION_START = "{";
    private static final String SECTION_END = "}";  
    private void parseFile(File file) throws IOException, ParseException{
    	BufferedReader br = new BufferedReader(new FileReader(file));
    	
    	parseToken(br,"map");
    	parseToken(br,SECTION_START);
    	
    	Point bottomLeft = null;
    	Point topRight = null;
    	while(true){
    		String token = parseToken(br);
    		if(token.equals(SECTION_END)){
    			break;
    		}
    		else if(token.equals("fiducials")){
    			fiducials = parseFiducials(br);
    		}
    		else if(token.equals("construction_objects")){
    			blocks = parseBlocks(br);
    		}
    		else if(token.equals("obstacles")){
    			obstacles = parseObstacles(br);
    		}
    		else if(token.equals("bottom_left")){
    			bottomLeft = parsePoint(br);
    		}
    		else if(token.equals("top_right")){
    			topRight = parsePoint(br);
    		} 
		else if(token.equals("robot_start")){
    			start = parsePoint(br);
    		}
    		else if(token.equals("robot_goal")){
    			goal = parsePoint(br);
    		}
    		else{
    			throw new ParseException("Unknown section token '"+token+"' in file.",0);
    		}
    	}
    	if(bottomLeft!=null && topRight!=null){
    		region = new BoundingBox(bottomLeft, topRight);
    	}
    	else{
    		throw new ParseException("Could not define the world rectangle.  Missing either bottom_left or top_right.",0);
    	}
    	br.close();
    }

    private List<Fiducial> parseFiducials(BufferedReader br) throws IOException, ParseException{
    	parseToken(br,SECTION_START);
    	parseToken(br,"num_fiducials");
    	int numFiducials = parseInt(br);
    	List<Fiducial> fids = new LinkedList<Fiducial>();
    	for(int i=0;i<numFiducials;i++){
    		int index = parseInt(br);
    		if(index<0 || index>=numFiducials){
    			throw new ParseException("Fiducial Index out of range: Expected [0-"+(numFiducials-1)+"].  Got '"+index+"'.",0);
    		}
    		fids.add(parseFiducial(br));
    	}
    	parseToken(br,SECTION_END);

    	return fids;
    }
    
    private Fiducial parseFiducial(BufferedReader br) throws IOException, ParseException{
    	parseToken(br,SECTION_START);
    	Fiducial fiducial = new Fiducial();
    	while(true){
    		String token = parseToken(br);
    		if(token.equals("position")){
    			fiducial.position = parsePoint(br);
    		}
    		else if(token.equals("top_color")){
    			fiducial.topColor = parseColor(br);
    		}
    		else if(token.equals("bottom_color")){
    			fiducial.bottomColor = parseColor(br);
    		}
    		else if(token.equals("top_radius")){
    			fiducial.topRadius = parseDouble(br);
    		}
    		else if(token.equals("bottom_radius")){
    			fiducial.bottomRadius = parseDouble(br);
    		}
    		else if(token.equals(SECTION_END)){
    			break;
    		}
    		else{
    			throw new ParseException("Unknown token in Fiducial: '"+token+"'",0);
    		}
    	}
    	return fiducial;
    }
    
    private List<Block> parseBlocks(BufferedReader br) throws IOException, ParseException{    	
    	parseToken(br,SECTION_START);
    	parseToken(br,"num_construction_objects");
    	int numConstructionObjects = parseInt(br);
    	List<Block> bls = new LinkedList<Block>();
    	for (int i=0;i<numConstructionObjects;i++){
    		int index = parseInt(br);
    		if(index<0 || index>=numConstructionObjects){
    			throw new ParseException("ConstructionObject Index out of range: Expected [0-"+(numConstructionObjects-1)+"].  Got '"+index+"'.",0);
    		}
    		bls.add(parseBlock(br));
    	}
    	parseToken(br,SECTION_END);
    	return bls;
    }
    
    private Block parseBlock(BufferedReader br) throws IOException, ParseException{    	
    	parseToken(br,SECTION_START);
    	Block block = new Block();
    	while(true){
    		String token = parseToken(br);
    		if(token.equals("position")){
    			block.position = parsePoint(br);
    		}
    		else if(token.equals("color")){
    			block.color = parseColor(br);
    		}
    		else if(token.equals("size")){
    			block.size = parseInt(br);
    		}
    		else if(token.equals(SECTION_END)){
    			break;
    		}
    		else{
    			throw new ParseException("Unknown token in ConstructionObject: '"+token+"'",0);
    		}
    	}
    	return block;
    }
    
    private List<Polygon> parseObstacles(BufferedReader br) throws IOException, ParseException{
    	parseToken(br,SECTION_START);
    	parseToken(br,"num_obstacles");
    	int numObstacles = parseInt(br);
    	List<Polygon> obs = new LinkedList<Polygon>();
    	for(int i=0;i<numObstacles;i++){
    		int index = parseInt(br);
    		if(index<0 || index>=numObstacles){
    			throw new ParseException("Obstacle Index out of range: Expected [0-"+(numObstacles-1)+"].  Got '"+index+"'.",0);
    		}
    		obs.add(parseObstacle(br));
    	}
    	return obs;
    }
    
    private Polygon parseObstacle(BufferedReader br) throws IOException, ParseException{
    	parseToken(br,SECTION_START);
    	parseToken(br,"num_points");
    	int numPoints = parseInt(br);
    	if(numPoints<3){
    		throw new ParseException("Cannot have fewer than 3 points in obstacles.  This one only has "+numPoints+" points.",0);
    	}
    	
    	List<Point> points = new LinkedList<Point>();
    	for (int i=0;i<numPoints;i++){
    		parseInt(br);//Don't need to store the index.
    		points.add(0, parsePoint(br));
    	}
    	parseToken(br,SECTION_END);    	
    	return new Polygon(points);
    }
    
    private double[] parseVectorNd(BufferedReader br, int n) throws IOException, ParseException {
    	if(n<1){
    		throw new ParseException("Tried to parse a Vector of length "+n+".",0);
    	}
    	double[] result = new double[n];
    	parseToken(br,SECTION_START);
    	for(int i=0;i<n;i++){
    		result[i] = parseDouble(br);
    	}
    	parseToken(br,SECTION_END);
    	return result;
    }
    
    private Point parsePoint(BufferedReader br) throws IOException, ParseException {
    	double[] vec = parseVectorNd(br,2);
    	return new Point(vec[0],vec[1]);
    }
    
    private double parseDouble(BufferedReader br) throws IOException, ParseException {
    	String token = parseToken(br);
    	
    	try{
    		return Double.parseDouble(token);
    	}catch(NumberFormatException nfe){
    		throw new ParseException("Expected Double.  Got '"+token+"'.",0);
    	}
    }

    private Color parseColor(BufferedReader br) throws IOException, ParseException {
    	String token = parseToken(br).toLowerCase();
		if(token.equals("red")){
		    return Color.RED;
		}
		else if(token.equals("blue")){
		    return Color.BLUE;
		}
		else if(token.equals("yellow")){
		    return Color.YELLOW;
		}
		else if(token.equals("orange")){
		    return Color.ORANGE;
		}
		else if(token.equals("green")){
		    return Color.GREEN;
		}
		throw new ParseException("Unable to parse color with token '"+token+"'.",0);
    }
    
    private int parseInt(BufferedReader br) throws IOException, ParseException {
    	String token = parseToken(br);
    	try{
    		return Integer.parseInt(token);
    	}catch(NumberFormatException nfe){
    		throw new ParseException("Expected Integer.  Got '"+token+"'.",0);
    	}
    }
    
    private String parseToken(BufferedReader br) throws IOException, ParseException {
    	String result = "";
    	boolean leadingWhitespace = true;
    	while(true){
    		int c = br.read();
    		String s = Character.toString((char)c);
    		if(s.equals(COMMENT)){
    			br.readLine();
    			continue;
    		}
    		else if(Character.isWhitespace(c)){
    			if(!leadingWhitespace){
    				break;
    			}
    		}
    		else{
    			leadingWhitespace = false;
    			result+=s;
    		}
    	}
    	return result;
    }
    
    private String parseToken(BufferedReader br, String expected) throws IOException, ParseException {
    	String token = parseToken(br);
    	if(!token.equals(expected)){
    		throw new ParseException("Expected token '"+expected+"'.  Got '"+token+"' instead.",0);
    	}
    	return token;
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
	
	public List<Block> getBlocks() {
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
		return occupancyGrid.collides(placedRobot); //TODO adjust threshold
	}
	
	public boolean robotMapAndGridCollision(Configuration c, double grow) {
		Shape placedRobot = getRobot(c, grow); //TODO lazily place robot
		for (Polygon obst : obstacles) {
			if (placedRobot.collides(obst)) {
				return true;
			}
		}		
		return !region.contains(placedRobot) || occupancyGrid.collides(placedRobot); //TODO adjust threshold
	}
	
	public boolean robotCollision(Configuration c, double grow, CollisionCheck check) {
		switch (check) {
		case MAPONLY: return robotMapOnlyCollision(c, grow);
		case GRIDONLY: return robotGridOnlyCollision(c, grow);
		case MAPANDGRID: return robotMapAndGridCollision(c, grow);
		default: return true;
		}
	}
	
	float OCCUPANCY_RESOLUTION = .02f;
	int OCCUPANCY_THRESHOLD = 3;
	public void updateOccupancy(HashMap<IntTuple, double[]> occpancyMap) {		
		Configuration current = navigator.getConfiguration();
		Polygon placedCone = getViewCone(current);
		
		for (Map.Entry<IntTuple, double[]> cell : occpancyMap.entrySet()) {
			double[] pointData = cell.getValue();
			double numPoints = pointData[0];
			if (numPoints > OCCUPANCY_THRESHOLD) {
				IntTuple loc = cell.getKey();
				Point point = new Point(loc.x*OCCUPANCY_RESOLUTION, loc.y*OCCUPANCY_RESOLUTION).forwardTransform(current);
				if (placedCone.contains(point)) {
					occupancyGrid.markColliding(point);
				}
			}
		}
	}	
}
