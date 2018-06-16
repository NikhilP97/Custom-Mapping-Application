package roadgraph;

/**
 * @author Nikhil Pinto
 * 
 * A class which represents a Road Node in a graph 
 * Road Nodes in the graph are a superSet of two Nodes 
 * and have properties of having a name, type, length & are made of two Nodes
 */

public class RoadNode {
	
	private String roadName;
	private String roadType;
	private double length;
	private MapNode intersection1;
	private MapNode intersection2;
	
	// Constructor
	public RoadNode(String roadName, String roadType, double length, MapNode intersection1, MapNode intersection2) {
		
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
		this.intersection1 = intersection1;
		this.intersection2 = intersection2;
	}
	
	// Getters & Setters
	public String getRoadName() {
		return roadName;
	}

	public String getRoadType() {
		return roadType;
	}

	public double getLength() {
		return length;
	}

	public MapNode getIntersection1() {
		return intersection1;
	}

	public MapNode getIntersection2() {
		return intersection2;
	}
	
	

}
