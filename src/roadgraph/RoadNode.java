package roadgraph;

public class RoadNode {
	
	private String roadName;
	private String roadType;
	private double length;
	private MapNode intersection1;
	private MapNode intersection2;
	
	public RoadNode(String roadName, String roadType, double length, MapNode intersection1, MapNode intersection2) {
		
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
		this.intersection1 = intersection1;
		this.intersection2 = intersection2;
	}
	
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
