package roadgraph;

import java.util.HashSet;

import geography.GeographicPoint;

public class MapNode {
	
	private GeographicPoint geoPoint;
	private HashSet<MapNode> neighbours;
	private MapNode parent;
	
	public MapNode(GeographicPoint geoPoint) {
		this.geoPoint = geoPoint;
		neighbours = new HashSet<MapNode>();
		this.parent = null;
	}

	public GeographicPoint getGeoPoint() {
		return geoPoint;
	}

	public void setNeighbours(MapNode newNeighbour) {
		neighbours.add(newNeighbour);
	}

	public HashSet<MapNode> getNeighbours() {
		return neighbours;
	}

	public MapNode getParent() {
		return parent;
	}

	public void setParent(MapNode parent) {
		this.parent = parent;
	}
	
	

}
