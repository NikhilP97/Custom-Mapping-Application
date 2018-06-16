package roadgraph;

import java.util.HashSet;

import geography.GeographicPoint;

/**
 * @author Nikhil Pinto
 * 
 * A class which represents a Node in a graph 
 * Nodes in the graph have properties of having a parent Node, neighbor & have a location
 */

public class MapNode {
	
	private GeographicPoint geoPoint;
	private HashSet<MapNode> neighbours;
	private MapNode parent;
	
	// Constructor to initialize class variables
	public MapNode(GeographicPoint geoPoint) {
		this.geoPoint = geoPoint;
		neighbours = new HashSet<MapNode>();
		this.parent = null;
	}
	
	// Getters & Setters
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
