package roadgraph;

import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;

import geography.GeographicPoint;

/**
 * @author Nikhil Pinto
 * 
 * A class which represents a Node in a graph 
 * Nodes in the graph have properties of having a parent Node, neighbor & have a location
 */

public class MapNode  {
	
	private GeographicPoint geoPoint;
	private HashSet<MapNode> neighbours;
	private MapNode parent;
	private double distanceOfNode;
	private double Fvalue;
	private HashMap<MapNode, Double> edgeLength;
	
	// Constructor to initialize class variables
	public MapNode(GeographicPoint geoPoint) {
		this.geoPoint = geoPoint;
		neighbours = new HashSet<MapNode>();
		this.parent = null;
		this.distanceOfNode = Integer.MAX_VALUE;
		edgeLength = new HashMap<MapNode, Double>();
	}
	
	public MapNode() {
		
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

	public double getDistanceOfNode() {
		return distanceOfNode;
	}

	public void setDistanceOfNode(double distanceOfNode) {
		this.distanceOfNode = distanceOfNode;
	}

	public HashMap<MapNode, Double> getEdgeLength() {
		return edgeLength;
	}

	public void setEdgeLength(MapNode otherNode, double edgeinKM) {
		this.edgeLength.put(otherNode, edgeinKM);
	}

//	@Override
//	public int compareTo(MapNode other) {
//		// TODO Auto-generated method stub
//		if(this.distanceOfNode > other.distanceOfNode) {
//			return 1;
//		}
//		else if(this.distanceOfNode < other.distanceOfNode) {
//			return -1;
//		}
//		else {
//			return 0;
//		}
//	}
//
//	@Override
//	public int compare(MapNode o1, MapNode o2) {
//		// TODO Auto-generated method stub
//		if(o1.Fvalue > o2.Fvalue) {
//			return 1;
//		}
//		else if(o1.Fvalue < o2.Fvalue) {
//			return -1;
//		}
//		else {
//			return 0;
//		}
//		
//	}
	
	public String toString()
    {
    	return "("+geoPoint.getX()+","+geoPoint.getY()+"="+Fvalue + ")" ;
    }

	public double getFvalue() {
		return Fvalue;
	}

	public void setFvalue(double fvalue) {
		Fvalue = fvalue;
	}
	
	
	

}
