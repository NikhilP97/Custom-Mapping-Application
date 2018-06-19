/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and Nikhil Pinto
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 * Uses MapNode & RoadNode classes for functionality
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private int numOfVertices; 
	private int numOfEdges;
	private HashSet<GeographicPoint> setOfVertices;
	private HashMap<GeographicPoint, MapNode> getNodefromLocation; // to get the Node of a graph from a Geographic Point
	private HashSet<RoadNode> setOfRoads; // A set containing Road Nodes which hold properties like road name, road type,
										  // length, intersection Nodes
	
	/** 
	 * Create a new empty MapGraph, initialize instance variables 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		numOfVertices = 0;
		numOfEdges = 0;
		setOfVertices = new HashSet<GeographicPoint>();
		getNodefromLocation = new HashMap<GeographicPoint, MapNode>();
		setOfRoads = new HashSet<RoadNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return numOfVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return setOfVertices;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return numOfEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		if(setOfVertices.contains(location) || location == null) {
			return false;
		}
		
		MapNode newVertx = new MapNode(location);
		numOfVertices++;
		setOfVertices.add(location);
		// Add to Map which stores location as key and MapNode as value
		getNodefromLocation.put(location, newVertx);
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
		if(!setOfVertices.contains(from) && !setOfVertices.contains(to) && from == null && to == null && length == 0) {
			return;
		}
		numOfEdges++;
		MapNode source = getNodefromLocation.get(from);
		MapNode destination = getNodefromLocation.get(to);
		// Add neighbour
		source.setNeighbours(destination);
		double distanceFromNodes = from.distance(to);
		source.setEdgeLength(destination, distanceFromNodes);
		// create new Road Node
		RoadNode newRoadNode = new RoadNode(roadName, roadType, length, source, destination);
		// Add to list of Road Nodes
		setOfRoads.add(newRoadNode);
		
		
		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		// Create initial Nodes & required lists and queues
		MapNode startNode = getNodefromLocation.get(start);
		MapNode destNode = getNodefromLocation.get(goal);
		HashSet<MapNode> visited = new HashSet<MapNode>();
		Queue<MapNode> queue = new LinkedList<MapNode>();
		
		// holds current Node while searching
		MapNode currentNode = startNode;
		// Add start to queue & continue until all Nodes are explored
		int countNoOfNodes = 0;
		queue.add(startNode);
		while(!queue.isEmpty()) {
			currentNode = queue.remove();
			countNoOfNodes++;
			System.out.println(currentNode.getGeoPoint()); // for debugging
			
			// Only if Node is not visited process it or else fetch the next Node from queue
			if(!visited.contains(currentNode)) {
				visited.add(currentNode);
				nodeSearched.accept(currentNode.getGeoPoint()); // for visualizing on the front-end interface
				// for every Neighbour of the current Node:
				for(MapNode currentNeighbour : currentNode.getNeighbours()) {
					// If Node not visited only then process or else check next
					if(!visited.contains(currentNeighbour)) {
						currentNeighbour.setParent(currentNode);
						// If X, Y of current is same as X, Y of start : start backtracking and find path
						if(currentNeighbour.getGeoPoint().getX() == goal.getX() && currentNeighbour.getGeoPoint().getY() == goal.getY()) {
							return searchedPath(currentNeighbour, start);
						}
						// else add to queue
						queue.add(currentNeighbour);
					}
					
				}
			}
		}
		// Return null if No Path found
		System.out.println("No Path found");
		return null;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		System.out.println("Starting djkstra");
		// Create initial Nodes & required lists and queues
				MapNode startNode = getNodefromLocation.get(start);
				MapNode destNode = getNodefromLocation.get(goal);
				HashSet<MapNode> visited = new HashSet<MapNode>();
				Comparator<MapNode> comparator = new PQueueForDijkstra();
				PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>(10, comparator);
				
				// holds current Node while searching
				MapNode currentNode = startNode;
				for(GeographicPoint currentPoint : setOfVertices ) {
					MapNode curr = getNodefromLocation.get(currentPoint);
					curr.setDistanceOfNode(Integer.MAX_VALUE);
				}
				// Add start to queue & continue until all Nodes are explored
				currentNode.setDistanceOfNode(0);
				int countNoOfNodes = 0;
				queue.add(startNode);
				while(!queue.isEmpty()) {
					currentNode = queue.remove();
					countNoOfNodes++;
//					System.out.println(currentNode.getGeoPoint()); // for debugging
//					System.out.println(queue);
					// Only if Node is not visited process it or else fetch the next Node from queue
					if(!visited.contains(currentNode)) {
						visited.add(currentNode);
						nodeSearched.accept(currentNode.getGeoPoint()); // for visualizing on the front-end interface
						// If X, Y of current is same as X, Y of start : start backtracking and find path
						if(currentNode.getGeoPoint().getX() == goal.getX() && currentNode.getGeoPoint().getY() == goal.getY()) {
							System.out.println("Nodes visited :"+visited.size());
							System.out.println("Nodes dequeued : "+countNoOfNodes);
							return searchedPath(currentNode, start);
						}
						// for every Neighbour of the current Node:
						for(MapNode currentNeighbour : currentNode.getNeighbours()) {
							// If Node not visited only then process or else check next
							if(!visited.contains(currentNeighbour)) {
								double edgeLengthofNode = currentNode.getEdgeLength().get(currentNeighbour);
								if(currentNode.getDistanceOfNode() + edgeLengthofNode < currentNeighbour.getDistanceOfNode()) {
									double newNodeDistance = currentNode.getDistanceOfNode() + edgeLengthofNode;
									currentNeighbour.setDistanceOfNode(newNodeDistance);
									currentNeighbour.setParent(currentNode);
									// add to queue
									queue.add(currentNeighbour);
								}
								
								
								
								
							}
							
						}
					}
				}
				// Return null if No Path found
				System.out.println("No Path found");
				return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		// Create initial Nodes & required lists and queues
		System.out.println("starting A star search");
		MapNode startNode = getNodefromLocation.get(start);
		MapNode destNode = getNodefromLocation.get(goal);
		HashSet<MapNode> visited = new HashSet<MapNode>();
		Comparator<MapNode> comparator = new PQueueForAStar();
		PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>(10, comparator);
		for(GeographicPoint currentPoint : setOfVertices ) {
			MapNode curr = getNodefromLocation.get(currentPoint);
			curr.setDistanceOfNode(Integer.MAX_VALUE);
			curr.setFvalue(Integer.MAX_VALUE);
		}
		// holds current Node while searching
		MapNode currentNode = startNode;
		// Add start to queue & continue until all Nodes are explored
		currentNode.setDistanceOfNode(0);
		currentNode.setFvalue(currentNode.getGeoPoint().distance(goal));
		double totaldistance = 0;
		int countNoOfNodes = 0;
		queue.add(startNode);
		while(!queue.isEmpty()) {
			currentNode = queue.remove();
			countNoOfNodes++;
			totaldistance = currentNode.getFvalue(); 
//			System.out.println(currentNode.getGeoPoint()+" total distance: "+totaldistance); // for debugging
//			System.out.println(queue);
			// Only if Node is not visited process it or else fetch the next Node from queue
			if(!visited.contains(currentNode)) {
				visited.add(currentNode);
				nodeSearched.accept(currentNode.getGeoPoint()); // for visualizing on the front-end interface
				// If X, Y of current is same as X, Y of start : start backtracking and find path
				if(currentNode.getGeoPoint().getX() == goal.getX() && currentNode.getGeoPoint().getY() == goal.getY()) {
					System.out.println("Nodes visited :"+visited.size());
					System.out.println("Nodes dequeued : "+countNoOfNodes);
					return searchedPath(currentNode, start);
				}
				// for every Neighbour of the current Node:
				for(MapNode currentNeighbour : currentNode.getNeighbours()) {
					// If Node not visited only then process or else check next
					double edgeLengthofNode = currentNode.getGeoPoint().distance(currentNeighbour.getGeoPoint());
					double distanceFromGoal = currentNeighbour.getGeoPoint().distance(goal);
					double parentNodeDistance = currentNode.getDistanceOfNode();
					double thisNodeDistance = currentNeighbour.getDistanceOfNode();
					double Gvalue = edgeLengthofNode+parentNodeDistance;
					double Hvalue = distanceFromGoal;
					double Fvalue = Gvalue+Hvalue;
					if(!visited.contains(currentNeighbour)) {
						
						System.out.println("For "+currentNeighbour.getGeoPoint()+" G:"+Gvalue+", H:"+Hvalue+" = "+Fvalue +" < "+currentNeighbour.getFvalue());
						System.out.println();
						if((Fvalue) < currentNeighbour.getFvalue()) {
							double newNodeDistance = Gvalue;
							currentNeighbour.setDistanceOfNode(newNodeDistance);
							currentNeighbour.setParent(currentNode);
							
							// add to queue
							currentNeighbour.setFvalue(Fvalue);	
							queue.add(currentNeighbour);
							
						}
						
					}
					
				}
				System.out.println("Queue : "+queue);
			}
		}
		// Return null if No Path found
		System.out.println("No Path found");
		return null;
	}

	
	/** Find the path from goal to start using parent of each Node
	 * 
	 * @param MapNode The found destination
	 * @param GeographicPoint The start point
	 * @return The list of GeographicPoint that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	private List<GeographicPoint> searchedPath(MapNode currentNeighbour, GeographicPoint start) {
		// TODO helper method
		System.out.println("Path found, backtracking");
		List<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode currentNode = currentNeighbour;
		// Add the new Node to start of the list, hence the last Node will always appear first
		path.add(0, currentNode.getGeoPoint());
		// Get parent of current Node until you find the start Node
		while(currentNode.getGeoPoint().getX() != start.getX() || currentNode.getGeoPoint().getY() != start.getY()) {
			currentNode = currentNode.getParent();
			path.add(0, currentNode.getGeoPoint());
		}
		
		return path;
	}
	
	/** Find the path from start to goal using the list from any Search
	 * 
	 * @paramList<GeographicPoint> list of locations that make the path
	 * @return nothing
	 */
	private void printPath(List<GeographicPoint> path) {
		for(GeographicPoint currentPoint : path) {
			System.out.print(currentPoint+" -> ");
		}
		System.out.println();
	}

	public static void main(String[] args)
	{
//		System.out.print("Making a new map...");
//		MapGraph firstMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
//		System.out.println("DONE.");
//		
//		GeographicPoint testStart1 = new GeographicPoint(1.0, 1.0);
//		GeographicPoint testEnd1 = new GeographicPoint(8.0, -1.0);
//		
//		List<GeographicPoint> getPath = firstMap.bfs(testStart1, testEnd1);
//		firstMap.printPath(getPath);
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		GeographicPoint test1 = new GeographicPoint(4.0, 0.0);
		GeographicPoint test2 = new GeographicPoint(4.0, 2.0);
		GeographicPoint test3 = new GeographicPoint(5.0, 1.0);
		GeographicPoint test4 = new GeographicPoint(7.0, 3.0);
		System.out.println("Queue test");
		MapNode Node1 = new MapNode(test1);
		Node1.setFvalue(903);
		MapNode Node2 = new MapNode(test2);
		Node2.setFvalue(999);
		MapNode Node3 = new MapNode(test3);
		Node3.setFvalue(845);
		MapNode Node4 = new MapNode(test4);
		Node4.setFvalue(1189);
		Comparator<MapNode> comparator = new PQueueForAStar();
		PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>(10, comparator);
		System.out.println(queue);
		queue.add(Node1);
		System.out.println(queue);

		queue.add(Node2);
		System.out.println(queue);

		queue.add(Node3);
		System.out.println(queue);

		queue.add(Node4);
		System.out.println(queue);
		queue.remove();
		System.out.println(queue+"\n\n\n");
		
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		
		System.out.println("testroute2 "+testroute2);
		simpleTestMap.printPath(testroute);
		simpleTestMap.printPath(testroute2);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		testMap.printPath(testroute);
		testMap.printPath(testroute2);
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		testMap.printPath(testroute);
		
		
		/* Use this code in Week 3 End of Week Quiz */
		System.out.println("Week 3 test!!!!!!!!!!!!\n\n\n\n\n");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		theMap.printPath(route);
		theMap.printPath(route2);
		
		
	}
	
}
