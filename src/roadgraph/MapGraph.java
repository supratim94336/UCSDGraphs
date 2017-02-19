/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;
import java.io.*;
import java.util.*;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import com.google.common.collect.Table;
import geography.GeographicPoint;
import util.GraphLoader;
import com.google.common.collect.HashBasedTable;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
    /**
     * A HashMap is created to get hold of corresponding mappings between a location and a MapNode class
     */
	private HashMap<GeographicPoint, MapNode> vertexId;
	private HashSet<MapEdge> edges;
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph() {
		// TODO: Implement in this constructor in WEEK 3
		vertexId = new HashMap<>();
		edges = new HashSet<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		//TODO: Implement this method in WEEK 3
		return vertexId.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		//TODO: Implement this method in WEEK 3
		return vertexId.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		//TODO: Implement this method in WEEK 3
		return edges.size();
	}
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		// TODO: Implement this method in WEEK 3
        if(vertexId.get(location)!= null || location == null){
            System.out.println("Location already exists or not a valid location");
            return false;
        }
        vertexId.put(location, new MapNode(location, new ArrayList<>()));
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
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
            throws IllegalArgumentException {
	    //TODO: Implement this method in WEEK 3
        if(from==null||to==null||roadName==null||roadType==null||length<0||vertexId.get(from)==null||
                vertexId.get(to)==null){
            throw new IllegalArgumentException();
        }
		MapNode fromNode = vertexId.get(from);
        MapNode toNode = vertexId.get(to);
        addEdge(fromNode, toNode, roadName, roadType, length);
	}

    private void addEdge(MapNode fromNode, MapNode toNode, String roadName, String roadType, double length) {
        MapEdge edge = new MapEdge(fromNode, toNode, roadName, roadType, length);
        edges.add(edge);
        fromNode.getEdges().add(edge);
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
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint>
            nodeSearched) {
		// TODO: Implement this method in WEEK 3
        if(start==null||goal==null){
            throw new IllegalArgumentException();
        }

        MapNode startNode = vertexId.get(start);
        MapNode endNode = vertexId.get(goal);

		Deque<MapNode> queue = new LinkedList<>();
		HashSet<MapNode> visited = new HashSet<>();
		HashMap<MapNode,MapNode> parent = new HashMap<>();

		queue.addLast(startNode);
		visited.add(startNode);
		boolean found = false;

		while(!queue.isEmpty()){
		    MapNode curr = queue.remove();
            nodeSearched.accept(curr.getLocation());
		    if(curr == endNode){
		        found = true;
		        break;
            }
            for (MapNode neighbor : getNeighbors(curr)) {
                if (!visited.contains(neighbor)) {
                    visited.add(neighbor);
                    parent.put(neighbor, curr);
                    queue.add(neighbor);
                }
            }
        }
        if (!found) {
            System.out.println("No path exists");
            //return new ArrayList<GeographicPoint>();
            return null;
        }

        LinkedList<GeographicPoint> path = new LinkedList<>();
        MapNode currNode = endNode;
        while (currNode!= startNode) {
            path.addFirst(currNode.getLocation());
            currNode = parent.get(currNode);
        }
        path.addFirst(startNode.getLocation());
        return path;
	}
    /**
     * A method to get the neighbors of a particular node
     */
    private Set<MapNode> getNeighbors(MapNode node) {
        return node.getNeighbors();
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

    public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint>
            nodeSearched) {
        return searchOnWeightedGraph(start, goal, nodeSearched, (a,b) -> 0.0);
    }

    public List<GeographicPoint> searchOnWeightedGraph(GeographicPoint start, GeographicPoint goal,
                                                       Consumer<GeographicPoint> nodeSearched, BiFunction<MapNode,
            MapNode, Double> f) {
		// TODO: Implement this method in WEEK 4
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
        if(start==null||goal==null){
            throw new IllegalArgumentException();
        }

        MapNode startNode = vertexId.get(start);
        MapNode endNode = vertexId.get(goal);

        PriorityQueue<MapNode> queue = new PriorityQueue<>();
        HashSet<MapNode> visited = new HashSet<>();
        HashMap<MapNode,MapNode> parent = new HashMap<>();

        initializeInfinityMap();

        startNode.setDistanceFromLast(0.0);
        startNode.setDistance(0.0);

        queue.add(startNode);
        boolean found = false;
        MapNode curr;

        while(!queue.isEmpty()){
            curr = queue.poll();
            if(!visited.contains(curr)) {
                visited.add(curr);
                nodeSearched.accept(curr.getLocation());
                if (curr == endNode) {
                    found = true;
                    break;
                }
                HashMap<MapNode, Double> distanceMap = getDistanceMap(curr);
                for (MapNode neighbor : getNeighbors(curr)) {
                    Double actualDistance = curr.getDistanceFromLast() + distanceMap.get(neighbor);
                    if (actualDistance < neighbor.getDistanceFromLast()) {
                        neighbor.setDistanceFromLast(actualDistance);
                        actualDistance += f.apply(neighbor,endNode);
                        neighbor.setDistance(actualDistance);
                        parent.put(neighbor, curr);
                        queue.offer(neighbor);
                    }
                }
            }
        }

        if (!found) {
            System.out.println("No path exists");
            //return new ArrayList<GeographicPoint>();
            return null;
        }

        System.out.println(visited.size());
        LinkedList<GeographicPoint> path = new LinkedList<>();
        MapNode currNode = endNode;
        while (currNode!= startNode) {
            path.addFirst(currNode.getLocation());
            currNode = parent.get(currNode);
        }

        path.addFirst(startNode.getLocation());
        return path;
	}


    /**
     * Here we initialize each node in the weighted map
     * having distance value of infinity
     */
    private void initializeInfinityMap(){
	    for(MapNode m: vertexId.values()){
            m.setDistanceFromLast(Double.MAX_VALUE);
            m.setDistance(Double.MAX_VALUE);
        }
    }

    private HashMap<MapNode, Double> getDistanceMap(MapNode m){
        return m.getDistanceMap();
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
        return searchOnWeightedGraph(start, goal, nodeSearched, (a,b) -> a.getLocation().distance(b.getLocation()));
	}

	
	
	public static void main(String[] args) throws IOException{
		System.out.print("Making a new map...");
        Scanner scan = new Scanner(System.in);
        Table<GeographicPoint, GeographicPoint, String> routemap = HashBasedTable.create();
		//MapGraph firstMap = new MapGraph();
		//System.out.print("DONE. \nLoading the map...");
		//GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		//System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */

		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		List<GeographicPoint> routeu = simpleTestMap.bfs(new GeographicPoint(1.0, 1.0),
                new GeographicPoint(8.0, -1.0));
        System.out.println(routeu);

        System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
        System.out.println(testroute.toString());
        System.out.println(testroute2.toString());
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
        System.out.println(testroute.toString());
        System.out.println(testroute2.toString());
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
        System.out.println(testroute.toString());
        System.out.println(testroute2.toString());

		
		//Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");
        /*while(true) {
            System.out.println("Where you do you want to go?");
            System.out.println("\nEnter Starting Latitude: ");
            String s1 = scan.next();
            double lat1 = Double.parseDouble(s1);
            System.out.println("\nEnter Starting Longitude: ");
            String s2 = scan.next();
            double lon1 = Double.parseDouble(s2);
            System.out.println("\nEnter Ending Latitude: ");
            String s3 = scan.next();
            double lat2 = Double.parseDouble(s3);
            System.out.println("\nEnter Ending Longitude: ");
            String s4 = scan.next();
            double lon2 = Double.parseDouble(s4);

            GeographicPoint start = new GeographicPoint(lat1, lon1);
            GeographicPoint end = new GeographicPoint(lat2, lon2);

            if (routemap.contains(start, end)) {
                System.out.println("You have chosen a known location: " + routemap.get(start, end));
                System.out.println("\nStill want to continue: ");
            } else {
                String path = theMap.aStarSearch(start, end).toString();
                routemap.put(start, end, path);
                System.out.println("Well this is a new search and we are saving it for future: " + path.toString());
            }
        }*/
		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

	}
	
}
