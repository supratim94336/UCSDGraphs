package roadgraph;

import geography.GeographicPoint;

import java.util.*;

/**
 * Created by Supra on 28/12/2016.
 */
public class MapNode implements Comparable{
    /**
     * A MapNode is a vertex on a map (which is nothing but a location having latitude and longitude values)
     * MapNode have a set of edges to other vertices
     * It has a distance which is
     */
    private GeographicPoint location;
    private List<MapEdge> edges;

    private Double distance;
    private Double distanceFromLast;

    /**
     * Constructor to initiate a MapNode
     */
    public MapNode(GeographicPoint location, List<MapEdge> edges){
        this.location=location;
        this.edges = edges;
    }

    public MapNode(GeographicPoint location){
        this.location = location;
        edges = new ArrayList<>();
        distance = 0.0;
        distanceFromLast = 0.0;
    }

    /**
     * Corresponding getters and setters
     */
    public GeographicPoint getLocation() {
        return location;
    }

    public void setLocation(GeographicPoint location) {
        this.location = location;
    }

    public Double getDistance() {
        return distance;
    }

    public void setDistance(Double distance) {
        this.distance = distance;
    }

    public Double getDistanceFromLast() {
        return distanceFromLast;
    }

    public void setDistanceFromLast(Double distanceFromLast) {
        this.distanceFromLast = distanceFromLast;
    }

    public List<MapEdge> getEdges() {
        return edges;

    }

    public void setEdges(List<MapEdge> edges) {
        this.edges = edges;
    }

    /**
     * Method to get the neighbors of a node
    */
    public Set<MapNode> getNeighbors(){
        Set<MapNode> neighbors = new HashSet<>();
        for (MapEdge edge : edges) {
            neighbors.add(edge.getOtherNode(this));
        }
        return neighbors;
    }

    public int compareTo(Object o) {
        MapNode m = (MapNode) o;
        return (this.distance).compareTo(m.distance);
    }

    /**
     * Get neighbors and their distances
    */
    public HashMap<MapNode, Double> getDistanceMap(){
        HashMap<MapNode, Double> lengthAndNodes = new HashMap<>();
        for(MapEdge e: this.getEdges()){
            lengthAndNodes.put(e.getOtherNode(this), e.getDist());
        }
        return lengthAndNodes;
    }
}
