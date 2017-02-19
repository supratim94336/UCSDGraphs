package roadgraph;

import geography.GeographicPoint;

/**
 * Created by Supra on 28/12/2016.
 */
public class MapEdge {
    /**
     * A MapEdge is a path on a map (which is nothing but a connector having a vertex/location as a start
       and another vertex/location as an end point)
     * A MapEdge have a name (which is a street type)
     * A MapEdge have a length/distance
     */
    private String name;
    private String type;
    private double dist;
    private MapNode start;
    private MapNode end;

    public double getDist() {
        return dist;
    }

    public void setDist(double dist) {
        this.dist = dist;
    }

    /**
     * Constructor to initiate an MapEdge
     */
    public MapEdge(MapNode start, MapNode end, String roadName, String roadType, double dist) {
        this.name = roadName;
        this.start = start;
        this.end = end;
        this.type = roadType;
        this.dist = dist;
    }
    /**
     * Corresponding getters and setters
     */
    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getType() {
        return type;
    }

    public void setType(String type) {
        this.type = type;
    }

    /**
     * A method to get the opposite end of an edge with respect to a node
     */
    MapNode getOtherNode(MapNode node) {
        if (node.equals(start))
            return end;
        else if (node.equals(end))
            return start;
        throw new IllegalArgumentException();
    }
}
