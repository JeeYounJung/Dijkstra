// --== CS400 File Header Information ==--
// Name: JeeYoun Jung
// Email: jjung83@wisc.edu
// Group and Team: BG, RED
// TA: <Naman Gupta>
// Lecturer: <GARY DHAL>
// Notes to Grader: <optional extra notes>

import java.util.PriorityQueue;
import java.util.Collections;
import java.util.Hashtable;
import java.util.List;
import java.util.LinkedList;
import java.util.NoSuchElementException;
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.Rule;
import org.junit.jupiter.api.Test;
import org.junit.rules.ExpectedException;
import org.junit.runner.RunWith;

/**
 * This class extends the BaseGraph data structure with additional methods for
 * computing the total cost and list of node data along the shortest path
 * connecting a provided starting to ending nodes. This class makes use of
 * Dijkstra's shortest path algorithm.
 */
public class DijkstraGraph<NodeType, EdgeType extends Number>
        extends BaseGraph<NodeType, EdgeType>
        implements GraphADT<NodeType, EdgeType> {

    /**
     * While searching for the shortest path between two nodes, a SearchNode
     * contains data about one specific path between the start node and another
     * node in the graph. The final node in this path is stored in it's node
     * field. The total cost of this path is stored in its cost field. And the
     * predecessor SearchNode within this path is referened by the predecessor
     * field (this field is null within the SearchNode containing the starting
     * node in it's node field).
     *
     * SearchNodes are Comparable and are sorted by cost so that the lowest cost
     * SearchNode has the highest priority within a java.util.PriorityQueue.
     */
    protected class SearchNode implements Comparable<SearchNode> {
        public Node node;
        public double cost;
        public SearchNode predecessor;

        public SearchNode(Node node, double cost, SearchNode predecessor) {
            this.node = node;
            this.cost = cost;
            this.predecessor = predecessor;
        }

        public int compareTo(SearchNode other) {
            if (cost > other.cost)
                return +1;
            if (cost < other.cost)
                return -1;
            return 0;
        }
    }

    /**
     * This helper method creates a network of SearchNodes while computing the
     * shortest path between the provided start and end locations. The
     * SearchNode that is returned by this method is represents the end of the
     * shortest path that is found: it's cost is the cost of that shortest path,
     * and the nodes linked together through predecessor references represent
     * all of the nodes along that shortest path (ordered from end to start).
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return SearchNode for the final end node within the shortest path
     * @throws NoSuchElementException when no path from start to end is found
     *                                or when either start or end data do not
     *                                correspond to a graph node
     */
    protected SearchNode computeShortestPath(NodeType start, NodeType end) {
        // TODO: implement in step 6
        if (!containsNode(start) || !containsNode(end)) {
            throw new NoSuchElementException("Start or end node not found");
        }

        // Node to keep track of the path from start to end
        SearchNode startSNode = new SearchNode(nodes.get(start), 0, null);

        // If start and end nodes are the same, return
        if (start.equals(end)) {
            return startSNode;
        }

        //make hash table for tracking visited nodes
        Hashtable<NodeType, SearchNode> visited = new Hashtable<>();

        //make Priority queue for keeping nodes
        PriorityQueue<SearchNode> pq = new PriorityQueue<>();
        pq.add(startSNode);
        visited.put(start, startSNode);

        while(!pq.isEmpty()) {
            SearchNode curSNode = pq.poll();
            Node curNode = curSNode.node;
            List<Edge> edges = curNode.edgesLeaving;

            // If found, return
            if (curSNode.node.data.equals(end)) {
                return curSNode;
            }

            // Put into visited
            visited.put(curNode.data, curSNode);

            // Iterate over edges
            for (int i = 0; i < edges.size(); i++) {
                Edge edge = edges.get(i);
                Node nextNode = edge.successor;
                double cost = curSNode.cost + edge.data.doubleValue();
                SearchNode nextSNode = new SearchNode(nextNode, cost, curSNode);
                
                // Check if the next node has already been visited
                if (visited.containsKey(nextNode.data)) {
                    // If cost at the visited node is greater than the current cost, replace
                    if (nextSNode.compareTo(visited.get(nextNode.data)) < 0) {
                        pq.add(nextSNode);
                        visited.replace(nextNode.data, nextSNode);
                    }
                }
                
                else {
                    pq.add(nextSNode);
                }
            }
        }

        // If did not find path, throw an exception
        throw new NoSuchElementException("No path from start to end");
    }


    /**
     * Returns the list of data values from nodes along the shortest path
     * from the node with the provided start value through the node with the
     * provided end value. This list of data values starts with the start
     * value, ends with the end value, and contains intermediary values in the
     * order they are encountered while traversing this shorteset path. This
     * method uses Dijkstra's shortest path algorithm to find this solution.
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return list of data item from node along this shortest path
     */
    public List<NodeType> shortestPathData(NodeType start, NodeType end) {
        // TODO: implement in step 7
        List<NodeType> pathData = new LinkedList<NodeType>();
        // If start and end are the same, return single element list
        if (start == end) {
            pathData.add(start);
            return pathData;
        }

        SearchNode curNode = computeShortestPath(start, end);

        // Iterate over predecessor nodes until we find the start node
        while (!curNode.node.data.equals(start)) {
            pathData.add(curNode.node.data);
            curNode = curNode.predecessor;
        }
        pathData.add(curNode.node.data);

        // Reverse the pathData
        Collections.reverse(pathData);
        
        return pathData;
    }

    /**
     * Returns the cost of the path (sum over edge weights) of the shortest
     * path freom the node containing the start data to the node containing the
     * end data. This method uses Dijkstra's shortest path algorithm to find
     * this solution.
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return the cost of the shortest path between these nodes
     */
    public double shortestPathCost(NodeType start, NodeType end) {
        return computeShortestPath(start, end).cost;
    }

    // TODO: implement 3+ tests in step 8.

    /**
     * This is the first test method
     * 
     * tests the shortest path and its cost. --> String, double
     * 
     */
    @Test
    public void test1() {
        DijkstraGraph<String, Double> tester1 = new DijkstraGraph<>();
        tester1.insertNode("A");
        tester1.insertNode("B");
        tester1.insertNode("C");
        tester1.insertNode("D");
        tester1.insertNode("E");
        tester1.insertNode("F");
        tester1.insertNode("G");
        tester1.insertNode("H");
        tester1.insertNode("I");
        tester1.insertNode("J");

        tester1.insertEdge("A", "B", 1.0);
        tester1.insertEdge("A", "H", 8.0);
        tester1.insertEdge("A", "J", 5.0);
        tester1.insertEdge("B", "J", 3.0);
        tester1.insertEdge("C", "A", 7.0);
        tester1.insertEdge("D", "G", 2.0);
        tester1.insertEdge("E", "G", 9.0);
        tester1.insertEdge("G", "E", 7.0);
        tester1.insertEdge("H", "B", 6.0);
        tester1.insertEdge("H", "I", 2.0);
        tester1.insertEdge("D", "A", 1.0);
        tester1.insertEdge("I", "E", 2.0);
        tester1.insertEdge("F", "L", 5.0);
        tester1.insertEdge("C", "E", 3.0);
        tester1.insertEdge("B", "F", 4.0);

        double expectedCost = 12.0;
        LinkedList<String> expectedPath = new LinkedList<>();
        expectedPath.add("A");
        expectedPath.add("H");
        expectedPath.add("I");
        expectedPath.add("E");

        assertEquals(expectedCost, tester1.shortestPathCost("A", "E"));
        assertEquals(expectedPath, tester1.shortestPathData("A", "E"));
    }

    /**
     * This is the second test method
     * 
     * tests the shortest path and its cost. --> Integer, double
     * 
     */
    @Test
    public void test2() {
        DijkstraGraph<Integer, Double> tester2 = new DijkstraGraph<>();
        tester2.insertNode(1);
        tester2.insertNode(2);
        tester2.insertNode(3);
        tester2.insertNode(4);
        tester2.insertNode(5);
        tester2.insertNode(6);
        tester2.insertNode(7);

        tester2.insertEdge(1, 2, 3.0);
        tester2.insertEdge(1, 4, 2.0);
        tester2.insertEdge(2, 3, 11.0);
        tester2.insertEdge(2, 5, 9.0);
        tester2.insertEdge(3, 6, 1.0);
        tester2.insertEdge(6, 7, 2.0);
        tester2.insertEdge(3, 1, 4.0);
        tester2.insertEdge(3, 5, 5.0);
        tester2.insertEdge(5, 7, 6.0);

        double expectedCost = 17.0;
        LinkedList<Integer> expectedPath = new LinkedList<>();
        expectedPath.add(1);
        expectedPath.add(2);
        expectedPath.add(3);
        expectedPath.add(6);
        expectedPath.add(7);
        assertEquals(expectedCost, tester2.shortestPathCost(1, 7));
        assertEquals(expectedPath, tester2.shortestPathData(1, 7));
    }

    /**
     * This is the third test method
     * 
     * tests the shortest path and its cost --> From 1 to 1 (one node in the path)
     * 
     */
    @Test
    public void test3() {
        DijkstraGraph<Integer, Double> tester3 = new DijkstraGraph<>();
        tester3.insertNode(1);
        tester3.insertNode(2);
        tester3.insertNode(3);

        tester3.insertEdge(1, 2, 3.0);
        tester3.insertEdge(2, 3, 2.0);
        tester3.insertEdge(3, 2, 11.0);

        double expectedCost = 0.0;
        LinkedList<Integer> expectedPath = new LinkedList<>();
        expectedPath.add(1);
        assertEquals(expectedCost, tester3.shortestPathCost(1, 1));
        assertEquals(expectedPath, tester3.shortestPathData(1, 1));

    }

    /**
     * This is rule for testing exception
     */
    @Rule
    public ExpectedException exception;

    /**
     * This is the forth test method
     * 
     * tests the No such element exception --> There is no end node
     * 
     */
    @Test
    public void exceptionTest1() {
        DijkstraGraph<Integer, Double> tester4 = new DijkstraGraph<>();
        tester4.insertNode(1);
        tester4.insertNode(2);
        tester4.insertNode(3);
        tester4.insertNode(4);
        tester4.insertNode(5);

        tester4.insertEdge(1, 2, 3.0);
        tester4.insertEdge(1, 4, 2.0);
        tester4.insertEdge(2, 3, 11.0);
        tester4.insertEdge(2, 5, 9.0);
        tester4.insertEdge(3, 6, 1.0);

        try{

            tester4.shortestPathData(1, 7);

            // should be found an exception
            exception.expect(NoSuchElementException.class);

        } catch (NoSuchElementException e){
            e.printStackTrace();
        }
    }

    /**
     * This is the fifth test method
     * 
     * tests the No such element exception --> There is no path on this tester
     * 
     */
    @Test
    public void exceptionTest2() {
        DijkstraGraph<Integer, Double> tester5 = new DijkstraGraph<>();
        tester5.insertNode(1);
        tester5.insertNode(2);
        tester5.insertNode(3);
        tester5.insertNode(4);
        tester5.insertNode(5);
        tester5.insertNode(6);

        tester5.insertEdge(1, 2, 3.0);
        tester5.insertEdge(1, 4, 2.0);
        tester5.insertEdge(2, 4, 11.0);
        tester5.insertEdge(3, 6, 1.0);
        tester5.insertEdge(5, 6, 9.0);

        try{

            tester5.shortestPathData(1, 6);

            // should be found an exception
            exception.expect(NoSuchElementException.class);

        } catch (NoSuchElementException e){
            e.printStackTrace();
        }
    }

}

