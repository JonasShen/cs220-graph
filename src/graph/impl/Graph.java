package graph.impl;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.Stack;

import graph.IGraph;
import graph.INode;
import graph.NodeVisitor;

class Path implements Comparable<Path> {
	private INode destination;
	private int cost;

	public Path(INode destination, int cost) {
		this.destination = destination;
		this.cost = cost;
	}

	public int compareTo(Path path) {
		return this.cost - path.cost;
	}

	// getters
	public INode getDestination() {
		return destination;
	}

	public int getCost() {
		return cost;
	}
}

class Edge implements Comparable<Edge> {
	private INode src;
	private INode dst;
	private int weight;

	public Edge(INode src, INode dst, int weight) {
		this.src = src;
		this.dst = dst;
		this.weight = weight;
	}

	public int compareTo(Edge ed) {
		return this.weight - ed.weight;
	}

	// getters
	public INode getSrc() {
		return src;
	}

	public INode getDst() {
		return dst;
	}

	public int getWeight() {
		return weight;
	}
}

/**
 * A basic representation of a graph that can perform BFS, DFS, Dijkstra, and
 * Prim-Jarnik's algorithm for a minimum spanning tree.
 * 
 * @author jspacco
 *
 */
public class Graph implements IGraph {
	private Map<String, INode> nodes = new HashMap<>();

	/**
	 * Return the {@link Node} with the given name.
	 * 
	 * If no {@link Node} with the given name exists, create a new node with the
	 * given name and return it. Subsequent calls to this method with the same name
	 * should then return the node just created.
	 * 
	 * @param name
	 * @return
	 */
	public INode getOrCreateNode(String name) {
		if (nodes.containsKey(name))
			return nodes.get(name);

		INode iNode = new Node(name);
		this.nodes.put(name, iNode);
		return iNode;
	}

	/**
	 * Return true if the graph contains a node with the given name, and false
	 * otherwise.
	 * 
	 * @param name
	 * @return
	 */
	public boolean containsNode(String name) {
		if (nodes.containsKey(name))
			return true;
		return false;
	}

	/**
	 * Return a collection of all of the nodes in the graph.
	 * 
	 * @return
	 */
	public Collection<INode> getAllNodes() {
		return nodes.values();
	}

	/**
	 * Perform a breadth-first search on the graph, starting at the node with the
	 * given name. The visit method of the {@link NodeVisitor} should be called on
	 * each node the first time we visit the node.
	 * 
	 * 
	 * @param startNodeName
	 * @param v
	 */
	public void breadthFirstSearch(String startNodeName, NodeVisitor v) {
		HashSet<INode> visited = new HashSet<>();
		Queue<INode> toVisit = new LinkedList<>();
		toVisit.add(getOrCreateNode(startNodeName));
		while (!toVisit.isEmpty()) {
			INode tempNode = toVisit.poll();
			if (visited.contains(tempNode))//node visited
				continue;
			
			v.visit(tempNode);
			visited.add(tempNode);

			for (INode node : tempNode.getNeighbors()) {//add neighbors
				if (!visited.contains(node))
					toVisit.offer(node);
			}
		}
	}

	/**
	 * Perform a depth-first search on the graph, starting at the node with the
	 * given name. The visit method of the {@link NodeVisitor} should be called on
	 * each node the first time we visit the node.
	 * 
	 * 
	 * @param startNodeName
	 * @param v
	 */
	public void depthFirstSearch(String startNodeName, NodeVisitor v) {
		HashSet<INode> visited = new HashSet<>();
		Stack<INode> toVisit = new Stack<>();
		toVisit.push(getOrCreateNode(startNodeName));

		while (!toVisit.isEmpty()) {
			INode tempNode = toVisit.pop();
			if (visited.contains(tempNode))//node visited
				continue;
			v.visit(tempNode);
			visited.add(tempNode);

			for (INode node : tempNode.getNeighbors()) {//add neighbors 
				if (!visited.contains(node))
					toVisit.push(node);
			}
		}
	}

	/**
	 * Perform Dijkstra's algorithm for computing the cost of the shortest path to
	 * every node in the graph starting at the node with the given name. Return a
	 * mapping from every node in the graph to the total minimum cost of reaching
	 * that node from the given start node.
	 * 
	 * <b>Hint:</b> Creating a helper class called Path, which stores a destination
	 * (String) and a cost (Integer), and making it implement Comparable, can be
	 * helpful. Well, either than or repeated linear scans.
	 * 
	 * @param startName
	 * @return
	 */
	public Map<INode, Integer> dijkstra(String startName) {
		// TODO: Implement this method
		Map<INode, Integer> result = new HashMap<>();
		PriorityQueue<Path> toDo = new PriorityQueue<>();
		
		toDo.add(new Path(getOrCreateNode(startName), 0));//create first node

		while (result.size() < getAllNodes().size()) {
			Path nextPath = toDo.poll();
			INode node = nextPath.getDestination();
			if (result.containsKey(node)) //node visited
				continue;
			
			int cost = nextPath.getCost();
			result.put(node, cost);// add visited node
			
			for (INode n : node.getNeighbors()) {
				toDo.add(new Path(n, node.getWeight(n) + cost));
			}
		}
		return result;
	}

	/**
	 * Perform Prim-Jarnik's algorithm to compute a Minimum Spanning Tree (MST).
	 * 
	 * The MST is itself a graph containing the same nodes and a subset of the edges
	 * from the original graph.
	 * 
	 * @return
	 */
	public IGraph primJarnik() {
		// TODO Implement this method
		IGraph graph = new Graph();
        PriorityQueue<Edge> toDo = new PriorityQueue<>();
        
        INode srcNode = nodes.values().iterator().next(); //Start from next node, actually it's random
        
        for(INode node : srcNode.getNeighbors()) //Add all edges
        	toDo.add(new Edge(srcNode, node, srcNode.getWeight(node)));
        
        while (graph.getAllNodes().size() < this.getAllNodes().size()) { 
        	Edge nextEdge = toDo.poll();
        	//Retrieve start and end nodes from the current edge
        	INode startNode = nextEdge.getSrc(); 
        	String startName = startNode.getName();
        	INode endNode = nextEdge.getDst();
        	String endName = endNode.getName();
        	
        	if(graph.containsNode(startName) && graph.containsNode(endName)) 
        		continue;
        	
        	INode lastStart = graph.getOrCreateNode(startName); 
        	INode lastEnd = graph.getOrCreateNode(endName);
        	lastStart.addUndirectedEdgeToNode(lastEnd, nextEdge.getWeight()); //Add the last start and end nodes into the new graph
        	
        	for(INode node : endNode.getNeighbors()) //Add neighbors to the queue
        		if(!node.equals(startNode))
        			toDo.add(new Edge(endNode,node,endNode.getWeight(node)));
 
        }
        
        return graph;
	}
}