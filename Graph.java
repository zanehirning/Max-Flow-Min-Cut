import java.util.*;

public class Graph {
    private final GraphNode[] vertices;  // Adjacency list for graph.
    private final String name;  //The file from which the graph was created.
    private int[][] residual;

    public Graph(String name, int vertexCount) {
        this.name = name;

        vertices = new GraphNode[vertexCount];
        for (int vertex = 0; vertex < vertexCount; vertex++) {
            vertices[vertex] = new GraphNode(vertex);
        }
    }

    public boolean addEdge(int source, int destination, int capacity) {
        // A little bit of validation
        if (source < 0 || source >= vertices.length) return false;
        if (destination < 0 || destination >= vertices.length) return false;

        // This adds the actual requested edge, along with its capacity
        vertices[source].addEdge(source, destination, capacity);
        vertices[destination].addEdge(destination, source, 0);

        return true;
    }

    /**
     * Algorithm to find max-flow in a network
     */
    public int findMaxFlow(int s, int t, boolean report) {
        System.out.printf("-- Max Flow: %s --\n", name);

        int totalFlow = 0;
        residual = new int[vertices.length][vertices.length];

        for (int i=0; i<vertices.length; i++) {
            for (int j=0; j<vertices.length; j++) {
                residual[i][j] = -1;
            }
        }

        for (int i = 0; i < vertices.length; i++) {
            for (GraphNode.EdgeInfo edge : vertices[i].successor) {
                residual[edge.from][edge.to] = edge.capacity;
            }
        }

        while (hasAugmentingPath(s, t)) {
            int availableFlow = Integer.MAX_VALUE;
            int v = t;
            while (vertices[v].visited) {
                availableFlow = Math.min(availableFlow, residual[vertices[v].parent][v]);
                v = vertices[v].parent;
            }
            ArrayList<Integer> augmentedPath = new ArrayList<>();
            v = t;
            while (vertices[v].visited) {
                residual[vertices[v].parent][v] -= availableFlow;
                residual[v][vertices[v].parent] += availableFlow;
                augmentedPath.add(v);
                v = vertices[v].parent;
            }
            printPaths(s, availableFlow, augmentedPath);
            totalFlow += availableFlow;
        }
        if (report) {
            printEdges(s, t);
        }
        return totalFlow;
    }

    private void printPaths(int s, int availableFlow, ArrayList<Integer> augmentedPath) {
        System.out.printf("Flow %d: %d ", availableFlow, s);
        for (int i=0; i< augmentedPath.size(); i++) {
            System.out.printf("%d ", augmentedPath.get(augmentedPath.size()-i-1));
        }
        System.out.println();
        augmentedPath.clear();
    }

    private void printEdges(int s, int t) {
        int v = s;
        while (v!=t) {
            for (GraphNode.EdgeInfo edge:vertices[v].successor) {
                if (residual[edge.from][edge.to] < edge.capacity) {
                    System.out.printf("Edge(%d, %d) transports %d\n", edge.from, edge.to, edge.capacity - residual[edge.from][edge.to]);
                    v = edge.to;
                }
            }
        }
    }

    /**
     * Algorithm to find an augmenting path in a network
     */
    private boolean hasAugmentingPath(int s, int t) {
        for (int i=0; i < vertices.length; i++) {
            vertices[i].parent = -1;
            vertices[i].visited=false;
        }
        Queue<Integer> q = new LinkedList<>();
        q.add(s);
        while (!q.isEmpty() && vertices[t].parent==-1) {
            int v = q.remove();
            for (GraphNode.EdgeInfo edge:vertices[v].successor) {
                int w = edge.to;
                if ((residual[v][w]) > 0 && w!=s && !vertices[w].visited) {
                    vertices[w].visited=true;
                    vertices[w].parent = v;
                    q.add(w);
                }
            }
        }
        return vertices[t].parent!=-1;
    }

    private void printResidual() {
        for (int i=0; i< residual.length; i++) {
            System.out.println(Arrays.toString(residual[i]));
        }
    }

    private ArrayList<GraphNode> traversal(ArrayList<GraphNode> R, int s) {
        if (R.contains(vertices[s])) {
            return R;
        }
        R.add(vertices[s]);
        for (GraphNode.EdgeInfo edge:vertices[s].successor) {
            if (residual[edge.from][edge.to] > 0 && !R.contains(edge)) {
                traversal(R, edge.to);
            }
        }
        return R;
    }

    /**
     * Algorithm to find the min-cut edges in a network
     */
    public void findMinCut(int s) {
        // TODO:
        System.out.printf("-- Min Cut: %s --\n", name);
        //TODO:Compute the residual graph
        //find set of vertices that are reachable from source

        ArrayList<GraphNode> R = new ArrayList<>();
        R = traversal(R, s);
        ArrayList<GraphNode.EdgeInfo> pathCut = new ArrayList<>();
        //all edges from vertex in R to vertex not in R
        for (int j=0; j<R.size(); j++) {
            for (GraphNode.EdgeInfo edge:vertices[R.get(j).id].successor) {
                if (!R.contains(edge) && residual[edge.from][edge.to]==0 && residual[edge.from][edge.to]<edge.capacity) { //check if R does not contain edge and there is flow through the edge.
                    pathCut.add(edge);
                    for (int i=0; i < pathCut.size(); i++) {
                        if (edge.from == pathCut.get(i).to) {
                            pathCut.remove(pathCut.get(i));
                        }
                        if (edge.to == pathCut.get(i).from) {
                            pathCut.remove(edge);
                        }
                    }
                }
            }
        }

        for (int i=0; i < pathCut.size(); i++) {
            System.out.printf("Min Cut Edge: (%d, %d)\n", pathCut.get(i).from, pathCut.get(i).to);
        }
        System.out.println();
    }

    public String toString() {
        StringBuilder sb = new StringBuilder();

        sb.append("The Graph " + name + " \n");
        for (var vertex : vertices) {
            sb.append((vertex.toString()));
        }
        return sb.toString();
    }
}
