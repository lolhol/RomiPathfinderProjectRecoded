package frc.robot.pathing.calculator;

import java.util.PriorityQueue;
import java.util.function.Predicate;

public class AStarCalculator {
    public Node run(byte[] grid, int width, Node startNode, Node endNode, int maxIter, double radius,
                    Predicate<Byte> isObstructed) {
        Node[] nodeGrid = new Node[grid.length];
        PriorityQueue<Node> nodePriorityQueue = new PriorityQueue<>();

        nodeGrid[startNode.convertTo1D(width)] = startNode;
        nodePriorityQueue.add(startNode);

        int curIter = 0;
        while (!nodePriorityQueue.isEmpty() && curIter < maxIter) {
            Node node = nodePriorityQueue.poll();

            if (node.equals(endNode)) {
                return node;
            }

            for (Node n : node.getNodesAround(radius, width, grid, isObstructed)) {
                int oneDPos = n.convertTo1D(width);
                if (nodeGrid[oneDPos] == null) {
                    nodeGrid[oneDPos] = n;
                } else {
                    continue;
                }

                n.genCosts(grid, width, endNode, isObstructed);

                nodeGrid[n.convertTo1D(width)].parent = node;

                nodePriorityQueue.add(n);
            }

            curIter++;
        }

        return null;
    }
}
