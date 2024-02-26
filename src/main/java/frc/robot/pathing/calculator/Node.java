package frc.robot.pathing.calculator;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Stack;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Predicate;

public class Node implements Comparable<Node> {
    public int x;
    public int y;
    public double totalCost;
    public Node parent;

    /**
     * @-1 = in closed
     * @0 = doesnt exist
     * @1 = in open
     */
    public byte curListState;

    public Node(int x, int y, Node parent) {
        this.x = x;
        this.y = y;
        this.parent = parent;
    }

    public HashSet<Node> getNodesAround(double radius, int boardWidth, byte[] grid, Predicate<Byte> isObstructed) {
        HashSet<Node> nodesInsideCircle = new HashSet<>();
        int centerX = this.x;
        int centerY = this.y;

        for (int x = centerX - (int) radius; x <= centerX + radius; x++) {
            for (int y = centerY - (int) radius; y <= centerY + radius; y++) {
                if (this.x == x && this.y == y) continue;

                Node n = new Node(x, y, this);

                if (isInsideCircle(n, this, radius)) {
                    int pos = n.convertTo1D(boardWidth);
                    if (pos < grid.length && pos >= 0) {
                        if (!isObstructedBetweenPoints(n, grid, boardWidth, isObstructed)) {
                            nodesInsideCircle.add(n);
                        }
                    }
                }
            }
        }

        return nodesInsideCircle;
    }


    private boolean isInsideCircle(Node n, Node node, double radius) {
        return n.getDistanceBetween(node) <= radius;
    }

    public List<Byte> getNodesAround(int xAdd, int yAdd, byte[] grid, int boardWidth) {
        List<Byte> nodes = new ArrayList<>();
        for (int x = -xAdd; x <= xAdd; x++) {
            for (int y = -yAdd; y <= yAdd; y++) {
                int pos = convertTo1D(boardWidth, x, y);

                if (pos < boardWidth && pos >= 0) {
                    nodes.add(grid[pos]);
                }
            }
        }

        return nodes;
    }

    public int convertTo1D(int boardWidth) {
        return (this.y * boardWidth + this.x);
    }

    public int convertTo1D(int boardWidth, int x, int y) {
        return (y * boardWidth + x);
    }

    public Stack<Node> reverse() {
        Stack<Node> stack = new Stack<>();
        Node curNode = this;
        while (curNode.parent != null) {
            stack.add(curNode);
            curNode = curNode.parent;
        }

        return stack;
    }

    public double getDistanceBetween(Node n) {
        return Math.sqrt((n.x - x) * (n.x - x) + (n.y - y) * (n.y - y));
    }

    public void genCosts(byte[] grid, int gridWidth, Node end, Predicate<Byte> isObstructed) {
        double hCost = this.getDistanceBetween(end);
        double gCost = 0;

        AtomicInteger closed = new AtomicInteger();
        this.getNodesAround(2, 2, grid, gridWidth).forEach((a) -> {
            if (isObstructed.test(a)) closed.incrementAndGet();
        });

        gCost += closed.get() * 2;
        if (this.parent != null) {
            gCost += parent.getDistanceBetween(this);
        }

        this.totalCost = gCost + hCost;
    }

    @Override
    public boolean equals(Object o) {
        Node o_node = (Node) o;
        return o_node == this || (o_node.x == x && o_node.y == y);
    }

    @Override
    public int compareTo(Node o) {
        return this.totalCost < o.totalCost ? -1 : 1;
    }

    public boolean isObstructedBetweenPoints(Node point2, byte[] values, int width, Predicate<Byte> predicate) {
        double x1 = this.x;
        double y1 = this.y;
        double x2 = point2.x;
        double y2 = point2.y;

        double dx = Math.abs(x2 - x1);
        double dy = Math.abs(y2 - y1);

        double sx = (float) ((x1 < x2) ? 0.5 : -0.5);
        double sy = (float) ((y1 < y2) ? 0.5 : -0.5);

        double err = dx - dy;

        while (true) {
            int index = (int) (y1 * width + x1);

            if (index >= 0 && index < values.length) {
                if (predicate.test(values[index])) {
                    return true;
                }
            }

            if (x1 == x2 && y1 == y2) {
                break;
            }

            double e2 = 2 * err;

            if (e2 > -dy) {
                err -= dy;
                x1 += sx;
            }

            if (e2 < dx) {
                err += dx;
                y1 += sy;
            }

            if (x1 != point2.x || y1 != point2.y) {
                int lineIndex = (int) (y1 * width + x1);
                if (predicate.test(values[lineIndex])) {
                    return true;
                }
            }
        }

        return false;
    }

}
