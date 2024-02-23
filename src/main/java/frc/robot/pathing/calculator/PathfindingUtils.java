package frc.robot.pathing.calculator;

public class PathfindingUtils {
    public static Node[] getNodeGrid(byte[] grid, int width) {
        Node[] nodes = new Node[grid.length];
        for (int i = 0; i < grid.length; i++) {
            int[] pos = posFromIndex(i, width);
            nodes[i] = new Node(pos[0], pos[1], null);
        }

        return nodes;
    }

    public static int[] posFromIndex(int index, int width) {
        int x = index / width;
        int y = index % width;
        return new int[]{x, y};
    }
}
