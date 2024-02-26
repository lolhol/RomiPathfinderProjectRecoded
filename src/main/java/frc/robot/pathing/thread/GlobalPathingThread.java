package frc.robot.pathing.thread;

import frc.robot.pathing.calculator.AStarCalculator;
import frc.robot.pathing.calculator.Node;

import java.util.Stack;
import java.util.function.Predicate;

public class GlobalPathingThread extends Thread {
    public boolean isDone;

    Stack<Node> nodes = null;

    final AStarCalculator calculator;
    final int[] start, end;
    final double scaleFactorMap;
    final byte[] map;
    final int mapWidth, maxIter;
    final Predicate<Byte> isObstructed;

    public GlobalPathingThread(int[] startPos, int[] endPos, double scaleFactorMap, byte[] map, int mapWidth,
                               int maxIter, Predicate<Byte> isObstructed) {
        this.calculator = new AStarCalculator();
        this.start = startPos;
        this.end = endPos;
        this.scaleFactorMap = scaleFactorMap;
        this.map = map;
        this.mapWidth = mapWidth;
        this.maxIter = maxIter;
        this.isDone = false;
        this.isObstructed = isObstructed;
    }

    // FIXME: scaleFactorMap
    @Override
    public void run() {
        Node result = calculator.run(shortenGrid(map, (int) scaleFactorMap, isObstructed),
                (int) (mapWidth / scaleFactorMap),
                new Node(start[0], start[1],
                        null), new Node(end[0],
                        end[1],
                        null), maxIter, 1, aByte -> aByte == 1);

        if (result != null) {
            nodes = result.reverse();
        }

        isDone = true;
    }

    private byte[] shortenGrid(byte[] grid, int division, Predicate<Byte> isObstructed) {
        byte[] result = new byte[grid.length / division];
        int curI = 0;
        int currentPosNew = 0;
        int obstructions = 0;
        for (byte b : grid) {
            if (isObstructed.test(b)) {
                obstructions++;
            }

            if (curI == division) {
                result[currentPosNew] = (byte) (obstructions == 0 ? 0 : 1);
                curI = 0;
                currentPosNew++;
                obstructions = 0;
            } else {
                curI++;
            }
        }

        return result;
    }

    public Stack<Node> getProgress() {
        return nodes;
    }
}
