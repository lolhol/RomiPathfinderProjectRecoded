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
        Node result =
                calculator.run(scaleGrid(map, mapWidth, map.length / mapWidth, (int) scaleFactorMap, isObstructed),
                        (int) (mapWidth / scaleFactorMap),
                        new Node((int) (start[0] / scaleFactorMap), (int) (start[1] / scaleFactorMap),
                                null), new Node((int) (end[0] / scaleFactorMap), (int) (end[1] / scaleFactorMap),
                                null), maxIter, 1.5, aByte -> aByte > 0);

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

    public byte[] scaleGrid(byte[] originalGrid, int originalWidth, int originalHeight, int division,
                            Predicate<Byte> isObstructed) {
        int scaledWidth = originalWidth / division;
        int scaledHeight = originalHeight / division;
        byte[] scaledGrid = new byte[scaledWidth * scaledHeight];

        for (int y = 0; y < scaledHeight; y++) {
            for (int x = 0; x < scaledWidth; x++) {
                int originalX = x * division;
                int originalY = y * division;

                scaledGrid[y * scaledWidth + x] = (byte) (isObstructed(originalGrid, originalWidth, originalX,
                        originalY, division,
                        isObstructed) ? 1 : 0);
            }
        }

        return scaledGrid;
    }

    private boolean isObstructed(byte[] originalGrid, int originalWidth, int startX, int startY, int division,
                                 Predicate<Byte> isObstructed) {
        for (int y = startY; y < startY + division; y++) {
            for (int x = startX; x < startX + division; x++) {
                if (x < originalWidth && y < originalGrid.length / originalWidth &&
                        isObstructed.test(originalGrid[y * originalWidth + x])) {
                    return true;
                }
            }
        }

        return false;
    }

    public Stack<Node> getProgress() {
        return nodes;
    }
}
