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

    @Override
    public void run() {
        Node result = calculator.run(map, mapWidth, new Node(start[0], start[1], null), new Node(end[0], end[1],
                null), maxIter, map.length / scaleFactorMap, isObstructed);
        
        if (result != null) {
            nodes = result.reverse();
        }

        isDone = true;
    }

    public Stack<Node> getProgress() {
        return nodes;
    }
}
