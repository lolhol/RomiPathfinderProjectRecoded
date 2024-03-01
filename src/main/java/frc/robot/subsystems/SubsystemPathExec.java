package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extern.carto.CartographerOut;
import frc.robot.pathing.calculator.Node;
import frc.robot.pathing.thread.GlobalPathingThread;
import frc.robot.render.JFrameRenderer;
import frc.robot.util.MathUtil;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Predicate;

/**
 * @apiNote going off of that the map is now infinite and we only have to check the position of the node
 */
public class SubsystemPathExec extends SubsystemBase {
    private final SubsystemPathExecInterface subsystemPathExecInterface;
    private long startPathExecTime = 0;
    private final SubsystemDrivetrain drivetrain;
    private GlobalPathingThread globalPathingThreadInitial = null;

    private boolean isOnline;
    private double[] endGoal;
    private byte[] curMap;
    private int boardWidth;
    private final int pathingRadius = 8;

    // Initial Calc
    int ticksUntilInitReCalc = 50;
    boolean reRunInitial = true;
    List<double[]> initialCalcList = null;
    double[] broken = null;
    int brokenPos = -2;

    int timeUntilCheckIfPathBroken = 0;

    double originXBefore, originYBefore, resolution;
    //

    // PathExec going
    int curInitialCalcGoingTo = -1;
    //

    CartographerOut lastOut = null;
    final Predicate<Byte> isObstructed;

    JFrameRenderer renderer;

    public SubsystemPathExec(SubsystemDrivetrain drivetrain, Predicate<Byte> isObstructed,
                             SubsystemPathExecInterface subsystemPathExecInterface, JFrameRenderer renderer) {
        this.drivetrain = drivetrain;
        this.isObstructed = isObstructed;
        this.subsystemPathExecInterface = subsystemPathExecInterface;
        this.renderer = renderer;
    }

    public void setState(boolean newState) {
        this.isOnline = newState;
    }

    public boolean setEndGoal(int[] pos) {
        if (lastOut != null) {
            if (curMap == null) {
                this.endGoal = lastOut.MapXYtoGlobal(pos);
            } else if (MathUtil.convertTo1D(boardWidth, pos[0], pos[1]) < curMap.length) {
                this.endGoal = lastOut.MapXYtoGlobal(pos);
                renderer.clearAdditionalData();
                renderer.addAdditionalInfo(endGoal);
                renderer.reDraw();

                initialCalcList = null;
                curInitialCalcGoingTo = -1;
                reRunInitial = true;
                broken = null;
                brokenPos = -2;
            }
        } else {
            return false;
        }

        return true;
    }

    public boolean setEndGoal(int[] pos, JFrameRenderer renderer, double resolution, double originX, double originY) {
        this.renderer = renderer;
        if (lastOut != null) {
            if (curMap == null) {
                this.endGoal = MathUtil.fromMapToGlobal(pos, resolution, originX, originY);
            } else if (MathUtil.convertTo1D(boardWidth, pos[0], pos[1]) < curMap.length) {
                this.endGoal = MathUtil.fromMapToGlobal(pos, resolution, originX, originY);
                renderer.clearAdditionalData();
                renderer.addAdditionalInfo(endGoal);
                renderer.reDraw();

                initialCalcList = null;
                curInitialCalcGoingTo = -1;
                reRunInitial = true;
                broken = null;
                brokenPos = -2;
            }
        } else {
            return false;
        }

        return true;
    }

    public void tick(CartographerOut out) {
        lastOut = out;
        if (!isOnline || endGoal == null) return;
        curMap = out.map;
        boardWidth = out.mapSizeX;
        int[] curPosMap = out.FromPosToMap(out.functions.GetGlobalData());

        // thread management is big :<
        if (reRunInitial) {
            if (globalPathingThreadInitial == null) {
                if (ticksUntilInitReCalc >= 50) {
                    originXBefore = out.originX;
                    originYBefore = out.originY;
                    resolution = out.resolution;

                    globalPathingThreadInitial =
                            new GlobalPathingThread(brokenPos == -2 || initialCalcList == null ? curPosMap :
                                    MathUtil.fromGlobalToMap(initialCalcList.get(brokenPos), out.resolution,
                                            out.originX, out.originY),
                                    new int[]{out.fromXToMapX(endGoal[0]), out.fromYToMapY(endGoal[1])},
                                    pathingRadius,
                                    curMap,
                                    boardWidth,
                                    curMap.length, this.isObstructed);
                    globalPathingThreadInitial.start();
                    ticksUntilInitReCalc = 0;
                } else {
                    ticksUntilInitReCalc++;
                }
            } else {
                if (!globalPathingThreadInitial.isAlive()) {
                    if (globalPathingThreadInitial.getProgress() == null) {
                        if (globalPathingThreadInitial.isDone) globalPathingThreadInitial = null;
                    } else {
                        initialCalcList = shortenList(
                                fromListNodeToGlobalPos(globalPathingThreadInitial.getProgress(), pathingRadius),
                                out.resolution, out.originX, out.originY, out.map, out.mapSizeX);
                        addPathToRenderer(initialCalcList);
                        globalPathingThreadInitial = null;
                        reRunInitial = false;
                        brokenPos = -2;
                    }
                }
            }
        }

        // We cant do anything without initialCalcList :<
        if (initialCalcList == null) return;

        // Check to see if we need to re-calc the initial path
        if (timeUntilCheckIfPathBroken > 50) {
            broken = nodeFromWhichToReCalc(initialCalcList, out);
            brokenPos = initialCalcList.indexOf(broken);
            if (broken != null) {
                reRunInitial = true;
            } else {
                brokenPos = -2;
            }

            broken = null;
            timeUntilCheckIfPathBroken = 0;
        } else {
            timeUntilCheckIfPathBroken++;
        }

        if (out.distanceFromGlobalToMap(out.functions.GetGlobalData(),
                endGoal) < 0.1) {
            isOnline = false;
            resetAllMotors();
            subsystemPathExecInterface.finishedPath(System.currentTimeMillis() - startPathExecTime);
            return;
        }


        if (curInitialCalcGoingTo >= brokenPos && brokenPos != -2) {
            curInitialCalcGoingTo = -1;
            initialCalcList = null;
            //reRunInitial = true;
            resetAllMotors();
            curInitialCalcGoingTo = -1;
            return;
        }

        if (curInitialCalcGoingTo < 0 || out.distanceFromGlobalToMap(out.functions.GetGlobalData(),
                initialCalcList.get(curInitialCalcGoingTo)) < 0.1) {
            curInitialCalcGoingTo++;

            if (curInitialCalcGoingTo >= initialCalcList.size()) {
                isOnline = false;
                resetAllMotors();
                subsystemPathExecInterface.finishedPath(System.currentTimeMillis() - startPathExecTime);
                curInitialCalcGoingTo = -1;
                return;
            }
        }

        if (startPathExecTime == 0) {
            startPathExecTime = System.currentTimeMillis();
        }

        double angleToTurn = getAngle(initialCalcList.get(curInitialCalcGoingTo), out.functions.GetGlobalData());
        if (Math.abs(angleToTurn) >= 15) {
            if (drivetrain.isGoing) drivetrain.forwardBackward(0);
            drivetrain.turn(angleToTurn < 0, 45);
        } else {
            drivetrain.forwardBackward(60);
        }
    }

    private void resetAllMotors() {
        drivetrain.forwardBackward(0);
        drivetrain.turn(false, 0);
    }

    private double getAngle(double[] mCurSQGoing, float[] curPos) {
        double dy = curPos[1] - mCurSQGoing[1]; // 7228-7450
        double dx = mCurSQGoing[0] - curPos[0]; // 5000-5313
        double angle = MathUtil.normaliseDeg(
                Math.atan2(dy, dx) / Math.PI * 180);
        double angleDeg = MathUtil.normaliseDeg(-Math.toDegrees(curPos[2]));

        return MathUtil.diffDeg(angleDeg, angle);
    }

    private double[] nodeFromWhichToReCalc(List<double[]> list, CartographerOut out) {
        int[] prev = null;
        for (double[] n : list) {
            if (prev == null) {
                prev = MathUtil.fromGlobalToMap(n, out.resolution, out.originX, out.originY);
                continue;
            }

            if (MathUtil.isObstructedBetweenPoints(prev, MathUtil.fromGlobalToMap(n, out.resolution, out.originX,
                    out.originY), out.map, out.mapSizeX, isObstructed)) {
                return MathUtil.fromMapToGlobal(prev, out.resolution, out.originX, out.originY);
            }
        }

        return null;
    }

    private List<double[]> fromListNodeToGlobalPos(List<Node> initial, double res) {
        List<double[]> newList = new ArrayList<>();

        for (Node n : initial) {
            double[] pos = MathUtil.fromMapToGlobal(new int[]{(int) (n.x * res), (int) (n.y * res)}, resolution,
                    originXBefore,
                    originYBefore);
            newList.add(pos);
        }
        Collections.reverse(newList);
        return newList;
    }

    private void addPathToRenderer(List<double[]> path) {
        renderer.clearAdditionalData();

        for (double[] i : path) {
            renderer.addAdditionalInfo(i);
        }

        renderer.addAdditionalInfo(endGoal);
        renderer.reDraw();
    }

    private List<double[]> shortenList(List<double[]> init, double res, double x, double y, byte[] map, int mapWidth) {
        List<double[]> returnList = new ArrayList<>();
        int[] prevPos = null;
        for (int i = 0; i < init.size(); i++) {
            int[] pos = MathUtil.fromGlobalToMap(init.get(i), res, x, y);
            if (prevPos == null) {
                prevPos = pos;
                returnList.add(MathUtil.fromMapToGlobal(pos, res, x, y));
                continue;
            }

            if (MathUtil.isObstructedBetweenPoints(prevPos, pos, map, mapWidth, this.isObstructed)) {
                returnList.add(init.get(i - 1));
                prevPos = MathUtil.fromGlobalToMap(init.get(i - 1), res, x, y);
            }
        }

        returnList.add(init.get(init.size() - 1));
        return returnList;
    }

    public interface SubsystemPathExecInterface {
        void finishedPath(long timeTookMS);
    }
}
