package frc.robot.util;

import java.util.function.Predicate;

public class MathUtil {
    public static double normaliseDeg(double a) {
        return (a % 360 + 360) % 360;
    }

    public static double diffDeg(double a, double b) {
        double sign = 1;
        if (b < a) {
            double tmp = b;
            b = a;
            a = tmp;
            sign = -1;
        }

        return sign * (b - a < 180 ? b - a : (b - a) - 360);
    }

    public static double wrap360(double angle) {
        return angle < 0 ? -angle : 360 - angle;
    }

    public static double wrap360Rad(double radAngle) {
        double angle = Math.toDegrees(radAngle);
        return angle < 0 ? -angle : 360 - angle;
    }

    public static int[] getPointDistFromWithAngle(
            int[] curPoint,
            double curAngle,
            int dist) {
        double angleRadians = Math.toRadians(curAngle);
        double x = curPoint[0] + dist * Math.cos(angleRadians);
        double y = curPoint[1] + dist * Math.sin(angleRadians);
        return new int[]{(int) x, (int) y};
    }

    public static double calculateAngle(
            int[] pointA,
            int[] pointB,
            int[] pointC) {
        double[] vectorAB = {pointB[0] - pointA[0], pointB[1] - pointA[1]};
        double[] vectorBC = {pointC[0] - pointB[0], pointC[1] - pointB[1]};

        double dotProduct = vectorAB[0] * vectorBC[0] + vectorAB[1] * vectorBC[1];
        double magnitudeAB = Math.sqrt(
                vectorAB[0] * vectorAB[0] + vectorAB[1] * vectorAB[1]);
        double magnitudeBC = Math.sqrt(
                vectorBC[0] * vectorBC[0] + vectorBC[1] * vectorBC[1]);

        double cosTheta = dotProduct / (magnitudeAB * magnitudeBC);
        double angleRad = Math.acos(cosTheta);
        double crossProduct = vectorAB[0] * vectorBC[1] - vectorAB[1] * vectorBC[0];

        return Math.signum(crossProduct) * angleRad;
    }

    public static int convertTo1D(int boardWidth, int x, int y) {
        return (x * boardWidth + y);
    }

    public static int fromGlobalXToMapX(double globalX, double resolution, double originX) {
        return (int) ((globalX - originX) / resolution);
    }

    public static double fromMapXToGlobalX(int mapX, double resolution, double originX) {
        return (mapX * resolution + originX);
    }

    public static int fromGlobalYToMapY(double globalY, double resolution, double originY) {
        return (int) ((globalY - originY) / resolution);
    }

    public static double fromMapYToGlobalY(int mapY, double resolution, double originY) {
        return (mapY * resolution + originY);
    }

    public static double[] fromMapToGlobal(int[] mapPos, double resolution, double originX, double originY) {
        return new double[]{fromMapXToGlobalX(mapPos[0], resolution, originX), fromMapYToGlobalY(mapPos[1],
                resolution, originY)};
    }

    public static int[] fromGlobalToMap(double[] mapPos, double resolution, double originX, double originY) {
        return new int[]{fromGlobalXToMapX(mapPos[0], resolution, originX), fromGlobalYToMapY(mapPos[1], resolution,
                originY)};
    }

    public static int[] fromGlobalToMap(float[] mapPos, double resolution, double originX, double originY) {
        return new int[]{fromGlobalXToMapX(mapPos[0], resolution, originX), fromGlobalYToMapY(mapPos[1], resolution,
                originY)};
    }

    public static boolean isObstructedBetweenPoints(int[] point1, int[] point2, byte[] values, int width,
                                                    Predicate<Byte> predicate) {
        double x1 = point1[0];
        double y1 = point1[1];
        double x2 = point2[0];
        double y2 = point2[1];

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

            if (x1 != point2[0] || y1 != point2[1]) {
                int lineIndex = (int) (y1 * width + x1);
                if (predicate.test(values[lineIndex])) {
                    return true;
                }
            }
        }

        return false;
    }
}
