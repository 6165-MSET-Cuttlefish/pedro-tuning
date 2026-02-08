package org.firstinspires.ftc.teamcode.modules;

import java.util.*;

public class ShooterInterpolation {

    public static class ShooterDataPoint {
        public final double x;
        public final double y;
        public final double rpm;
        public final double hood;
        public final double distance;
        public final int distanceIndex;

        public ShooterDataPoint(double x, double y, double rpm, double hood, int distanceIndex, double robotHeadingRad) {
            this.x = x;
            this.y = y;
            this.rpm = rpm;
            this.hood = hood;
            this.distanceIndex = distanceIndex;
            
            // Calculate turret position from robot position and heading
            double turretX = Turret.turretX;  // Turret offset X
            double turretY = Turret.turretY;    // Turret offset Y
            double turretFieldX = x + turretX * Math.cos(robotHeadingRad) - turretY * Math.sin(robotHeadingRad);
            double turretFieldY = y + turretX * Math.sin(robotHeadingRad) + turretY * Math.cos(robotHeadingRad);
            
            // Distance from turret position to target
            this.distance = Math.hypot(
                    144 - x,
                    144 - y
            );
        }
    }

    private static final List<ShooterDataPoint> DATA_POINTS = new ArrayList<>();
    private static final Map<Integer, Double> INDEX_TO_DISTANCE = new HashMap<>();

    private static final double ABOVE_MEAN_RPM = 70;
    public static double lastTargetDistance = 0;
    public static double lastBaseRPM = 0;
    public static double lastCompensatedRPM = 0;
    public static double lastSelectedHood = 0;
    public static int lastClosestDistanceIndex = -1;
    public static int lastPointsCount = 0;

    static {
        DATA_POINTS.add(new ShooterDataPoint(72, 72, 2800, 0.3, 0, 90));
        DATA_POINTS.add(new ShooterDataPoint(72, 72, 2600, 0.29, 0, 90));
        DATA_POINTS.add(new ShooterDataPoint(72, 72, 2500, 0.39, 0, 90));

        DATA_POINTS.add(new ShooterDataPoint(72, 14, 3325, 0.3, 1, 90));
        DATA_POINTS.add(new ShooterDataPoint(72, 14, 3100, 0.24, 1, 90));

        DATA_POINTS.add(new ShooterDataPoint(91, 84, 2600, 0.29, 2, 90));
        DATA_POINTS.add(new ShooterDataPoint(91, 84, 2400, 0.23, 2, 90));
        DATA_POINTS.add(new ShooterDataPoint(91, 84, 2300, 0.19, 2, 90));

//        DATA_POINTS.add(new ShooterDataPoint(55, 12, 3350, 0.3, 3, 90));
//        DATA_POINTS.add(new ShooterDataPoint(55, 12, 3200, 0.26, 3, 90));
//        DATA_POINTS.add(new ShooterDataPoint(55, 12, 3100, 0.28, 3, 90));

        for (ShooterDataPoint p : DATA_POINTS) {
            INDEX_TO_DISTANCE.putIfAbsent(p.distanceIndex, p.distance);
        }
    }

    public static double getTargetRPM(double targetDistance) {
        int[] idx = getOrderedDistanceIndices(targetDistance);

        List<ShooterDataPoint> near = getPointsByIndex(idx[0]);
        List<ShooterDataPoint> far = getPointsByIndex(idx[1]);

        double rpmNear = meanRPM(near);
        double rpmFar = meanRPM(far);

        double dNear = INDEX_TO_DISTANCE.get(idx[0]);
        double dFar = INDEX_TO_DISTANCE.get(idx[1]);

        double t = clamp01((targetDistance - dNear) / (dFar - dNear));
        double baseRPM = lerp(rpmNear, rpmFar, t);

        double compensated = baseRPM + ABOVE_MEAN_RPM;

        lastTargetDistance = targetDistance;
        lastBaseRPM = baseRPM;
        lastCompensatedRPM = compensated;
        lastClosestDistanceIndex = idx[0];
        lastPointsCount = near.size() + far.size();

        return compensated;
    }

    public static double getHoodPosition(double targetDistance, double currentRPM) {
        int[] idx = getOrderedDistanceIndices(targetDistance);

        double hoodNear = interpolateGroupHood(idx[0], currentRPM);
        double hoodFar = interpolateGroupHood(idx[1], currentRPM);

        double dNear = INDEX_TO_DISTANCE.get(idx[0]);
        double dFar = INDEX_TO_DISTANCE.get(idx[1]);

        double t = clamp01((targetDistance - dNear) / (dFar - dNear));
        double hood = lerp(hoodNear, hoodFar, t);

        lastSelectedHood = hood;
        return hood;
    }

    private static double interpolateGroupHood(int index, double rpm) {
        List<ShooterDataPoint> group = getPointsByIndex(index);
        group.sort(Comparator.comparingDouble(p -> p.rpm));

        ShooterDataPoint low = group.get(0);
        ShooterDataPoint high = group.get(group.size() - 1);

        for (int i = 0; i < group.size() - 1; i++) {
            ShooterDataPoint a = group.get(i);
            ShooterDataPoint b = group.get(i + 1);
            if (rpm >= a.rpm && rpm <= b.rpm) {
                low = a;
                high = b;
                break;
            }
        }

        double t = clamp01((rpm - low.rpm) / (high.rpm - low.rpm));
        return lerp(low.hood, high.hood, t);
    }

    private static int[] getOrderedDistanceIndices(double target) {
        List<Map.Entry<Integer, Double>> list = new ArrayList<>(INDEX_TO_DISTANCE.entrySet());
        list.sort(Comparator.comparingDouble(e -> Math.abs(e.getValue() - target)));

        int a = list.get(0).getKey();
        int b = list.get(1).getKey();

        double da = INDEX_TO_DISTANCE.get(a);
        double db = INDEX_TO_DISTANCE.get(b);

        return da <= db ? new int[]{a, b} : new int[]{b, a};
    }

    private static List<ShooterDataPoint> getPointsByIndex(int index) {
        List<ShooterDataPoint> out = new ArrayList<>();
        for (ShooterDataPoint p : DATA_POINTS) {
            if (p.distanceIndex == index) out.add(p);
        }
        return out;
    }

    private static double meanRPM(List<ShooterDataPoint> pts) {
        double sum = 0;
        for (ShooterDataPoint p : pts) sum += p.rpm;
        return sum / pts.size();
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}