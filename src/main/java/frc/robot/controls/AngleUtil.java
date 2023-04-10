package frc.robot.controls;

public class AngleUtil {
    public static final double normalizeAngle(double angle) {
        double normalizedAngle = angle % 360.0; // get the angle within the range of 0 to 359.999...
        if (normalizedAngle > 180.0) {
            normalizedAngle -= 360.0; // subtract 360 degrees to get a negative angle
        } else if (normalizedAngle < -180.0) {
            normalizedAngle += 360.0; // add 360 degrees to get a positive angle
        }
        return normalizedAngle;
    }

    public static final double nearestCardinalAngle(double angle) {
        double nearestAngle = Math.round(angle / 90.0) * 90.0;
        if (nearestAngle == 360.0) {
            nearestAngle = 0.0;
        }
        return nearestAngle;
    }

}
