package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;

public class ArmPose {

    private static double length; // In meters
    private static double angle; // In radians

    public ArmPose(double angle, double length) {
        this.length = length;
        this.angle = angle;
    }

    /**
     * Gets an ArmPos object from a Pose2d object
     * @param pose - the Pose2d object
     * @return new ArmPos object
     */
    public static ArmPose fromPose2d(Pose2d pose) {
        return new ArmPose(pose.getRotation().getRadians(), pose.getTranslation().getX());
    }

    /**
     * The same as `new ArmPos(angle, length)`
     * @param length - the length of the arm in meters
     * @param angle - the angle of the arm in radians
     * @return new ArmPos object
     */
    public static ArmPose getPos(double angle, double length) {
        return new ArmPose(angle, length);
    }

    /**
     * @param length - the length of the arm in meters
     * @param angle - the angle of the arm in degrees
     * @return new ArmPos object
     */
    public static ArmPose getPosDeg(double angle, double length) {
        return new ArmPose(Math.toRadians(angle), length);
    }

    /**
     * Gets the length
     * @return the length of the arm in meters
     */
    public double getLength() {
        return length;
    }

    /**
     * Gets the angle
     * @return the angle of the arm in radians
     */
    public double getAngleRad() {
        return angle;
    }

    /**
     * Gets the angle
     * @return the angle of the arm in degrees
     */
    public double getAngleDeg() {
        return Math.toDegrees(angle);
    }

    /**
     * Sets the length
     * @param length - the length of the arm in meters
     */
    public void setLength(double length) {
        this.length = length;
    }

    /**
     * Sets the angle
     * @param angle - the angle of the arm in radians
     */
    public void setAngleRad(double angle) {
        this.angle = angle;
    }

    /**
     * Sets the angle
     * @param angle - the angle of the arm in degrees
     */
    public void setAngleDeg(double angle) {
        this.angle = Math.toRadians(angle);
    }
}
