package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.ArmConstants;

public class ArmPose {

    private double length; // In meters
    private double angle; // In radians

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
     * @return this ArmPos object
     */
    public ArmPose setLength(double length) {
        this.length = length;
        return this;
    }

    /**
     * Sets the angle
     * @param angle - the angle of the arm in radians
     * @return this ArmPos object
     */
    public ArmPose setAngleRad(double angle) {
        this.angle = angle;
        return this;
    }

    /**
     * Sets the angle
     * @param angle - the angle of the arm in degrees
     * @return this ArmPos object
     */
    public ArmPose setAngleDeg(double angle) {
        this.angle = Math.toRadians(angle);
        return this;
    }

    /**
     * Checks if the arm is within the error of the target pose, error values are defined in {@link ArmConstants}
     * @param pose - the pose to compare
     * @return true if the arm is within the error of the compared pose
     */
    public boolean isWithinError(ArmPose pose) {
        return Math.abs(pose.getAngleRad() - getAngleRad()) < ArmConstants.armAngleError && Math.abs(pose.getLength() - getLength()) < ArmConstants.armExtensionError;
    }
}
