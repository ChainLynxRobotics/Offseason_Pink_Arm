package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase  {

  ArmIOInputs inputs = new ArmIOInputs();

  private final ArmIO armIO;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(ArmIO armIO) {
    this.armIO = armIO;
  }

  @Override
  public void periodic() {
    armIO.updateInputs(inputs);
  }

  /**
   * Gets the target pose of the arm, this may not be the current pose, see {@link #getCurrentPose() getCurrentPose()} for that
   * 
   * NOTE: Modifications to the returned pose will not affect the arm
   * @return - The target pose of the arm
   */
  public ArmPose getTargetPose() {
    return armIO.getTargetPose();
  }

  /**
   * Gets the current pose of the arm, this may not be the target pose, see {@link #getTargetPose() getTargetPose()} for that
   * @return - The current pose of the arm
   */
  public ArmPose getCurrentPose() {
    return armIO.getCurrentPose();
  }

  /**
   * Sets the target pose of the arm
   * @param pose - The target pose of the arm
   */
  public void setTargetPose(ArmPose pose) {
    armIO.setTargetPose(pose);
  }

  public void setTargetPose(double angleRad, double length) {
    armIO.setTargetPose(new ArmPose(angleRad, length));
  }

  /**
   * Sets the target angle in radians, the target extension length will stay the same
   * @param angle
   */
  public void setTargetAngleRad(double angle) {
    setTargetPose(getTargetPose().setAngleRad(angle));
  }

  /**
   * Sets the target angle in degrees, the target extension length will stay the same
   * @param angle - The target angle in degrees
   */
  public void setTargetAngleDeg(double angle) {
    setTargetPose(getTargetPose().setAngleDeg(angle));
  }

  /**
   * Sets the target extension length in meters, the target angle will stay the same
   * @param length - The target extension length in meters
   */
  public void setTargetExtensionMeters(double length) {
    setTargetPose(getTargetPose().setLength(length));
  }
  
  /**
   * Returns if the arm is close enough to the target pose to be considered at the target pose
   * @return - If the arm is close enough
   */
  public boolean atTargetPose() {
    return getTargetPose().isWithinError(getCurrentPose());
  }

  /**
   * Returns if the arm is close enough to the target pose to be considered at the target pose
   * @param pose - The pose to compare to
   * @return - If the arm is close enough
   */
  public boolean atTargetPose(ArmPose pose) {
    return pose.isWithinError(getCurrentPose());
  }

  public void stop() {
    armIO.stop();
  }
}
