package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ArmSubsystem extends SubsystemBase {

  @AutoLogOutput
  private ArmPositions targetPose;
  ArmIOInputs inputs = new ArmIOInputs();
  ArmIO armIO;
  

  public ArmSubsystem(ArmIO armIO) {
    this.armIO = armIO; 
    armIO.initializeInputs();
  }

  @Override
  public void periodic() {
    armIO.updateInputs(inputs);
  }

  public void setAngleInput(double angleRad) {
    inputs.shoulderAngleRad = angleRad;
  }

  public void reachGoal(double extension, double angle) {
    armIO.setTargetExtensionLength(extension);
    armIO.setTargetShoulderAngle(angle);
  }


  public void setArmPosition(ArmPositions position) {
    this.targetPose = position;
  }


  public Pose2d calcCurrentPose(double armLengthMeters, double shoulderAngleRad) {
    double x = ArmConstants.PIVOT_X_OFFSET + armLengthMeters * Math.cos(shoulderAngleRad);
    double y = ArmConstants.PIVOT_Y_OFFSET + armLengthMeters * Math.sin(shoulderAngleRad);

    return new Pose2d(x, y, new Rotation2d(shoulderAngleRad));
  }

  public ArmPositions calcTargetPose(Pose2d target) {
    double x = target.getX() - ArmConstants.PIVOT_X_OFFSET;
    double y = target.getY() - ArmConstants.PIVOT_Y_OFFSET;
    double extensionDist = Math.hypot(x, y);

    return new ArmPositions(Math.asin(y / extensionDist), extensionDist);
  }

  public boolean atTarget() {
    if (Math.abs(targetPose.getExtensionLengthMeters() - inputs.extensionPositionMeters) < ArmConstants.armExtensionError &&
        Math.abs(targetPose.getShoulderAngleRad() - inputs.shoulderAngleRad) < ArmConstants.armAngleError) {
          return true;
    }
    return false;
  }

  public void stop() {
    armIO.stop();
  }

}
