package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ArmSubsystem extends SubsystemBase {

  ArmIOInputs inputs = new ArmIOInputs();
  ArmIO armIO;
  

  public ArmSubsystem(ArmIO armIO) {
    this.armIO = armIO; 
  }

  @Override
  public void periodic() {
    armIO.updateInputs(inputs);
  }

  public void setExtensionOutput(Joystick stick, int axis) {
    armIO.setMotorOutput(stick.getRawAxis(axis));
    System.out.println("axis value: " + stick.getRawAxis(axis));
  }

  public void reachGoal(double extension, double angle) {
    armIO.setTargetExtensionLength(extension);
    armIO.setTargetShoulderAngle(angle);
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

  public boolean atTarget(Pose2d target) {
    if (Math.abs(calcTargetPose(target).getExtensionLengthMeters() - inputs.extensionPositionMeters) < ArmConstants.armExtensionError &&
        Math.abs(calcTargetPose(target).getShoulderAngleRad() - inputs.shoulderAngleRad) < ArmConstants.armAngleError) {
          return true;
    }
    return false;
  }

  public ArmIOInputs getInputs() {
    return inputs;
  }

  public void stop() {
    armIO.stop();
  }
}
