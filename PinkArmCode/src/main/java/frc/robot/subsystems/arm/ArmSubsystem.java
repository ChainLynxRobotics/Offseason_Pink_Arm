package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {

    } else {

    }
  }

  public void setTargetPose(Pose3d targetPose, Pose2d currentPose) {
  }

  private Pose2d calcCurrentPose(double armLengthMeters, double shoulderAngleRad) {
    throw new UnsupportedOperationException("unimplemented");
  }

  private boolean checkValidState(double targetShoulderAngle, double targetExtensionLength) {
    throw new UnsupportedOperationException("unimplemented");
  }
}
