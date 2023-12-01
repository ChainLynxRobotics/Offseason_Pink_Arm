package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;

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

  public void setTarget(double length) {
    throw new UnsupportedOperationException("unimplemented");
  }

  // Actual implementation of the arm subsystem
  public void setTargetPose(Pose3d targetPose, Pose2d currentPose) {
  }

  private Pose2d calcCurrentPose(double armLengthMeters, double shoulderAngleRad) {
    throw new UnsupportedOperationException("unimplemented");
  }

  private boolean checkValidState(double targetShoulderAngle, double targetExtensionLength) {
    throw new UnsupportedOperationException("unimplemented");
  }

  public void stop() {
    throw new UnsupportedOperationException("unimplemented");
  }
}
