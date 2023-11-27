package frc.robot.subsystems.arm;

import java.awt.Color;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase implements AutoCloseable {

  @AutoLogOutput
  private final Mechanism2d mech2d;
  @AutoLogOutput
  private final MechanismRoot2d m_root2d;
  @AutoLogOutput
  private final MechanismLigament2d m_arm;

  private final DCMotor m_elevatorGearbox = DCMotor.getNeo550(2);
  private final CANSparkMax m_elevatorMotor1 = new CANSparkMax(ArmConstants.controller1Id, MotorType.kBrushless);
  private final CANSparkMax m_elevatorMotor2 = new CANSparkMax(ArmConstants.controller2Id, MotorType.kBrushless);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    m_elevatorMotor2.follow(m_elevatorMotor1);
    
    // Simulation
    mech2d = new Mechanism2d(ArmConstants.Simulation.simWidth, ArmConstants.Simulation.sinHeight);
    m_root2d = mech2d.getRoot("PinkArm", ArmConstants.Simulation.rootX, ArmConstants.Simulation.rootY);
    m_arm = m_root2d.append(
      new MechanismLigament2d("Elevator", 
        ArmConstants.Simulation.armLength, 
        0.0,
        ArmConstants.Simulation.armWidth, 
        new Color8Bit(0, 0, 255)
      ));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {

    } else {

    }
  }

  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    Pose2d currentPose = calcCurrentPose(ArmConstants.Simulation.armLength, 0.0);
    m_arm.setAngle(currentPose.getRotation());
    m_arm.setLength(currentPose.getTranslation().getX());
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

  @Override
  public void close() throws Exception {
    mech2d.close();
    // TODO: close other resources
  }
}
