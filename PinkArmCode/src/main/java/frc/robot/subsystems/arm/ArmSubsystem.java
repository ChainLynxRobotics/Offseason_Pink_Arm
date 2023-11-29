package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase implements AutoClosable {
  @AutoLogOutput
  private final Mechanism2d mechanism;
  @AutoLogOutput
  private final MechanismRoot2d root;
  @AutoLogOutput
  private final MechanismLigament2d ligament;
  private final DCMotor m_eleatorGearbox = DCMotor.getNeo550(numMotors: 2);
  private final CANSparkMax m_elevatorMotor = new CANSparkMax(Robot.Constants.kElevatorMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    mechanism = new Mechanism2d(
    Constants.SIM_HEIGHT,
    Constants.SIM_WIDTH
    );
    root = new MechanismRoot2d(mechanism.getRoot("arm", PIVOT_X_OFFSET, PIVOT_Y_OFFSET));
    ligament = new MechanismLigament2d("ligament", 3, 0, 5, new Color8Bit(Color.kWhite));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {

    } else {

    }
  }
  public void periodicSim() {
    
  }

  public void setTarget(double length) {
    
  }
  public void setTargetPose(Pose3d targetPose, Pose2d currentPose) {
  }

  private Pose2d calcCurrentPose(double armLengthMeters, double shoulderAngleRad) {
    throw new UnsupportedOperationException("unimplemented");
  }0

  private boolean checkValidState(double targetShoulderAngle, double targetExtensionLength) {
    throw new UnsupportedOperationException("unimplemented");
  }
  @Override
  public void close() throws Exception {
    mech2d.close();
  }
}
