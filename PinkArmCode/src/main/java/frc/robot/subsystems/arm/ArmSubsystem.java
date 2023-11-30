package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.AutoLogOutput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase implements AutoClosable {
  @AutoLogOutput
  private final Mechanism2d mechanism;
  @AutoLogOutput
  private final MechanismRoot2d root;
  @AutoLogOutput
  private final MechanismLigament2d ligament;
  private final DCMotor m_eleatorGearbox = DCMotor.getNeo550(numMotors 2);
  private final CANSparkMax m_elevatorMotor = new CANSparkMax(ArmConstants.controller1port, CANSparkMaxLowLevel.MotorType.kBrushless);
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    mechanism = new Mechanism2d(
    ArmConstants.SIM_HEIGHT,
    ArmConstants.SIM_WIDTH
    );
    root = new MechanismRoot2d(mechanism.getRoot("arm", ArmConstants.PIVOT_X_OFFSET, ArmConstants.PIVOT_Y_OFFSET));
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
  }

  private boolean checkValidState(double targetShoulderAngle, double targetExtensionLength) {
    throw new UnsupportedOperationException("unimplemented");
  }
  @Override
  public void close() throws Exception {
    mech2d.close();
  }
}
