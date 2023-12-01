package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;

public class ArmIOSimulationImpl implements ArmIO {

    @AutoLogOutput
    private final Mechanism2d mech2d;
    @AutoLogOutput
    private final MechanismRoot2d m_root2d;
    @AutoLogOutput
    private final MechanismLigament2d m_arm;

    REVPhysicsSim REVsim = REVPhysicsSim.getInstance();



    private final DCMotor elevatorGearbox = DCMotor.getNeo550(2);
    private final CANSparkMax elevatorMotor1 = new CANSparkMax(ArmConstants.controller1port, MotorType.kBrushless);
    private final CANSparkMax elevatorMotor2 = new CANSparkMax(ArmConstants.controller2port, MotorType.kBrushless);

    private final DCMotor rotationalGearbox = DCMotor.getNeo550(1);
    private final CANSparkMax rotationalMotor = new CANSparkMax(ArmConstants.controller3port, MotorType.kBrushless);

    private final RelativeEncoder elevatorEncoder = elevatorMotor1.getEncoder();
    private final RelativeEncoder rotationalEncoder = rotationalMotor.getEncoder();

    private final ProfiledPIDController elevatorController =
      new ProfiledPIDController(
          ArmConstants.kElevatorKp,
          ArmConstants.kElevatorKi,
          ArmConstants.kElevatorKd,
          new TrapezoidProfile.Constraints(ArmConstants.maxVelElevator, ArmConstants.maxAccelElevator));
    private final PIDController rotationController = 
      new PIDController(
          ArmConstants.kRotKp, 
          ArmConstants.kRotKi, 
          ArmConstants.kRotKd);
    
    private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          ArmConstants.kElevatorkS,
          ArmConstants.kElevatorkG,
          ArmConstants.kElevatorkV,
          ArmConstants.kElevatorkA);

    private final ElevatorSim elevatorSim =
      new ElevatorSim(
          elevatorGearbox,
          ArmConstants.kElevatorGearing,
          ArmConstants.kCarriageMass,
          ArmConstants.kElevatorDrumRadius,
          ArmConstants.kMinElevatorHeightMeters,
          ArmConstants.kMaxElevatorHeightMeters,
          true,
          VecBuilder.fill(0.01));

    public ArmIOSimulationImpl() {
        elevatorMotor2.follow(elevatorMotor1); // Sync the 2 different motors used for extension

        mech2d = new Mechanism2d(ArmConstants.Simulation.simWidth, ArmConstants.Simulation.sinHeight);
        m_root2d = mech2d.getRoot("PinkArmRoot", ArmConstants.Simulation.rootX, ArmConstants.Simulation.rootY);
        m_arm = m_root2d.append(
        new MechanismLigament2d("Elevator", 
            ArmConstants.Simulation.armLength, 
            0.0,
            ArmConstants.Simulation.armWidth, 
            ArmConstants.Simulation.armColor
        ));

        SmartDashboard.putData("PinkArm", mech2d);
        REVsim.addSparkMax(elevatorMotor1, elevatorGearbox);
        REVsim.addSparkMax(elevatorMotor2, elevatorGearbox);
        REVsim.addSparkMax(rotationalMotor, rotationalGearbox);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {


        elevatorSim.setInput(elevatorMotor1.get() * RobotController.getBatteryVoltage());


        
    }

    @Override
    public void setTargetPose(ArmPose pose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTargetPose'");
    }

    @Override
    public ArmPose getTargetPose() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTargetPose'");
    }

    @Override
    public void stop() {
        elevatorController.setGoal(0.0);
        rotationController.setSetpoint(0.0);
        elevatorMotor1.set(0.0);
        rotationalMotor.set(0.0);
    }
}
