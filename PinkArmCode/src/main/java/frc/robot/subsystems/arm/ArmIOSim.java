package frc.robot.subsystems.arm;


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

/** Draws and updates arm on WPILib simulation GUI and logs arm info to AdvantageScope */
//2 ways to update simulator: ElevatorSim and matrix math
public class ArmIOSim implements ArmIO {
    private final DCMotor m_gearbox = DCMotor.getNeo550(2);
    private final DCMotor m_rotMotor = DCMotor.getNeo550(1);
    private final ProfiledPIDController extensionController =
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
    private final ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          ArmConstants.kElevatorkS,
          ArmConstants.kElevatorkG,
          ArmConstants.kElevatorkV,
          ArmConstants.kElevatorkA);
    private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_gearbox,
          ArmConstants.kElevatorGearing,
          ArmConstants.kCarriageMass,
          ArmConstants.kElevatorDrumRadius,
          ArmConstants.kMinElevatorHeightMeters,
          ArmConstants.kMaxElevatorHeightMeters,
          true,
          VecBuilder.fill(0.01));
    @AutoLogOutput
    private double elevatorPos;

    private final CANSparkMax m_motorController1 = new CANSparkMax(ArmConstants.controller1port, MotorType.kBrushless);
    private final CANSparkMax m_motorController2 = new CANSparkMax(ArmConstants.controller2port, MotorType.kBrushless);
    private final CANSparkMax m_rotMotorController = new CANSparkMax(ArmConstants.controller3port, MotorType.kBrushless);

    private final RelativeEncoder m_Encoder1 = m_motorController1.getEncoder();
    private final RelativeEncoder m_rotEncoder =  m_rotMotorController.getEncoder();
    private final REVPhysicsSim r = new REVPhysicsSim(); //REV is incompatible with default hardware sim, so update encoders from here

    private final Mechanism2d curMech = new Mechanism2d(5, 5);
    private final MechanismRoot2d curRoot =
                curMech.getRoot("arm", ArmConstants.PIVOT_X_OFFSET, ArmConstants.PIVOT_Y_OFFSET);
    @AutoLogOutput
    private final MechanismLigament2d curShoulder = curRoot.append(
        new MechanismLigament2d("targetShoulder", 1, 60, 10, new Color8Bit(Color.kRed)));

    public ArmIOSim() {
        SmartDashboard.putData("mech screen", curMech);
        r.addSparkMax(m_motorController1, m_gearbox);
        r.addSparkMax(m_motorController2, m_gearbox);
        r.addSparkMax(m_rotMotorController, m_rotMotor);

        m_motorController2.follow(m_motorController1);
    }


    @Override
    public void updateInputs(ArmIOInputs inputs) {
        m_Encoder1.setPosition(m_elevatorSim.getPositionMeters() * ArmConstants.sparkMaxEncoderRotPerMeter); //could also set position conversion factor
        m_elevatorSim.setInput(m_motorController1.get() * RobotController.getBatteryVoltage());
        elevatorPos = m_elevatorSim.getPositionMeters();
        curShoulder.setLength(ArmConstants.minExtensionLength + inputs.extensionPositionMeters); //use elevatorPos during teleop
        curShoulder.setAngle((inputs.shoulderAngleRad*180/Math.PI)%360);
        
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));


        Matrix<N4, N1> systemStates = VecBuilder.fill(
            inputs.shoulderAngleRad,
            inputs.shoulderAngularVelocityRadPerSec,
            inputs.extensionPositionMeters,
            inputs.extensionVelocityMetersPerSec
        );

        Matrix<N2, N1> systemInputs = VecBuilder.fill(
            rotationController.calculate(inputs.shoulderAngleRad),
            extensionController.calculate(inputs.extensionPositionMeters)
        );

        //https://lpsa.swarthmore.edu/NumInt/NumIntFourth.html
        Matrix<N4, N1> nextState = NumericalIntegration.rk4(this::calculateSystem, systemStates, systemInputs, 0.02);
    
        //use integrator to set the values for the arm at the next timestep
        //can also use this to update UI position and angle
        inputs.shoulderAngleRad = nextState.get(0, 0);
        inputs.shoulderAngularVelocityRadPerSec = nextState.get(1, 0);
        inputs.extensionPositionMeters = nextState.get(2, 0);
        inputs.extensionVelocityMetersPerSec = nextState.get(3, 0);

        //get control effort from PID controllers
        inputs.shoulderAppliedVolts = systemInputs.get(0, 0);
        inputs.extensionAppliedVolts = systemInputs.get(1, 0);

        inputs.shoulderCurrentDrawAmps = Math.min(
           Math.abs(m_gearbox.getCurrent(inputs.shoulderAngularVelocityRadPerSec, inputs.shoulderAppliedVolts)),
           ArmConstants.sparkMaxCurrentLimit);
        inputs.extensionCurrentDrawAmps = Math.min(
           Math.abs(m_gearbox.getCurrent(inputs.extensionVelocityMetersPerSec, inputs.extensionAppliedVolts)),
           ArmConstants.sparkMaxCurrentLimit);
    }


    @Override
    public void setMotorOutput(double output) {
        m_motorController1.set(output);
    }

    @Override
    public void setTargetPose(ArmIOInputs inputs, ArmPositions pose) {
        inputs.extensionPositionMeters = pose.getExtensionLengthMeters();
        inputs.shoulderAngleRad = pose.getShoulderAngleRad();
    }

    @Override
    public void setTargetShoulderAngle(double angleRad) {
        rotationController.setSetpoint(angleRad / ArmConstants.rotConversionFactor);
        double pidOutput = rotationController.calculate(m_rotEncoder.getPosition());
        m_rotMotorController.setVoltage(pidOutput);
    }

    @Override
    public void setTargetExtensionLength(double extension) {
        extensionController.setGoal(extension);

        //calculate pid control effort and set to motor controllers
        double pidOutput = extensionController.calculate(m_Encoder1.getPosition());
        double feedforwardOutput = m_feedforward.calculate(extensionController.getSetpoint().velocity);
        m_motorController1.setVoltage(pidOutput + feedforwardOutput);
    }
    
    @Override
    public void stop() {
        extensionController.setGoal(0.0);
        rotationController.setSetpoint(0.0);
        m_motorController1.set(0.0);
    }


    //moment of inertia changes based on extension (mass distribution changes), so we need to interpolate the actual value
    private double calculateShoulderMomentOfInertia(double angle, double extension) {
        //normalize current extension as a fraction of max extension
        double normExtension = inverseInterpolate(
            ArmConstants.minExtensionLength, ArmConstants.maxExtensionLength, extension);
        //estimate moment of inertia resulting in normExtension
        double mInertia = MathUtil.interpolate(
            ArmConstants.minMomentOfInertia, ArmConstants.maxMomentofInertia, normExtension);
        return mInertia;
    }

    private static double inverseInterpolate(double startValue, double endValue, double x) {
        return (x - startValue) / (endValue - startValue);
    }

     private Matrix<N4, N1> calculateSystem(Matrix<N4, N1> states, Matrix<N2, N1> inputs) {
        double shoulderAngle = states.get(0, 0);
        double shoulderAngularVelocity = states.get(1, 0);

        double extensionLength = states.get(2, 0);
        double extensionVelocity = states.get(3, 0);

        double shoulderVoltage = inputs.get(0, 0);
        double extensionVoltage = inputs.get(1, 0);

        double shoulderCurrent = m_rotMotor.getCurrent(shoulderAngularVelocity, shoulderVoltage);
        shoulderCurrent = MathUtil.clamp(shoulderCurrent, -ArmConstants.sparkMaxCurrentLimit, ArmConstants.sparkMaxCurrentLimit);
        double shoulderTorque = m_rotMotor.getTorque(shoulderCurrent);
        double shoulderAngularAcceleration =
                shoulderTorque / calculateShoulderMomentOfInertia(shoulderAngle, extensionLength);

        double extensionCurrent = m_gearbox.getCurrent(extensionVelocity, extensionVoltage);
        extensionCurrent =
                MathUtil.clamp(extensionCurrent, -ArmConstants.sparkMaxCurrentLimit, ArmConstants.sparkMaxCurrentLimit);
        double extensionTorque = m_gearbox.getTorque(extensionCurrent);
        double extensionForce = extensionTorque / ArmConstants.kElevatorDrumRadius;
        double EXTENSION_MASS_KG = ArmConstants.kTotalMass - ArmConstants.kCarriageMass;
        double extensionAcceleration = extensionForce / EXTENSION_MASS_KG;

        return VecBuilder.fill(
                shoulderAngularVelocity,
                shoulderAngularAcceleration,
                extensionVelocity,
                extensionAcceleration);
     }

}
