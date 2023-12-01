package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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


    private final DCMotor elevatorGearbox = DCMotor.getNeo550(2);
    private final DCMotor rotationalGearbox = DCMotor.getNeo550(1);

    private final ProfiledPIDController elevatorController =
      new ProfiledPIDController(
          ArmConstants.kElevatorKp,
          ArmConstants.kElevatorKi,
          ArmConstants.kElevatorKd,
          new TrapezoidProfile.Constraints(ArmConstants.maxVelElevator, ArmConstants.maxAccelElevator));

    private final PIDController rotationalController =
      new PIDController(
          ArmConstants.kRotKp,
          ArmConstants.kRotKi,
          ArmConstants.kRotKd);

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
    
    private final DCMotorSim rotationSim = 
        new DCMotorSim(
            rotationalGearbox, 
            ArmConstants.shoulderGearRatio,
            ArmConstants.rotationalInertiaKgMetersSquared);

    public ArmIOSimulationImpl() {

        // Initialize the simulation visualization
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

        // Sets the target pose
        setTargetPose(new ArmPose(0, 0));

        System.out.println("ArmIOSimulationImpl init");
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // Update inputs and calculate values
        inputs.extensionAppliedVolts = elevatorController.calculate(elevatorSim.getPositionMeters()) * ArmConstants.elevatorMaxVolts;
        inputs.extensionCurrentDrawAmps = elevatorSim.getCurrentDrawAmps();
        inputs.extensionVelocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
        inputs.extensionPositionMeters = elevatorSim.getPositionMeters();

        inputs.rotationalAppliedVolts = rotationalController.calculate(rotationSim.getAngularPositionRad()) * ArmConstants.rotMaxVolts;
        inputs.rotationalCurrentDrawAmps = rotationSim.getCurrentDrawAmps();
        inputs.rotationalAngularVelocityRadPerSec = rotationSim.getAngularVelocityRadPerSec();
        inputs.rotationalAngleRad = rotationSim.getAngularPositionRad();
        
        // Update the simulation
        elevatorSim.setInputVoltage(inputs.extensionAppliedVolts);
        rotationSim.setInputVoltage(inputs.rotationalAppliedVolts);
        
        elevatorSim.update(0.02);
        rotationSim.update(0.02);

        // Update the visualization
        m_arm.setAngle(inputs.rotationalAngleRad);
        m_arm.setLength(inputs.extensionPositionMeters);

        System.out.printf("Extension[Actual: %.3f, Target: %.3f] Rotation[Actual: %.3f, Target: %.3f]\n", 
            inputs.extensionPositionMeters, getTargetPose().getLength(),
            inputs.rotationalAngleRad, getTargetPose().getAngleRad());

    }

    @Override
    public void setTargetPose(ArmPose pose) {
        elevatorController.setGoal(pose.getLength());
        rotationalController.setSetpoint(pose.getAngleRad());
    }

    @Override
    public ArmPose getTargetPose() {
        return new ArmPose(rotationalController.getSetpoint(), elevatorController.getGoal().position);
    }

    @Override
    public ArmPose getCurrentPose() {
        return new ArmPose(rotationSim.getAngularPositionRad(), elevatorSim.getPositionMeters());
    }

    @Override
    public void stop() {
        // TODO: Stop the PID controllers
    }
}
