package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ArmIOInputs {
    public double rotationalAngleRad;
    public double rotationalAppliedVolts;
    public double rotationalCurrentDrawAmps;
    public double rotationalAngularVelocityRadPerSec;

    public double rotationalMotorOneTemp;
    public double rotationalMotorTwoTemp;

    public boolean isFastShoulderAcceleration;
    public boolean isHighExtensionCurrentLimit;

    public double extensionAppliedVolts;
    public double extensionCurrentDrawAmps;
    public double extensionVelocityMetersPerSec;
    public double extensionPositionMeters;

    public double extensionMotorOneTemp;
    public double extensionMotorTwoTemp;
}
