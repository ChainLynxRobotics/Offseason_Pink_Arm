package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ArmIOInputs {
    public double shoulderAngleRad;
    public double shoulderAppliedVolts;
    public double shoulderCurrentDrawAmps;
    public double shoulderAngularVelocityRadPerSec;

    public double shoulderMotorOneTemp;
    public double shoulderMotorTwoTemp;

    public boolean isFastShoulderAcceleration;
    public boolean isHighExtensionCurrentLimit;

    public double extensionAppliedVolts;
    public double extensionCurrentDrawAmps;
    public double extensionVelocityMetersPerSec;
    public double extensionPositionMeters;

    public double extensionMotorOneTemp;
    public double extensionMotorTwoTemp;
}
