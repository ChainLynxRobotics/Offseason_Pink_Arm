package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    /** update the values of all ArmIO inputs*/
    public default void initializeInputs() {}

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setTargetShoulderAngle(double angleRad) {}

    public default void setTargetExtensionLength(double length) {}
  
    public default void setShoulderVoltage(double volts) {}

    public default void setExtensionVoltage(double volts) {}
  
    public default void setBrakeMode(boolean shoulderBrake) {}

    public default void stop() {}

    @AutoLog
    class ArmIOInputs {
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
}