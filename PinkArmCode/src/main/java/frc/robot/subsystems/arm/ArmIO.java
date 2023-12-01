package frc.robot.subsystems.arm;

public interface ArmIO {

    public void updateInputs(ArmIOInputs inputs);

    public void setTargetPose(ArmPose pose);

    public ArmPose getTargetPose();

    public ArmPose getCurrentPose();
  
    // public default void setShoulderVoltage(double volts) {}

    // public default void setExtensionVoltage(double volts) {}

    // public default void setMotorOutput(double output) {}
  
    // public default void setBrakeMode(boolean shoulderBrake) {}

    public void stop();
}