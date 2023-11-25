package frc.robot.subsystems.arm;


public class ArmPositions {
    private double shoulderAngleRad = 0;
    private double extensionLengthMeters = 0;

    public ArmPositions(double shoulderAngleRad, double extensionLengthMeters) {
        this.shoulderAngleRad = shoulderAngleRad;
        this.extensionLengthMeters = extensionLengthMeters;
    }

    public double getShoulderAngleRad() {
        return shoulderAngleRad;
    }

    public double getExtensionLengthMeters() {
        return extensionLengthMeters;
    }
}