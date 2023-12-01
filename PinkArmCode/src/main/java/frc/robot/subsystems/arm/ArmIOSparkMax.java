package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;

public class ArmIOSparkMax implements ArmIO {
    private final CANSparkMax extension1;
    private final CANSparkMax extension2;
    private final CANSparkMax rotation;
    private final SparkMaxPIDController posController;
    private final SparkMaxPIDController rotController;

    public ArmIOSparkMax() {
        extension1 = new CANSparkMax(ArmConstants.controller1port, MotorType.kBrushless);
        extension2 = new CANSparkMax(ArmConstants.controller2port, MotorType.kBrushless);
        rotation = new CANSparkMax(ArmConstants.controller3port, MotorType.kBrushless); 

        posController = extension1.getPIDController();
        rotController = rotation.getPIDController();

        extension1.getEncoder().setPositionConversionFactor(ArmConstants.sparkMaxEncoderRotPerMeter);
        rotation.getEncoder().setPositionConversionFactor(ArmConstants.rotConversionFactor);
        rotation.getEncoder().setVelocityConversionFactor(Math.PI/30);

        extension2.follow(extension1);
    }


    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.shoulderAngleRad = rotation.getEncoder().getPosition();
        inputs.shoulderAngularVelocityRadPerSec = rotation.getEncoder().getVelocity();

        inputs.extensionPositionMeters = extension1.getEncoder().getPosition();
        inputs.extensionVelocityMetersPerSec = extension1.getEncoder().getVelocity();

        inputs.shoulderMotorTemp = rotation.getMotorTemperature();
        inputs.extensionMotorOneTemp = extension1.getMotorTemperature();
        inputs.extensionMotorTwoTemp = extension2.getMotorTemperature();


        //set other inputs
    }

    @Override
    public void setTargetShoulderAngle(double angleRad) {
        rotController.setReference(angleRad, ControlType.kPosition);
    }

    @Override
    public void setTargetExtensionLength(double length) {
        posController.setReference(length, ControlType.kPosition);
    }
  
    @Override
    public void setShoulderVoltage(double volts) {
        rotation.setVoltage(volts);
    }

    @Override
    public void setExtensionVoltage(double volts) {
        extension1.setVoltage(volts);
    }
  
    @Override
    public void setBrakeMode(boolean shoulderBrake) {
        if (shoulderBrake) {
            extension1.setSmartCurrentLimit(ArmConstants.sparkMaxCurrentLimit-50);
        }
    }


    @Override
    public void stop() {
        extension1.set(0);
        rotation.set(0);
    }
    
}
