package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmPositions;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmPIDSimCommand extends CommandBase {
      private final ArmSubsystem m_arm;
      private final double angle;
      private final double extension;

      public ArmPIDSimCommand(ArmSubsystem arm, ArmPositions targetPose) {
        this.m_arm = arm;
        this.angle = targetPose.getShoulderAngleRad();
        this.extension = targetPose.getExtensionLengthMeters();

        addRequirements(m_arm);
      }


      @Override
      public void execute() {
        m_arm.reachGoal(extension, angle);
        double curAngle = m_arm.getInputs().shoulderAngleRad;
        double curExtension = m_arm.getInputs().extensionPositionMeters;

        System.out.println("current: " + toPose(curAngle, curExtension));
        System.out.println("target: " + toPose(angle, extension) + "\n");
      }

      public Pose2d toPose(double angle, double extension) {
        return new Pose2d(extension*Math.cos(angle), extension*Math.sin(angle), new Rotation2d(angle));
      }

      @Override
      public void end(boolean interrupted) {
        m_arm.stop();
      }

    
      @Override
      public boolean isFinished() {
        if (m_arm.atTarget(new Pose2d(extension*Math.cos(angle), extension*Math.sin(angle), new Rotation2d(angle)))) {
            System.out.println("FINISHED!!!!");
            return true;
        }
        return false;
      }
}
    
