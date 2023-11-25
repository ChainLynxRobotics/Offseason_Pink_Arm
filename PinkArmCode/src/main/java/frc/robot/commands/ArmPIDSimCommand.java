package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmPositions;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmPIDSimCommand extends CommandBase {
      private final ArmSubsystem m_arm;
      private final ArmPositions targetPose;

      public ArmPIDSimCommand(ArmSubsystem arm, ArmPositions targetPose) {
        this.m_arm = arm;
        this.targetPose = targetPose;

        addRequirements(m_arm);
      }
    
      @Override
      public void initialize() {
        m_arm.setArmPosition(targetPose);
      }

      @Override
      public void execute() {
        System.out.println("NOT FINISHED!!!!");
        m_arm.reachGoal(targetPose.getExtensionLengthMeters(), targetPose.getShoulderAngleRad());
      }

      @Override
      public void end(boolean interrupted) {
        m_arm.stop();
      }

    
      @Override
      public boolean isFinished() {
        if (m_arm.atTarget()) {
            System.out.println("FINISHED!!!!");
            return true;
        }
        return false;
      }
}
    
