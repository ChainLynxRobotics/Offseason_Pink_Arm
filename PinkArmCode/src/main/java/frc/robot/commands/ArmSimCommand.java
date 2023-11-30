package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmSimCommand extends CommandBase {
      private final ArmSubsystem m_arm;
      private int counter;

      public ArmSimCommand(ArmSubsystem arm) {
        this.m_arm = arm;

        addRequirements(m_arm);
      }

      @Override
      public void initialize() {
        counter = 0;
      }
    

      @Override
      public void execute() {
        double input = counter % 180;
        if (input > 90) {
            m_arm.setAngleInput((180 - input) * Math.PI/180);
        } else {
            m_arm.setAngleInput(input * Math.PI/180);
        }
        counter++;
      }


    
      @Override
      public boolean isFinished() {
        if (counter > 1800) {
            return true;
        }
        return false;
      }
}
    
