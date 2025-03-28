package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.GroundIntake;

public class GroundIntakeCommands 
{
    public static Command intake(GroundIntake groundIntake) 
    {
        return Commands.runOnce(
            () -> {
                groundIntake.moveIntake(0.25);
            },
            groundIntake);
    }

    public static Command stop(GroundIntake groundIntake) 
    {
        return Commands.runOnce(
            () -> {
                groundIntake.moveIntake(0);
            },
            groundIntake);
    }

    public static Command setStage(GroundIntake groundIntake, int stage) 
    {
        return Commands.runOnce(
            () -> {
                switch(stage){
                    case 0:
                        groundIntake.setWristAngle(0);
                        break;
                    case 1:
                        groundIntake.setWristAngle(100);
                        break;
                }
            },
            groundIntake);
    }
    
}
