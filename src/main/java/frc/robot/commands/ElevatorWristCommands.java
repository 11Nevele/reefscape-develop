// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SuperStructureState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Wrist;

// use this during driving
public class ElevatorWristCommands {
  public static Command setElevatorStage(Elevator elevator, int level) {

    return Commands.runOnce(
        () -> {
          double elevatorHeight = 0;
          switch (level) {
            case 0 -> {
              elevatorHeight = SuperStructureState.L0_HEIGHT;
            }

            case 1 -> {
              elevatorHeight = SuperStructureState.L1_HEIGHT;
            }
            case 2 -> {
              elevatorHeight = SuperStructureState.L2_HEIGHT;
            }
            case 3 -> {
              elevatorHeight = SuperStructureState.L3_HEIGHT;
            }
            case 4 -> {
              elevatorHeight = SuperStructureState.L4_HEIGHT;
            }
            case 5 -> {
              elevatorHeight = SuperStructureState.HUMAN_HEIGHT;
            }
            case 6 -> {
              elevatorHeight = SuperStructureState.L1B_HEIGHT;
            }
            case 7 -> {
              elevatorHeight = SuperStructureState.L2B_HEIGHT;
            }
            case 8 -> {
              elevatorHeight = SuperStructureState.HUMAN_HEIGHT;
            }
          }
          elevator.setElevatorHeight(elevatorHeight);
        },
        elevator);
  }

  public static Command moveElevator(Elevator elevator, double spd) {
    return Commands.runOnce(
        () -> {
          elevator.manualMove(spd * 0.25);
        },
        elevator);
  }

  public static Command stopElevator(Elevator elevator) {
    return Commands.runOnce(
        () -> {
          elevator.manualMove(0);
          elevator.setElevatorHeight(elevator.getElevatorHeight() + 1);
        },
        elevator);
  }

  public static Command moveWrist(Wrist wrist, double spd) {
    return Commands.run(
        () -> {
          wrist.moveWrist(spd * 0.25);
        },
        wrist);
  }

  static boolean groundPos = true;
  static int curLevel = 10;

  public static Command setWristLevel(Wrist wrist, int level) {
    return Commands.runOnce(
        () -> {
          if (curLevel != 0 && level == 0) groundPos = true;
          curLevel = level;
          switch (level) {
            case 0:
              if (groundPos) wrist.setWristAngle(145); // Ground Intake
              else wrist.setWristAngle(170);
              groundPos = !groundPos;
              break;
            case 1:
              wrist.setWristAngle(155); // L2 //170
              break;
            case 2:
              wrist.setWristAngle(155); // L3 //170
              break;
            case 3:
              wrist.setWristAngle(203); // Human Player
              break;
            case 4:
              wrist.setWristAngle(180); // Human Player
              break;
            case 5:
              wrist.zeroWrist();
              break;
          }
        },
        wrist);
  }

  public static Command stopWrist(Wrist wrist) {
    return Commands.runOnce(
        () -> {
          wrist.moveWrist(0);
          wrist.setWristAngle(wrist.getAngle() + 1);
        },
        wrist);
  }
}
