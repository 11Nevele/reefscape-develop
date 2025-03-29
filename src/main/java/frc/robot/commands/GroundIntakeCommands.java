// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.GroundIntake;

public class GroundIntakeCommands {
  public static Command intake(GroundIntake groundIntake) {
    return Commands.runOnce(
        () -> {
          groundIntake.moveIntake(0.25);
        },
        groundIntake);
  }

  public static Command stop(GroundIntake groundIntake) {
    return Commands.runOnce(
        () -> {
          groundIntake.moveIntake(0);
        },
        groundIntake);
  }

  public static Command manuelWrist(GroundIntake groundIntake, double spd) {
    return Commands.runOnce(
        () -> {
          groundIntake.manuelWrist(spd);
        },
        groundIntake);
  }

  public static Command setStage(GroundIntake groundIntake, int stage) {
    return Commands.runOnce(
        () -> {
          switch (stage) {
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
