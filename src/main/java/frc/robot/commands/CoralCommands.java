// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Wrist;

public class CoralCommands {
  private CoralCommands() {}

  public static Command moveIntake(Wrist wrist) {
    return Commands.run(
        () -> {
          wrist.moveIntake(1);
        },
        wrist);
  }
}
