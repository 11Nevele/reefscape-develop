// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

// L1 13
// L2 42.5

// Algea L1
public class SuperStructureState {

  public static final double SOURCE_HEIGHT = 0.25;
  public static final double SOURCE_ANGLE = 50;

  public static final double L0_HEIGHT = 0.1; // default
  public static final double L1_HEIGHT = 12; // L2 Coral
  public static final double L2_HEIGHT = 24; // L2 Algea
  public static final double L3_HEIGHT = 40; // L3 Coral
  public static final double L4_HEIGHT = 48; // L3 Algea
  public static final double L5_HEIGHT = 3; // Human Player

  // intake 35
  public static final double L0_ANGLE = 144; // defalt
  public static final double L1_ANGLE = 155; // L2 Coral
  public static final double L2_ANGLE = 177; // L2 Algea
  public static final double L3_ANGLE = 155; // L3 Coral
  public static final double L4_ANGLE = 177; // L3 Algea
  public static final double L5_ANGLE = 190; // Human Player
  public static final double L6_ANGLE = 227; // Avoid Collsion with ground Intake

  // ground intake level
  public static final double GROUND0 = 10; // defalt
  public static final double GROUND1 = 50; // reef and human player
  public static final double GROUND2 = 50; // resting
  public static final double GROUND3 = 70; // intake

  //test test

  public double height;
  public double angle;
  public String name;

  public SuperStructureState(String name, double height, double angle) {
    this.name = name;
    this.height = height;
    this.angle = angle;
  }
}
