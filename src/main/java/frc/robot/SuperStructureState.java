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
  public static final double L1_HEIGHT = 20; // L2 Coral
  public static final double L2_HEIGHT = 30; // L2 Algea
  public static final double L3_HEIGHT = 48; // L3 Coral
  public static final double L4_HEIGHT = 48; // L3 Algea
  public static final double L5_HEIGHT = 5; // Human Player

  // intake 35
  public static final double L0_ANGLE = 100; // defalt
  public static final double L1_ANGLE = 137; // L2 Coral
  public static final double L2_ANGLE = 10; // L2 Algea
  public static final double L3_ANGLE = -30; // L3 Coral
  public static final double L4_ANGLE = 10; // L3 Algea
  public static final double L5_ANGLE = -100; // Human Player

  public double height;
  public double angle;
  public String name;

  public SuperStructureState(String name, double height, double angle) {
    this.name = name;
    this.height = height;
    this.angle = angle;
  }
}
