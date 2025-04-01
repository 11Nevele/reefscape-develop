// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class LimeLight extends SubsystemBase {

  public static final String limelightName = "limelight-front"; // Default name
  private static final double offset_side = -0.185;
  private static final double offset_forward = 0.8;

  // private final AprilTagFieldLayout APRILTAGFIELDLAYOUT =
  public static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  // meter up from center.
  @AutoLog
  public static class LimeLightIOInputs {
    public int tagId = 0;

    public double current_r = 0;
    public double current_x = 0;
    public double current_y = 0;
    public double target_r = 0;
    public double target_x = 0;
    public double target_y = 0;
  }

  private LimeLightIOInputsAutoLogged limeLightInputs = new LimeLightIOInputsAutoLogged();

  public final HashMap<String, Pose2d> APRILTAG_TARGET_POSE = new HashMap<String, Pose2d>();

  private static final Transform2d transform_left =
      new Transform2d(offset_forward, offset_side, Rotation2d.fromDegrees(0));
  private static final Transform2d transform_right =
      new Transform2d(offset_forward, -offset_side, Rotation2d.fromDegrees(0));

  private Pose2d ReversePose(Pose2d pose) {
    return new Pose2d(
        pose.getX(), pose.getY(), pose.getRotation().rotateBy(new Rotation2d(Math.PI)));
  }

  public LimeLight() {

    // level 2,3
    APRILTAG_TARGET_POSE.put(
        "6L", ReversePose(TAG_LAYOUT.getTagPose(6).get().toPose2d().transformBy(transform_left)));
    APRILTAG_TARGET_POSE.put(
        "7L", ReversePose(TAG_LAYOUT.getTagPose(7).get().toPose2d().transformBy(transform_left)));
    APRILTAG_TARGET_POSE.put(
        "8L", ReversePose(TAG_LAYOUT.getTagPose(8).get().toPose2d().transformBy(transform_left)));
    APRILTAG_TARGET_POSE.put(
        "9L", ReversePose(TAG_LAYOUT.getTagPose(9).get().toPose2d().transformBy(transform_left)));
    APRILTAG_TARGET_POSE.put(
        "10L", ReversePose(TAG_LAYOUT.getTagPose(10).get().toPose2d().transformBy(transform_left)));
    APRILTAG_TARGET_POSE.put(
        "11L", ReversePose(TAG_LAYOUT.getTagPose(11).get().toPose2d().transformBy(transform_left)));

    APRILTAG_TARGET_POSE.put(
        "17L", ReversePose(TAG_LAYOUT.getTagPose(17).get().toPose2d().transformBy(transform_left)));
    APRILTAG_TARGET_POSE.put(
        "18L", ReversePose(TAG_LAYOUT.getTagPose(18).get().toPose2d().transformBy(transform_left)));
    APRILTAG_TARGET_POSE.put(
        "19L", ReversePose(TAG_LAYOUT.getTagPose(19).get().toPose2d().transformBy(transform_left)));
    APRILTAG_TARGET_POSE.put(
        "20L", ReversePose(TAG_LAYOUT.getTagPose(20).get().toPose2d().transformBy(transform_left)));
    APRILTAG_TARGET_POSE.put(
        "21L", ReversePose(TAG_LAYOUT.getTagPose(21).get().toPose2d().transformBy(transform_left)));
    APRILTAG_TARGET_POSE.put(
        "22L", ReversePose(TAG_LAYOUT.getTagPose(22).get().toPose2d().transformBy(transform_left)));

    APRILTAG_TARGET_POSE.put(
        "6R", ReversePose(TAG_LAYOUT.getTagPose(6).get().toPose2d().transformBy(transform_right)));
    APRILTAG_TARGET_POSE.put(
        "7R", ReversePose(TAG_LAYOUT.getTagPose(7).get().toPose2d().transformBy(transform_right)));
    APRILTAG_TARGET_POSE.put(
        "8R", ReversePose(TAG_LAYOUT.getTagPose(8).get().toPose2d().transformBy(transform_right)));
    APRILTAG_TARGET_POSE.put(
        "9R", ReversePose(TAG_LAYOUT.getTagPose(9).get().toPose2d().transformBy(transform_right)));
    APRILTAG_TARGET_POSE.put(
        "10R",
        ReversePose(TAG_LAYOUT.getTagPose(10).get().toPose2d().transformBy(transform_right)));
    APRILTAG_TARGET_POSE.put(
        "11R",
        ReversePose(TAG_LAYOUT.getTagPose(11).get().toPose2d().transformBy(transform_right)));

    APRILTAG_TARGET_POSE.put(
        "17R",
        ReversePose(TAG_LAYOUT.getTagPose(17).get().toPose2d().transformBy(transform_right)));
    APRILTAG_TARGET_POSE.put(
        "18R",
        ReversePose(TAG_LAYOUT.getTagPose(18).get().toPose2d().transformBy(transform_right)));
    APRILTAG_TARGET_POSE.put(
        "19R",
        ReversePose(TAG_LAYOUT.getTagPose(19).get().toPose2d().transformBy(transform_right)));
    APRILTAG_TARGET_POSE.put(
        "20R",
        ReversePose(TAG_LAYOUT.getTagPose(20).get().toPose2d().transformBy(transform_right)));
    APRILTAG_TARGET_POSE.put(
        "21R",
        ReversePose(TAG_LAYOUT.getTagPose(21).get().toPose2d().transformBy(transform_right)));
    APRILTAG_TARGET_POSE.put(
        "22R",
        ReversePose(TAG_LAYOUT.getTagPose(22).get().toPose2d().transformBy(transform_right)));

    LimelightHelpers.setCameraPose_RobotSpace(
        limelightName,
        0.381, // Forward offset (meters)
        0, // Side offset (meters)
        0.16, // Height offset (meters)
        0.0, // Roll (degrees)
        0.0, // Pitch (degrees)
        0.0 // Yaw (degrees)
        );
  }

  @Override
  public void periodic() {
    // limeLightInputs.current_r = getRobotPose().getRotation().getDegrees();
    // limeLightInputs.current_x = getRobotPose().getX();
    // limeLightInputs.current_y = getRobotPose().getY();
    Logger.processInputs("LimeLight", limeLightInputs);
  }

  /** Check if an AprilTag is detected */
  public boolean hasTarget() {
    return LimelightHelpers.getTV(limelightName);
  }

  /** Get horizontal offset (tx) from the crosshair */
  public double getTx() {
    return LimelightHelpers.getTX(limelightName);
  }

  /** Get vertical offset (ty) from the crosshair */
  public double getTy() {
    return LimelightHelpers.getTY(limelightName);
  }

  /** Get target area (ta) */
  public double getTa() {
    return LimelightHelpers.getTA(limelightName);
  }

  /** Get the AprilTag ID */
  public int getTagID() {
    int tagId = (int) LimelightHelpers.getFiducialID(limelightName);
    return tagId;
  }

  /** Get estimated robot pose from the Limelight */
  public Pose2d getRobotPose() {
    return LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
  }

  public Pose2d getTargetPose2D(boolean isLeft) {
    Pose2d targetPose = null;
    limeLightInputs.tagId = getTagID();
    System.out.println(getTagID());
    if (limeLightInputs.tagId != 0) {
      targetPose = APRILTAG_TARGET_POSE.get(limeLightInputs.tagId + (isLeft ? "L" : "R"));
      System.out.println(limeLightInputs.tagId + (isLeft ? "L" : "R"));
      if (targetPose == null) return null;

      limeLightInputs.target_r = targetPose.getRotation().getDegrees();
      limeLightInputs.target_x = targetPose.getX();
      limeLightInputs.target_y = targetPose.getY();
      limeLightInputs.current_r = getRobotPose().getRotation().getDegrees();
      limeLightInputs.current_x = getRobotPose().getX();
      limeLightInputs.current_y = getRobotPose().getY();
    }

    return targetPose;
  }
}
