// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlgeaCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorWristCommands;
import frc.robot.commands.GroundIntakeCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Wrist;
import frc.robot.subsystems.intake.GroundIntake;
import frc.robot.subsystems.intake.Shooter;
import frc.robot.subsystems.vision.LimeLight;
import frc.robot.subsystems.vision.LimelightHelpers;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static final boolean USEMAG2 = false;
  // Subsystems
  private final Drive drive;
  // private final LimeLight vision;
  // public final AlgeaIntake intake;
  public final Shooter shooter;
  public final Wrist wrist;
  public final Elevator elevator;
  public final LimeLight vision;
  public final GroundIntake groundIntake;

  // Controller
  public final XboxController m_controller = new CommandXboxController(0).getHID();
  public final CommandXboxController controller = new CommandXboxController(0);
  public final XboxController controller2 = new XboxController(1);
  public final CommandXboxController c_controller2 = new CommandXboxController(1);
  public final CommandJoystick keyboard = new CommandJoystick(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  public static boolean transferState = true;
  ShuffleboardTab autoSystem;
  private boolean intakeToggle = false;

  public Drive getDrive() {
    return drive;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // intake = new AlgeaIntake();
    shooter = new Shooter();
    wrist = new Wrist();
    elevator = new Elevator();
    vision = new LimeLight();
    groundIntake = new GroundIntake();
    NamedCommands.registerCommand("Intake", IntakeCommands.intake(wrist));
    NamedCommands.registerCommand("StopIntake", IntakeCommands.stop(wrist));
    NamedCommands.registerCommand("Outake", IntakeCommands.outake(wrist));
    NamedCommands.registerCommand(
        "CoralL2",
        ElevatorWristCommands.setElevatorStage(elevator, 1) // huamn player
            .andThen(ElevatorWristCommands.setWristLevel(wrist, 1))
            .andThen(GroundIntakeCommands.setStage(groundIntake, 0)));
    NamedCommands.registerCommand(
        "CoralL3",
        ElevatorWristCommands.setElevatorStage(elevator, 3) // huamn player
            .andThen(ElevatorWristCommands.setWristLevel(wrist, 3))
            .andThen(GroundIntakeCommands.setStage(groundIntake, 0)));
    NamedCommands.registerCommand(
        "IntakeHuman",
        ElevatorWristCommands.setElevatorStage(elevator, 5) // huamn player
            .andThen(ElevatorWristCommands.setWristLevel(wrist, 5))
            .andThen(GroundIntakeCommands.setStage(groundIntake, 0)));
    NamedCommands.registerCommand(
        "AlgeaL2",
        ElevatorWristCommands.setElevatorStage(elevator, 2) // L3 Algea
            .andThen(ElevatorWristCommands.setWristLevel(wrist, 2))
            .andThen(GroundIntakeCommands.setStage(groundIntake, 1)));

    NamedCommands.registerCommand(
        "Transfer",
        GroundIntakeCommands.intake(groundIntake, 0.5)
            .alongWith(AlgeaCommands.Transfer(shooter, 0.5)));
    NamedCommands.registerCommand(
        "StopTransfer",
        GroundIntakeCommands.intake(groundIntake, 0.0)
            .alongWith(AlgeaCommands.Transfer(shooter, 0.0)));

    // Real robot, instantiate hardware IO implementations
    // vision = new LimeLight();
    // wrist = new Wrist();
    // elevator = new Elevator();
    // intake = new Intake();

    drive =
        new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));

    ShuffleboardTab tab = Shuffleboard.getTab("Auto");
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoSystem = Shuffleboard.getTab("Auto");

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // transfer
    controller.rightBumper().onTrue(AlgeaCommands.Transfer(shooter, 0.5));
    controller.rightBumper().onFalse(AlgeaCommands.Transfer(shooter, 0));
    controller.y().onTrue(AlgeaCommands.Transfer(shooter, -0.5));
    controller.y().onFalse(AlgeaCommands.Transfer(shooter, 0));

    // shoot
    controller.rightTrigger().onTrue(AlgeaCommands.shoot(shooter, true));
    controller.rightTrigger().onFalse(AlgeaCommands.shoot(shooter, false));
    controller.x().onTrue(AlgeaCommands.shootRev(shooter, true));
    controller.x().onFalse(AlgeaCommands.shootRev(shooter, false));
    // controller.y().onTrue(Drive.stopWithX());
    // intake
    controller
        .leftBumper()
        .onTrue(
            IntakeCommands.intake(wrist).andThen(GroundIntakeCommands.intake(groundIntake, -0.5)));
    controller
        .leftBumper()
        .onFalse(IntakeCommands.stop(wrist).andThen(GroundIntakeCommands.stop(groundIntake)));
    controller
        .leftTrigger()
        .onTrue(
            IntakeCommands.outake(wrist).andThen(GroundIntakeCommands.intake(groundIntake, 0.5)));
    controller
        .leftTrigger()
        .onFalse(IntakeCommands.stop(wrist).andThen(GroundIntakeCommands.stop(groundIntake)));

    controller
        .povDown()
        .onTrue(
            (ElevatorWristCommands.setWristLevel(wrist, 0))
                .andThen(GroundIntakeCommands.setStage(groundIntake, 3))
                .andThen(ElevatorWristCommands.setElevatorStage(elevator, 0)));
    controller.povUp().onTrue(ElevatorWristCommands.setWristLevel(wrist, 6));
    controller
        .povLeft()
        .onTrue(
            ElevatorWristCommands.setElevatorStage(elevator, 2) // L3 Algea
                .andThen(ElevatorWristCommands.setWristLevel(wrist, 2))
                .andThen(GroundIntakeCommands.setStage(groundIntake, 1)));
    controller
        .povRight()
        .onTrue(
            ElevatorWristCommands.setElevatorStage(elevator, 4) // L3 Algea
                .andThen(ElevatorWristCommands.setWristLevel(wrist, 4))
                .andThen(GroundIntakeCommands.setStage(groundIntake, 1)));

    controller.a().onTrue(Commands.runOnce(() -> drive.setPose(new Pose2d())));

    /*
     * // manuel elevator
     * c_controller2.x().onTrue(ElevatorWristCommands.moveElevator(elevator, 0.5));
     * c_controller2.x().onFalse(ElevatorWristCommands.moveElevator(elevator, 0));
     * c_controller2.a().onTrue(ElevatorWristCommands.moveElevator(elevator, -0.5));
     * c_controller2.a().onFalse(ElevatorWristCommands.moveElevator(elevator, 0));
     *
     * // manuel wrist
     * c_controller2.y().onTrue(ElevatorWristCommands.moveWrist(wrist, 1));
     * c_controller2.y().onFalse(ElevatorWristCommands.stopWrist(wrist));
     *
     * c_controller2.rightBumper().onTrue(ElevatorWristCommands.moveWrist(wrist,
     * -1));
     * c_controller2.rightBumper().onFalse(ElevatorWristCommands.stopWrist(wrist));
     */

    controller.back().onTrue(GroundIntakeCommands.intake(groundIntake, 0.5));
    controller.back().onFalse(GroundIntakeCommands.stop(groundIntake));
    controller.start().onTrue(MoveToReeftarget(true, 0, 0));
    // controller.start().onFalse(GroundIntakeCommands.stop(groundIntake));

    // elevator and wrist
    /*controller
        .povDown()
        .onTrue(
            ElevatorWristCommands.setWristLevel(wrist, 0)
                .andThen(ElevatorWristCommands.setElevatorStage(elevator, 0)));

    controller
        .povLeft()
        .onTrue(
            ElevatorWristCommands.setWristLevel(wrist, 1)
                .andThen(ElevatorWristCommands.setElevatorStage(elevator, 1)));
    controller
        .povUp()
        .onTrue(
            ElevatorWristCommands.setWristLevel(wrist, 2)
                .andThen(ElevatorWristCommands.setElevatorStage(elevator, 2)));
    controller
        .povRight()
        .onTrue(
            ElevatorWristCommands.setWristLevel(wrist, 3)
                .andThen(ElevatorWristCommands.setElevatorStage(elevator, 8)));
    */
    keyboard
        .button(17)
        .onTrue(
            ElevatorWristCommands.setElevatorStage(elevator, 0) // Ground Intake
                .andThen(ElevatorWristCommands.setWristLevel(wrist, 0))
                .andThen(GroundIntakeCommands.setStage(groundIntake, 3)));
    keyboard.button(6).onTrue(MoveToReeftarget(true, 1, 1)); // L2 left coral
    keyboard.button(1).onTrue(MoveToReeftarget(true, 3, 3)); // L3 left coral
    keyboard.button(7).onTrue(MoveToReeftarget(false, 1, 1)); // L2 right coral
    keyboard.button(2).onTrue(MoveToReeftarget(false, 3, 3)); // L3 right coral
    keyboard
        .button(16)
        .onTrue(
            ElevatorWristCommands.setElevatorStage(elevator, 5) // huamn player
                .andThen(ElevatorWristCommands.setWristLevel(wrist, 5))
                .andThen(GroundIntakeCommands.setStage(groundIntake, 0)));
    keyboard
        .button(20)
        .onTrue(
            ElevatorWristCommands.setElevatorStage(elevator, 0) // defense mode
                .andThen(ElevatorWristCommands.setWristLevel(wrist, 6))
                .andThen(GroundIntakeCommands.setStage(groundIntake, 0)));

    // manuel ground intake wrist
    keyboard.button(8).onTrue(GroundIntakeCommands.manuelWrist(groundIntake, -1));
    keyboard.button(8).onFalse(GroundIntakeCommands.manuelWrist(groundIntake, 0));
    keyboard.button(13).onTrue(GroundIntakeCommands.manuelWrist(groundIntake, 1));
    keyboard.button(13).onFalse(GroundIntakeCommands.manuelWrist(groundIntake, 0));

    keyboard.button(3).onTrue(GroundIntakeCommands.reset(groundIntake));

    // manuel wrist
    keyboard.button(9).onTrue(ElevatorWristCommands.moveElevator(elevator, 1));
    keyboard.button(9).onFalse(ElevatorWristCommands.stopElevator(elevator));
    keyboard.button(14).onTrue(ElevatorWristCommands.moveElevator(elevator, -1));
    keyboard.button(14).onFalse(ElevatorWristCommands.stopElevator(elevator));

    keyboard.button(10).onTrue(ElevatorWristCommands.moveWrist(wrist, 1));
    keyboard.button(10).onFalse(ElevatorWristCommands.stopWrist(wrist));
    keyboard.button(15).onTrue(ElevatorWristCommands.moveWrist(wrist, -1));
    keyboard.button(15).onFalse(ElevatorWristCommands.stopWrist(wrist));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    autoSystem.add("A", autoChooser.get());
    Command autonomous = autoChooser.get();

    return autonomous;
  }

  // Camera

  public static PathPlannerPath createPath(Pose2d fromPose2d, Pose2d targetPose2d) {

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(fromPose2d, targetPose2d);
    double velocity = 3;
    double accelaration = 3;

    PathConstraints constraints =
        new PathConstraints(
            velocity, accelaration, 2 * Math.PI, 4 * Math.PI); // The constraints for this
    // path.
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); //
    // You can also use unlimited constraints, only limited by motor torque and
    // nominal battery voltage

    List<EventMarker> ListEM = Arrays.asList();
    List<RotationTarget> ListRT = Arrays.asList();
    List<ConstraintsZone> ListCZ = Arrays.asList();
    List<PointTowardsZone> ListPTZ = Arrays.asList();

    // Create the path using the waypoints created above
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            ListRT,
            ListPTZ,
            ListCZ,
            ListEM,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(
                0.0,
                targetPose2d
                    .getRotation()), // Goal end state. You can set a holonomic rotation here. If
            // using a differential drivetrain, the rotation will have no
            // effect.
            false);

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;

    return path;
  }

  public static PathPlannerPath GoReefTarget(Drive drive, LimeLight vision, boolean isLeft) {

    if (LimelightHelpers.getTV("limelight-front")) { // set position based on limelight
      drive.estimatePose();
    }

    Pose2d targetPose2d = vision.getTargetPose2D(isLeft);
    if (targetPose2d == null) {
      return null;
    }

    Pose2d fromPose2d =
        new Pose2d(drive.getPose().getX(), drive.getPose().getY(), drive.getPose().getRotation());
    return createPath(fromPose2d, targetPose2d);
  }

  Command cmd;

  public Command MoveToReeftarget(boolean isLeft, int elevatorLevel, int wristLevel) {
    return new InstantCommand(
            () -> {
              var path = GoReefTarget(drive, vision, isLeft);
              System.out.println("Tracking start");
              if (path != null) {
                // System.out.println("path found");
                cmd = AutoBuilder.followPath(path);
                drive.isTracking = true;
                cmd.schedule();
              }
            })
        .alongWith(ElevatorWristCommands.setElevatorStage(elevator, elevatorLevel))
        .alongWith(ElevatorWristCommands.setWristLevel(wrist, wristLevel))
        .alongWith(GroundIntakeCommands.setStage(groundIntake, 1))
        .andThen(
            new InstantCommand(
                () -> {
                  drive.isTracking = false;
                }));
  }
}
