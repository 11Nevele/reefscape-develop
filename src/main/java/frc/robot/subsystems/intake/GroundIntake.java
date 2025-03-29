// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.elevator.WristIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class GroundIntake extends SubsystemBase {
  private TalonFX wristMotor;
  private TalonFX intakeMotor;

  public static final double reduction =
      67.5; // wrist gearbox gear ration 60.0 * 60.0 * 30.0 / (10.0 * 18.0 * 12.0)
  // horizontal
  public static final double minAngle = 1;
  public static final double maxAngle = 40;

  double targetDegrees = 15;

  MotionMagicVoltage pMmPos = new MotionMagicVoltage(0);

  public GroundIntake() {
    wristMotor = new TalonFX(25, TunerConstants.kCANBus);
    intakeMotor = new TalonFX(26, TunerConstants.kCANBus);
    TalonFXConfiguration armTalonConfig = new TalonFXConfiguration();
    armTalonConfig.CurrentLimits.SupplyCurrentLimit = 50.0;
    armTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armTalonConfig.Feedback.SensorToMechanismRatio = reduction;
    armTalonConfig.Feedback.RotorToSensorRatio = 1;

    armTalonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;

    // Move the arm
    armTalonConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armTalonConfig.Slot0.kG = 0.1; // 0.35; // to hold the arm weight
    armTalonConfig.Slot0.kP = 50; // 60; // 100; // adjust PID
    armTalonConfig.Slot0.kI = 0;
    armTalonConfig.Slot0.kD = 1;
    armTalonConfig.Slot0.kS = 0;

    armTalonConfig.MotionMagic.MotionMagicCruiseVelocity = 8; // 1.0; // 0.5;
    armTalonConfig.MotionMagic.MotionMagicAcceleration = 2; // 2; // 1.0;
    armTalonConfig.MotionMagic.MotionMagicJerk = 20; // 10; // 10;

    pMmPos.Slot = 0;
    pMmPos.EnableFOC = true;

    // Set up armTalonConfig
    wristMotor.getConfigurator().apply(armTalonConfig, 0.25);
    intakeMotor.getConfigurator().apply(armTalonConfig, 0.25);
  }

  @AutoLog
  public static class WristIOInputs {
    public boolean motorConnected = true;
    public boolean encoderConnected = false;
    public double targetAngle = 0.0;
    public double currentAngle = 0.0;
    public boolean manuelMoving = false;
  }

  public final WristIOInputsAutoLogged pivotInputs = new WristIOInputsAutoLogged();
  private boolean manuelMoving = false;

  @Override
  public void periodic() {
    pivotInputs.motorConnected = wristMotor.isConnected();
    pivotInputs.targetAngle = targetDegrees;
    pivotInputs.currentAngle = getAngle();
    Logger.processInputs("GroundIntake", pivotInputs);
    if (!manuelMoving)
      wristMotor.setControl(pMmPos.withPosition(Units.degreesToRotations(targetDegrees + 1)));
    if (DriverStation.isDisabled()) {
      wristMotor.setControl(new NeutralOut());
    }
  }

  public double getAngle() {
    return wristMotor.getPosition().getValueAsDouble() * 360;
  }

  public void moveIntake(double spd) {
    intakeMotor.set(spd);
  }

  public void manuelWrist(double spd) {
    if (spd == 0) {
      manuelMoving = false;
      wristMotor.stopMotor();
      targetDegrees = getAngle();
    } else {
      wristMotor.set(spd * 0.15);
      manuelMoving = true;
    }
  }

  public void resetWrist() {
    wristMotor.setPosition(0);
  }

  public void setWristAngle(double setPointAngle) {
    targetDegrees = MathUtil.clamp(setPointAngle, minAngle, maxAngle);
  }
}
