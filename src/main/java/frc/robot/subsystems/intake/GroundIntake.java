// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
  private CANcoder wristEncoder;

  public static final double reduction =
      67.5; // wrist gearbox gear ration 60.0 * 60.0 * 30.0 / (10.0 * 18.0 * 12.0)
  // horizontal
  public static final double minAngle = 1;
  public static final double maxAngle = 100;

  double targetDegrees = 50;

  MotionMagicVoltage pMmPos = new MotionMagicVoltage(0);

  public GroundIntake() {
    wristEncoder = new CANcoder(30, TunerConstants.kCANBus);
    wristMotor = new TalonFX(25, TunerConstants.kCANBus);
    intakeMotor = new TalonFX(26, TunerConstants.kCANBus);

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 50.0;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    intakeConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;

    pMmPos.Slot = 0;
    pMmPos.EnableFOC = true;

    // Set up armTalonConfig
    intakeMotor.getConfigurator().apply(intakeConfig, 0.25);
    TalonFXConfiguration wristConfigure = new TalonFXConfiguration();
    CANcoderConfiguration wristCanCoderConfig = new CANcoderConfiguration();
    wristCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    wristCanCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    wristCanCoderConfig.MagnetSensor.MagnetOffset = -0.88; // -0.74975; // 0.107178

    wristEncoder.getConfigurator().apply(wristCanCoderConfig);

    wristConfigure.CurrentLimits.SupplyCurrentLimit = 50.0;
    wristConfigure.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfigure.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    wristConfigure.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfigure.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    wristConfigure.Feedback.FeedbackRemoteSensorID = 30;
    wristConfigure.Feedback.SensorToMechanismRatio = 1;
    wristConfigure.Feedback.RotorToSensorRatio = 1;

    wristConfigure.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;

    // Move the arm
    wristConfigure.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    wristConfigure.Slot0.kG = 0.05; // 0.35; // to hold the arm weight
    wristConfigure.Slot0.kP = 25; // 60; // 100; // adjust PID
    wristConfigure.Slot0.kI = 0;
    wristConfigure.Slot0.kD = 1;
    wristConfigure.Slot0.kS = 0;

    wristConfigure.MotionMagic.MotionMagicCruiseVelocity = 20; // 1.0; // 0.5;
    wristConfigure.MotionMagic.MotionMagicAcceleration = 10; // 2; // 1.0;
    wristConfigure.MotionMagic.MotionMagicJerk = 20; // 10; // 10;
    wristMotor.getConfigurator().apply(wristConfigure);
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
      wristMotor.setControl(pMmPos.withPosition(Units.degreesToRotations(targetDegrees)));
    if (DriverStation.isDisabled()) {
      wristMotor.setControl(new NeutralOut());
    }
  }

  public double getAngle() {
    return wristMotor.getPosition().getValueAsDouble() * 360.0;
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
      wristMotor.set(spd * 0.1);
      manuelMoving = true;
    }
  }

  public void resetWrist() {
    targetDegrees = 0;
    wristMotor.setPosition(0);
  }

  public void setWristAngle(double setPointAngle) {
    targetDegrees = MathUtil.clamp(setPointAngle, minAngle, maxAngle);
  }
}
