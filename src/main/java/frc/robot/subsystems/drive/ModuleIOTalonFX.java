// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.TalonFXSwerveConstants;

import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX angle motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/AngleAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX angleTalon;
  private final CANcoder cancoder;

  private final Queue<Double> timestampQueue;

  private final StatusSignal<Double> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> angleAbsolutePosition;
  private final StatusSignal<Double> anglePosition;
  private final Queue<Double> anglePositionQueue;
  private final StatusSignal<Double> angleVelocity;
  private final StatusSignal<Double> angleAppliedVolts;
  private final StatusSignal<Double> angleCurrent;

  private final boolean isAngleMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOTalonFX(int index) {
    switch (index) {
      case 0:
        driveTalon = new TalonFX(TalonFXSwerveConstants.FrontLeftModule.DRIVE_MOTOR_INDEX);
        angleTalon = new TalonFX(TalonFXSwerveConstants.FrontLeftModule.ANGLE_MOTOR_INDEX);
        cancoder = new CANcoder(TalonFXSwerveConstants.FrontLeftModule.CANCODER_INDEX);
        absoluteEncoderOffset = new Rotation2d(TalonFXSwerveConstants.FrontLeftModule.CANCODER_ANGLE_OFFSET); // MUST BE CALIBRATED
        driveTalon.getConfigurator().apply(TalonFXSwerveConstants.FrontLeftModule.swerveDriveFXConfig());
        setDriveBrakeMode(true);
        angleTalon.getConfigurator().apply(TalonFXSwerveConstants.FrontLeftModule.swerveAngleFXConfig());
        setAngleBrakeMode(true);
        cancoder.getConfigurator().apply(TalonFXSwerveConstants.FrontLeftModule.swerveCancoderConfig());
        break;
      case 1:
        driveTalon = new TalonFX(TalonFXSwerveConstants.FrontRightModule.DRIVE_MOTOR_INDEX);
        angleTalon = new TalonFX(TalonFXSwerveConstants.FrontRightModule.ANGLE_MOTOR_INDEX);
        cancoder = new CANcoder(TalonFXSwerveConstants.FrontRightModule.CANCODER_INDEX);
        absoluteEncoderOffset = new Rotation2d(TalonFXSwerveConstants.FrontRightModule.CANCODER_ANGLE_OFFSET); // MUST BE CALIBRATED
        driveTalon.getConfigurator().apply(TalonFXSwerveConstants.FrontRightModule.swerveDriveFXConfig());
        setDriveBrakeMode(true);
        angleTalon.getConfigurator().apply(TalonFXSwerveConstants.FrontRightModule.swerveAngleFXConfig());
        setAngleBrakeMode(true);
        cancoder.getConfigurator().apply(TalonFXSwerveConstants.FrontRightModule.swerveCancoderConfig());
        break;
      case 2:
        driveTalon = new TalonFX(TalonFXSwerveConstants.BackLeftModule.DRIVE_MOTOR_INDEX);
        angleTalon = new TalonFX(TalonFXSwerveConstants.BackLeftModule.ANGLE_MOTOR_INDEX);
        cancoder = new CANcoder(TalonFXSwerveConstants.BackLeftModule.CANCODER_INDEX);
        absoluteEncoderOffset = new Rotation2d(TalonFXSwerveConstants.BackLeftModule.CANCODER_ANGLE_OFFSET); // MUST BE CALIBRATED
        driveTalon.getConfigurator().apply(TalonFXSwerveConstants.BackLeftModule.swerveDriveFXConfig());
        setDriveBrakeMode(true);
        angleTalon.getConfigurator().apply(TalonFXSwerveConstants.BackLeftModule.swerveAngleFXConfig());
        setAngleBrakeMode(true);
        cancoder.getConfigurator().apply(TalonFXSwerveConstants.BackLeftModule.swerveCancoderConfig());        
        break;
      case 3:
        driveTalon = new TalonFX(TalonFXSwerveConstants.BackRightModule.DRIVE_MOTOR_INDEX);
        angleTalon = new TalonFX(TalonFXSwerveConstants.BackRightModule.ANGLE_MOTOR_INDEX);
        cancoder = new CANcoder(TalonFXSwerveConstants.BackRightModule.CANCODER_INDEX);
        absoluteEncoderOffset = new Rotation2d(TalonFXSwerveConstants.BackRightModule.CANCODER_ANGLE_OFFSET); // MUST BE CALIBRATED
        driveTalon.getConfigurator().apply(TalonFXSwerveConstants.BackRightModule.swerveDriveFXConfig());
        setDriveBrakeMode(true);
        angleTalon.getConfigurator().apply(TalonFXSwerveConstants.BackRightModule.swerveAngleFXConfig());
        setAngleBrakeMode(true);
        cancoder.getConfigurator().apply(TalonFXSwerveConstants.BackRightModule.swerveCancoderConfig());        
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();

    angleAbsolutePosition = cancoder.getAbsolutePosition();
    anglePosition = angleTalon.getPosition();
    anglePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(angleTalon, angleTalon.getPosition());
    angleVelocity = angleTalon.getVelocity();
    angleAppliedVolts = angleTalon.getMotorVoltage();
    angleCurrent = angleTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY, drivePosition, anglePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        angleAbsolutePosition,
        angleVelocity,
        angleAppliedVolts,
        angleCurrent);
    driveTalon.optimizeBusUtilization();
    angleTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        angleAbsolutePosition,
        anglePosition,
        angleVelocity,
        angleAppliedVolts,
        angleCurrent);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / TalonFXSwerveConstants.DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / TalonFXSwerveConstants.DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.angleAbsolutePosition =
        Rotation2d.fromRotations(angleAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.anglePosition =
        Rotation2d.fromRotations(anglePosition.getValueAsDouble() / TalonFXSwerveConstants.ANGLE_GEAR_RATIO);
    inputs.angleVelocityRadPerSec =
        Units.rotationsToRadians(angleVelocity.getValueAsDouble()) / TalonFXSwerveConstants.ANGLE_GEAR_RATIO;
    inputs.angleAppliedVolts = angleAppliedVolts.getValueAsDouble();
    inputs.angleCurrentAmps = new double[] {angleCurrent.getValueAsDouble()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / TalonFXSwerveConstants.DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryAnglePositions =
        anglePositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / TalonFXSwerveConstants.ANGLE_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    anglePositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setAngleVoltage(double volts) {
    angleTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setAngleBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isAngleMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    angleTalon.getConfigurator().apply(config);
  }
}
