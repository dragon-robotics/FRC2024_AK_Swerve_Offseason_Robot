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

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** Intake Subsystem Constants */
  public static class IntakeConstants {
    public static final int MOTOR_ID = 1;
    
    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final int STALL_CURRENT_LIMIT = 55;
    public static final int FREE_CURRENT_LIMIT = 40;
    public static final double SECONDARY_CURRENT_LIMIT = 60.0;
    public static final double RAMP_RATE_IN_SEC = 0.25; // Ramp rate in seconds
    
    // public static final int BEAM_BREAK_DIGITAL_CHANNEL = 0;
    public static final double NOTE_DETECT_CURRENT_THRESHOLD = 40.0;
  }

  /** Uptake Subsystem Constants */
  public static class UptakeConstants {
    public static final int LEFT_MOTOR_ID = 3;
    public static final int RIGHT_MOTOR_ID = 4;
    
    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final int STALL_CURRENT_LIMIT = 55;
    public static final int FREE_CURRENT_LIMIT = 40;
    public static final double SECONDARY_CURRENT_LIMIT = 60.0;
    public static final double RAMP_RATE_IN_SEC = 0.25; // Ramp rate in seconds
    
    public static final int NOTE_BEAM_BREAK_DIGITAL_CHANNEL = 0;
  }

  /** Shooter Subsystem Constants */
  public static class ShooterConstants {
    public static final int LED_CHANNEL = 9;

    public static final int TOP_MOTOR_ID = 5;
    public static final int BOTTOM_MOTOR_ID = 6;

    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final int STALL_CURRENT_LIMIT = 55;
    public static final int FREE_CURRENT_LIMIT = 40;
    public static final double SECONDARY_CURRENT_LIMIT = 60.0;
    public static final double RAMP_RATE_IN_SEC = 0.02; // Ramp rate in seconds

    public static final int NOTE_BEAM_BREAK_DIGITAL_CHANNEL = 1;

    public static final double P = 0.0;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double F = 0.0;
    public static final double IZ = 0.0;
    public static final double MIN_OUTPUT = -0.2;
    public static final double MAX_OUTPUT = 0.8;

    /* Desired and max RPM for the shooter (to be tuned later) */
    public static final double DESIRED_SHOOTER_RPM = 4000;
    public static final double MAX_RPM = 5000;
  }

  public static class ArmConstants {
    public static final int LEFT_MOTOR_ID = 7;
    public static final int RIGHT_MOTOR_ID = 9;

    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final int STALL_CURRENT_LIMIT = 40;
    public static final double SECONDARY_CURRENT_LIMIT = 60.0;

    public static final double P = 1.0;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double F = 0.156;
    public static final double IZ = 0.0;
    public static final double MIN_OUTPUT = -0.3;
    public static final double MAX_OUTPUT = 0.3;

    public static final int SMART_MOTION_SLOT = 0;
    public static final int SMART_MOTION_MAX_VELOCITY = 2000;
    public static final int SMART_MOTION_MIN_OUTPUT_VELOCITY = 0;
    public static final int SMART_MOTION_MAX_ACCELERATION = 1500;
    public static final int SMART_MOTION_ALLOWED_ERROR = 10;

    /* Desired absolute encoder setpoint for moving shooter and amp mechanism (to be tuned later using absolute encoder) */
    public static final double INITIAL_GOAL = 0.05;
    public static final double SHOOTER_GOAL = 0.13;
    public static final double AMP_GOAL = 0.32;
    public static final double AMP_ASSIST_GOAL = 0.4;
  }

  /** Climber Subsystem Constants */
  public static class ClimberConstants {
    public static final int LEFT_MOTOR_ID = 10;
    public static final int RIGHT_MOTOR_ID = 11;

    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final int STALL_CURRENT_LIMIT = 40;
    public static final double SECONDARY_CURRENT_LIMIT = 60.0;

    public static final float STARTING_LIMIT = 0.0f;
    public static final float ENDING_LIMIT = 100.0f;
  }

  public static class LEDConstants {
    public static final double RAINBOW_RAINBOW_PALETTE = -0.99;
    public static final double RAINBOW_PARTY_PALETTE = -0.97;
    public static final double RAINBOW_OCEAN_PALETTE = -0.95;
    public static final double RAINBOW_LAVE_PALETTE = -0.93;
    public static final double RAINBOW_FOREST_PALETTE = -0.91;
    public static final double RAINBOW_WITH_GLITTER = -0.89;
    public static final double CONFETTI = -0.87;
    public static final double SHOT_RED = -0.85;
    public static final double SHOT_BLUE = -0.83;
    public static final double SHOT_WHITE = -0.81;
    public static final double SINELON_RAINBOW_PALETTE = -0.79;
    public static final double SINELON_PARTY_PALETTE = -0.77;
    public static final double SINELON_OCEAN_PALETTE = -0.75;
    public static final double SINELON_LAVA_PALETTE = -0.73;
    public static final double SINELON_FOREST_PALETTE = -0.71;
    public static final double BPM_RAINBOW_PALETTE = -0.69;
    public static final double BPM_PARTY_PALETTE = -0.67;
    public static final double BPM_OCEAN_PALETTE = -0.65;
    public static final double BPM_LAVA_PALETTE = -0.63;
    public static final double BPM_FOREST_PALETTE = -0.61;
    public static final double FIRE_MEDIUM = -0.59;
    public static final double FIRE_LARGE = -0.57;
    public static final double TWINKLES_RAINBOW_PALETTE = -0.55;
    public static final double TWINKLES_PARTY_PALETTE = -0.53;
    public static final double TWINKLES_OCEAN_PALETTE = -0.51;
    public static final double TWINKLES_LAVA_PALETTE = -0.49;
    public static final double TWINKLES_FOREST_PALETTE = -0.47;
    public static final double COLOR_WAVES_RAINBOW_PALETTE = -0.45;
    public static final double COLOR_WAVES_PARTY_PALETTE = -0.43;
    public static final double COLOR_WAVES_OCEAN_PALETTE = -0.41;
    public static final double COLOR_WAVES_LAVA_PALETTE = -0.39;
    public static final double COLOR_WAVES_FOREST_PALETTE = -0.37;
    public static final double LARSON_SCANNER_RED = -0.35;
    public static final double LARSON_SCANNER_GRAY = -0.33;
    public static final double LIGHT_CHASE_RED = -0.31;
    public static final double LIGHT_CHASE_BLUE = -0.29;
    public static final double LIGHT_CHASE_GRAY = -0.27;
    public static final double HEARTBEAT_RED = -0.25;
    public static final double HEARTBEAT_BLUE = -0.23;
    public static final double HEARTBEAT_WHITE = -0.21;
    public static final double HEARTBEAT_GRAY = -0.19;
    public static final double BREATH_RED = -0.17;
    public static final double BREATH_BLUE = -0.15;
    public static final double BREATH_GRAY = -0.13;
    public static final double STROBE_RED = -0.11;
    public static final double STROBE_BLUE = -0.09;
    public static final double STROBE_GOLD = -0.07;
    public static final double STROBE_WHITE = -0.05;
    public static final double COLOR1_END_TO_END_BLEND_TO_BLACK = -0.03;
    public static final double COLOR1_LARSON_SCANNER = -0.01;
    public static final double COLOR1_LIGHT_CHASE = 0.01;
    public static final double COLOR1_HEARTBEAT_SLOW = 0.03;
    public static final double COLOR1_HEARTBEAT_MEDIUM = 0.05;
    public static final double COLOR1_HEARTBEAT_FAST = 0.07;
    public static final double COLOR1_BREATH_SLOW = 0.09;
    public static final double COLOR1_BREATH_FAST = 0.11;
    public static final double COLOR1_SHOT = 0.13;
    public static final double COLOR1_STROBE = 0.15;
    public static final double COLOR2_END_TO_END_BLEND_TO_BLACK = 0.17;
    public static final double COLOR2_LARSON_SCANNER = 0.19;
    public static final double COLOR2_LIGHT_CHASE = 0.21;
    public static final double COLOR2_HEARTBEAT_SLOW = 0.23;
    public static final double COLOR2_HEARTBEAT_MEDIUM = 0.25;
    public static final double COLOR2_HEARTBEAT_FAST = 0.27;
    public static final double COLOR2_BREATH_SLOW = 0.29;
    public static final double COLOR2_BREATH_FAST = 0.31;
    public static final double COLOR2_SHOT = 0.33;
    public static final double COLOR2_STROBE = 0.35;
    public static final double SPARKLE_1_ON_2 = 0.37;
    public static final double SPARKLE_2_ON_1 = 0.39;
    public static final double GRADIENT_1_AND_2 = 0.41;
    public static final double BPM_1_AND_2 = 0.43;
    public static final double END_TO_END_BLEND_1_TO_2 = 0.45;
    public static final double END_TO_END_BLEND_2_TO_1 = 0.47;
    public static final double COLOR_WAVES_1_AND_2 = 0.49;
    public static final double TWINKLES_1_AND_2 = 0.51;
    public static final double WAVES_1_AND_2 = 0.53;
    public static final double SINELON_1_AND_2 = 0.55;
    public static final double HOT_PINK = 0.57;
    public static final double DARK_RED = 0.59;
    public static final double RED = 0.61;
    public static final double RED_ORANGE = 0.63;
    public static final double ORANGE = 0.65;
    public static final double GOLD = 0.67;
    public static final double YELLOW = 0.69;
    public static final double LAWN_GREEN = 0.71;
    public static final double LIME = 0.73;
    public static final double DARK_GREEN = 0.75;
    public static final double GREEN = 0.77;
    public static final double BLUE_GREEN = 0.79;
    public static final double AQUA = 0.81;
    public static final double SKY_BLUE = 0.83;
    public static final double DARK_BLUE = 0.85;
    public static final double BLUE = 0.87;
    public static final double BLUE_VIOLET = 0.89;
    public static final double VIOLET = 0.91;
    public static final double WHITE = 0.93;
    public static final double GRAY = 0.95;
    public static final double DARK_GRAY = 0.97;
    public static final double BLACK = 0.99;
  }

  public static class OperatorConstants {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final int OPERATOR_BUTTON_PORT = 2;
    public static final int TEST_PORT = 3;
  }

  public static class CustomButtonBoxConstants {
    public static final int BTN_1 = 1;
    public static final int BTN_2 = 2;
    public static final int BTN_3 = 3;
    public static final int BTN_4 = 4;
    public static final int BTN_5 = 5;
    public static final int BTN_6 = 6;
    public static final int BTN_7 = 7;
    public static final int BTN_8 = 8;
    public static final int BTN_9 = 9;
    public static final int BTN_10 = 10;
    public static final int BTN_11 = 11;
    public static final int BTN_12 = 12;        
  }

  public static class JoystickConstants {
    // Joystick Analog Axis/Stick //
    public static final int STICK_LEFT_X = 0;
    public static final int STICK_LEFT_Y = 1;
    public static final int TRIGGER_LEFT = 2;
    public static final int TRIGGER_RIGHT = 3;
    public static final int STICK_RIGHT_X = 4;
    public static final int STICK_RIGHT_Y = 5;

    // Joystick Buttons //
    public static final int BTN_A = 1;
    public static final int BTN_B = 2;
    public static final int BTN_X = 3;
    public static final int BTN_Y = 4;
    public static final int BUMPER_LEFT = 5;
    public static final int BUMPER_RIGHT = 6;
    public static final int BTN_BACK = 7;
    public static final int BTN_START = 8;
    public static final int BTN_STICK_LEFT = 9;
    public static final int BTN_STICK_RIGHT = 10;
  }

  public static class Extreme3DProConstants {
    // Extreme 3D Pro Analog Axis/Stick //
    public static final int X_AXIS = 0;
    public static final int Y_AXIS = 1;
    public static final int ROTATE = 2;
    public static final int SLIDER = 3;

    // Extreme 3D Pro Buttons //
    public static final int BTN_TRIGGER = 1;
    public static final int BTN_THUMB = 2;
    public static final int BTN_BOT_LEFT = 3;
    public static final int BTN_BOT_RIGHT = 4;
    public static final int BTN_TOP_LEFT = 5;
    public static final int BTN_TOP_RIGHT = 6;
    public static final int BTN_7 = 7;
    public static final int BTN_8 = 8;
    public static final int BTN_9 = 9;
    public static final int BTN_10 = 10;
    public static final int BTN_11 = 11;
    public static final int BTN_12 = 12;
  }

  public static final class SwerveConstants {

    //#region Drivetrain Constants Definitions
    public static final double TRACK_WIDTH = Units.inchesToMeters(23);
    public static final double WHEELBASE_LENGTH = Units.inchesToMeters(23);
    
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;

    public static final double DRIVE_GEAR_RATIO = 6.12;
    public static final double ANGLE_GEAR_RATIO = 12.8;

    public static final double WHEEL_GRIP_COF = 1.19;

    public static final edu.wpi.first.math.geometry.Translation2d[] SWERVE_MODULE_LOCATIONS = {
            new edu.wpi.first.math.geometry.Translation2d(WHEELBASE_LENGTH / 2.0, TRACK_WIDTH / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(WHEELBASE_LENGTH / 2.0, -TRACK_WIDTH / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(-WHEELBASE_LENGTH / 2.0, TRACK_WIDTH / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(-WHEELBASE_LENGTH / 2.0, -TRACK_WIDTH / 2.0)
    };

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(SWERVE_MODULE_LOCATIONS);
    //#endregion

    //#region Swerve Motor Constants Definitions

    /* Current Limits */
    public static final int DRIVE_CONTINUOUS_SUPPLY_CURRENT = 50;
    public static final int DRIVE_PEAK_CURRENT_SUPPLY_LIMIT = 60;
    public static final double DRIVE_PEAK_SUPPLY_CURRENT_DURATION = 0.1;
    public static final boolean DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT = true;

    public static final int ANGLE_CONTINUOUS_SUPPLY_CURRENT = 25;
    public static final int ANGLE_PEAK_SUPPLY_CURRENT_LIMIT = 40;
    public static final double ANGLE_PEAK_SUPPLY_CURRENT_DURATION = 0.1;
    public static final boolean ANGLE_ENABLE_SUPPLY_CURRENT_LIMIT = true;
    
    public static final int DRIVE_STATOR_CURRENT_LIMIT = 100;
    public static final boolean DRIVE_ENABLE_STATOR_LIMIT = true;

    public static final int ANGLE_STATOR_CURRENT_LIMIT = 50;
    public static final boolean ANGLE_ENABLE_STATOR_LIMIT = true;

    /* Swerve Voltage */
    public static final double MAX_DRIVE_VOLTAGE = 11.0;
    public static final double MAX_ANGLE_VOLTAGE = 11.0;

    /* Ramp Rate */
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /* Neutral Modes */
    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;

    /* Motor Inverts */
    public static final boolean DRIVE_MOTOR_INVERT = false;
    public static final boolean ANGLE_MOTOR_INVERT = false;

    /* Angle Encoder Invert */
    public static final boolean CANCODER_INVERT = false;

    //#endregion

    //#region Swerve Profiling Values
    public static final double MAX_SPEED = 5.4864; // meters per second
    public static final double MAX_ACCELERATION = Double.MAX_VALUE; // meters per second squared - No acceleration limit
    public static final double MAX_ANGULAR_VELOCITY = 11.5;
    public static final double MAX_ATTAINABLE_SPEED = MAX_SPEED * 0.85; // Max out at 85% to make sure speeds are attainable (4.6 mps)
    //#endregion

    //#region Swerve PID Values
    public static final double DRIVE_P = 1.0;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;
    public static final double DRIVE_KS = 0.0;
    public static final double DRIVE_KV = MAX_DRIVE_VOLTAGE / MAX_SPEED;
    public static final double DRIVE_KA = MAX_DRIVE_VOLTAGE / (WHEEL_GRIP_COF * 9.81);


    public static final double ANGLE_P = 50.0;
    public static final double ANGLE_I = 0.0;
    public static final double ANGLE_D = 0.32;
    public static final double ANGLE_KS = 0.0;
    public static final double ANGLE_KV = 0.0;
    public static final double ANGLE_KA = 0.0;

    //#endregion

    //#region Swerve Control Profile
    /* Controller Invert */
    public static final boolean kInvertXAxis = false;
    public static final boolean kInvertYAxis = false;
    public static final boolean kInvertRAxis = false;
    public static final double kSwerveDeadband = 0.1;
    //#endregion


    //#region Swerve Module Specific
    /* Front Left Module - Module 0 */
    public static final class FrontLeftModule {
      public static final double CANCODER_ANGLE_OFFSET = 289.248046875;
      public static final int DRIVE_MOTOR_INDEX = 0;
      public static final int ANGLE_MOTOR_INDEX = 1;
      public static final int kCancoderIndex = 0;
      public static final boolean DRIVE_MOTOR_INVERTED = true;
      public static final boolean ANGLE_MOTOR_INVERTED = false;

      public static TalonFXConfiguration swerveDriveFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit = DRIVE_CONTINUOUS_SUPPLY_CURRENT;
        config.CurrentLimits.SupplyCurrentThreshold = DRIVE_PEAK_CURRENT_SUPPLY_LIMIT;
        config.CurrentLimits.SupplyTimeThreshold = DRIVE_PEAK_SUPPLY_CURRENT_DURATION;

        config.CurrentLimits.StatorCurrentLimitEnable = DRIVE_ENABLE_STATOR_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = DRIVE_STATOR_CURRENT_LIMIT;

        config.Voltage.PeakForwardVoltage = MAX_DRIVE_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -MAX_DRIVE_VOLTAGE;

        config.Slot0.kP = DRIVE_P;
        config.Slot0.kI = DRIVE_I;
        config.Slot0.kD = DRIVE_D;
        config.Slot0.kS = DRIVE_KS;
        config.Slot0.kV = DRIVE_KV;
        config.Slot0.kA = DRIVE_KA;

        config.MotorOutput.NeutralMode = DRIVE_NEUTRAL_MODE;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        return config;
      }

      public static TalonFXConfiguration swerveAngleFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = ANGLE_ENABLE_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit = ANGLE_CONTINUOUS_SUPPLY_CURRENT;
        config.CurrentLimits.SupplyCurrentThreshold = ANGLE_PEAK_SUPPLY_CURRENT_DURATION;
        config.CurrentLimits.SupplyTimeThreshold = ANGLE_PEAK_SUPPLY_CURRENT_DURATION;

        config.CurrentLimits.StatorCurrentLimitEnable = ANGLE_ENABLE_STATOR_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = ANGLE_STATOR_CURRENT_LIMIT;

        config.Voltage.PeakForwardVoltage = MAX_ANGLE_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -MAX_ANGLE_VOLTAGE;

        config.Slot0.kP = ANGLE_P;
        config.Slot0.kI = ANGLE_I;
        config.Slot0.kD = ANGLE_D;
        config.Slot0.kS = ANGLE_KS;
        config.Slot0.kV = ANGLE_KV;
        config.Slot0.kA = ANGLE_KA;

        config.MotorOutput.NeutralMode = ANGLE_NEUTRAL_MODE;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        config.OpenLoopRamps.TorqueOpenLoopRampPeriod = OPEN_LOOP_RAMP;

        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = CLOSED_LOOP_RAMP;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = CLOSED_LOOP_RAMP;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = CLOSED_LOOP_RAMP;

        return config;
      }
        
      public static CANcoderConfiguration swerveCancoderConfig() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        MagnetSensorConfigs magConfig = new MagnetSensorConfigs();
        magConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor = magConfig;
        return config;
      }
    }

    /* Front Right Module - Module 1 */
    public static final class FrontRightModule {
      public static final double CANCODER_ANGLE_OFFSET = 249.9609375;
      public static final int DRIVE_MOTOR_INDEX = 2;
      public static final int ANGLE_MOTOR_INDEX = 3;
      public static final int CANCODER_INDEX = 1;
      public static final boolean DRIVE_MOTOR_INVERTED = true;
      public static final boolean ANGLE_MOTOR_INVERTED = false;

      public static TalonFXConfiguration swerveDriveFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit = DRIVE_CONTINUOUS_SUPPLY_CURRENT;
        config.CurrentLimits.SupplyCurrentThreshold = DRIVE_PEAK_CURRENT_SUPPLY_LIMIT;
        config.CurrentLimits.SupplyTimeThreshold = DRIVE_PEAK_SUPPLY_CURRENT_DURATION;

        config.CurrentLimits.StatorCurrentLimitEnable = DRIVE_ENABLE_STATOR_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = DRIVE_STATOR_CURRENT_LIMIT;

        config.Voltage.PeakForwardVoltage = MAX_DRIVE_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -MAX_DRIVE_VOLTAGE;

        config.Slot0.kP = DRIVE_P;
        config.Slot0.kI = DRIVE_I;
        config.Slot0.kD = DRIVE_D;
        config.Slot0.kS = DRIVE_KS;
        config.Slot0.kV = DRIVE_KV;
        config.Slot0.kA = DRIVE_KA;

        config.MotorOutput.NeutralMode = DRIVE_NEUTRAL_MODE;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        return config;
      }

      public static TalonFXConfiguration swerveAngleFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = ANGLE_ENABLE_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit = ANGLE_CONTINUOUS_SUPPLY_CURRENT;
        config.CurrentLimits.SupplyCurrentThreshold = ANGLE_PEAK_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyTimeThreshold = ANGLE_PEAK_SUPPLY_CURRENT_DURATION;

        config.CurrentLimits.StatorCurrentLimitEnable = ANGLE_ENABLE_STATOR_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = ANGLE_STATOR_CURRENT_LIMIT;

        config.Voltage.PeakForwardVoltage = MAX_ANGLE_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -MAX_ANGLE_VOLTAGE;

        config.Slot0.kP = ANGLE_P;
        config.Slot0.kI = ANGLE_I;
        config.Slot0.kD = ANGLE_D;
        config.Slot0.kS = ANGLE_KS;
        config.Slot0.kV = ANGLE_KV;
        config.Slot0.kA = ANGLE_KA;

        config.MotorOutput.NeutralMode = ANGLE_NEUTRAL_MODE;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        config.OpenLoopRamps.TorqueOpenLoopRampPeriod = OPEN_LOOP_RAMP;

        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = CLOSED_LOOP_RAMP;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = CLOSED_LOOP_RAMP;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = CLOSED_LOOP_RAMP;

        return config;
      }
        
      public static CANcoderConfiguration swerveCancoderConfig() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        MagnetSensorConfigs magConfig = new MagnetSensorConfigs();
        magConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor = magConfig;
        return config;
      }
    }

    /* Back Left Module - Module 2 */
    public static final class BackLeftModule {
      public static final double CANCODER_ANGLE_OFFSET = 206.54296875;
      public static final int DRIVE_MOTOR_INDEX = 4;
      public static final int ANGLE_MOTOR_INDEX = 5;
      public static final int kCancoderIndex = 2;
      public static final boolean DRIVE_MOTOR_INVERTED = true;
      public static final boolean ANGLE_MOTOR_INVERTED = false;

      public static TalonFXConfiguration swerveDriveFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit = DRIVE_CONTINUOUS_SUPPLY_CURRENT;
        config.CurrentLimits.SupplyCurrentThreshold = DRIVE_PEAK_CURRENT_SUPPLY_LIMIT;
        config.CurrentLimits.SupplyTimeThreshold = DRIVE_PEAK_SUPPLY_CURRENT_DURATION;

        config.CurrentLimits.StatorCurrentLimitEnable = DRIVE_ENABLE_STATOR_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = DRIVE_STATOR_CURRENT_LIMIT;

        config.Voltage.PeakForwardVoltage = MAX_DRIVE_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -MAX_DRIVE_VOLTAGE;

        config.Slot0.kP = DRIVE_P;
        config.Slot0.kI = DRIVE_I;
        config.Slot0.kD = DRIVE_D;
        config.Slot0.kS = DRIVE_KS;
        config.Slot0.kV = DRIVE_KV;
        config.Slot0.kA = DRIVE_KA;

        config.MotorOutput.NeutralMode = DRIVE_NEUTRAL_MODE;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        return config;
      }

      public static TalonFXConfiguration swerveAngleFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = ANGLE_ENABLE_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit = ANGLE_CONTINUOUS_SUPPLY_CURRENT;
        config.CurrentLimits.SupplyCurrentThreshold = ANGLE_PEAK_SUPPLY_CURRENT_DURATION;
        config.CurrentLimits.SupplyTimeThreshold = ANGLE_PEAK_SUPPLY_CURRENT_DURATION;

        config.CurrentLimits.StatorCurrentLimitEnable = ANGLE_ENABLE_STATOR_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = ANGLE_STATOR_CURRENT_LIMIT;

        config.Voltage.PeakForwardVoltage = MAX_ANGLE_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -MAX_ANGLE_VOLTAGE;

        config.Slot0.kP = ANGLE_P;
        config.Slot0.kI = ANGLE_I;
        config.Slot0.kD = ANGLE_D;
        config.Slot0.kS = ANGLE_KS;
        config.Slot0.kV = ANGLE_KV;
        config.Slot0.kA = ANGLE_KA;

        config.MotorOutput.NeutralMode = ANGLE_NEUTRAL_MODE;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        config.OpenLoopRamps.TorqueOpenLoopRampPeriod = OPEN_LOOP_RAMP;

        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = CLOSED_LOOP_RAMP;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = CLOSED_LOOP_RAMP;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = CLOSED_LOOP_RAMP;

        return config;
      }
        
      public static CANcoderConfiguration swerveCancoderConfig() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        MagnetSensorConfigs magConfig = new MagnetSensorConfigs();
        magConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor = magConfig;
        return config;
      }

    }

    /* Back Left Module - Module 3 */
    public static final class BackRightModule {
      public static final double CANCODER_ANGLE_OFFSET = 347.16;
      public static final int DRIVE_MOTOR_INDEX = 6;
      public static final int ANGLE_MOTOR_INDEX = 7;
      public static final int kCancoderIndex = 3;
      public static final boolean DRIVE_MOTOR_INVERTED = true;
      public static final boolean ANGLE_MOTOR_INVERTED = false;

      public static TalonFXConfiguration swerveDriveFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit = DRIVE_CONTINUOUS_SUPPLY_CURRENT;
        config.CurrentLimits.SupplyCurrentThreshold = DRIVE_PEAK_CURRENT_SUPPLY_LIMIT;
        config.CurrentLimits.SupplyTimeThreshold = DRIVE_PEAK_SUPPLY_CURRENT_DURATION;

        config.CurrentLimits.StatorCurrentLimitEnable = DRIVE_ENABLE_STATOR_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = DRIVE_STATOR_CURRENT_LIMIT;

        config.Voltage.PeakForwardVoltage = MAX_DRIVE_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -MAX_DRIVE_VOLTAGE;

        config.Slot0.kP = DRIVE_P;
        config.Slot0.kI = DRIVE_I;
        config.Slot0.kD = DRIVE_D;
        config.Slot0.kS = DRIVE_KS;
        config.Slot0.kV = DRIVE_KV;
        config.Slot0.kA = DRIVE_KA;

        config.MotorOutput.NeutralMode = DRIVE_NEUTRAL_MODE;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        return config;
      }

      public static TalonFXConfiguration swerveAngleFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = ANGLE_ENABLE_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit = ANGLE_CONTINUOUS_SUPPLY_CURRENT;
        config.CurrentLimits.SupplyCurrentThreshold = ANGLE_PEAK_SUPPLY_CURRENT_DURATION;
        config.CurrentLimits.SupplyTimeThreshold = ANGLE_PEAK_SUPPLY_CURRENT_DURATION;

        config.CurrentLimits.StatorCurrentLimitEnable = ANGLE_ENABLE_STATOR_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = ANGLE_STATOR_CURRENT_LIMIT;

        config.Voltage.PeakForwardVoltage = MAX_ANGLE_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -MAX_ANGLE_VOLTAGE;

        config.Slot0.kP = ANGLE_P;
        config.Slot0.kI = ANGLE_I;
        config.Slot0.kD = ANGLE_D;
        config.Slot0.kS = ANGLE_KS;
        config.Slot0.kV = ANGLE_KV;
        config.Slot0.kA = ANGLE_KA;

        config.MotorOutput.NeutralMode = ANGLE_NEUTRAL_MODE;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        config.OpenLoopRamps.TorqueOpenLoopRampPeriod = OPEN_LOOP_RAMP;

        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = CLOSED_LOOP_RAMP;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = CLOSED_LOOP_RAMP;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = CLOSED_LOOP_RAMP;

        return config;
      }
        
      public static CANcoderConfiguration swerveCancoderConfig() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        MagnetSensorConfigs magConfig = new MagnetSensorConfigs();
        magConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor = magConfig;
        return config;
      }
    }
    //#endregion
  }
}
