package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double kJoystickDeadband = 0.08;
    public static final double kJoystickSlewRate = 6.0;

    public static final class Swerve {
        // always ensure gyro is counter-clock-wise positive
        public static final boolean invertGyro = true;

        // drivetrain constants
        public static final double kTrackWidth = Units.inchesToMeters(19.50);
        public static final double kWheelBase = Units.inchesToMeters(19.50);
        public static final double kWheelDiameter = Units.inchesToMeters(4.00);
        public static final double kWheelCircumference = kWheelDiameter * Math.PI;

        // open loop (percent output) ramp probably better handled with our slew rate limiter
        public static final double kOpenLoopRamp = 0.0;
        public static final double kClosedLoopRamp = 0.0;
        public static final int kTalonConfigTimeout = 250;
        public static final int kTalonCANStatusFastRateInMs = 10;
        public static final int kTalonCANStatusSlowRateInMs = 100;
        public static final int kTalonCANStatusVerySlowRateInMs = 250;

        // SDS Swerve Module MK3 fast drive gearing
        public static final double kDriveGearRatio = (6.86 / 1.0);
        public static final double kAngleGearRatio = (12.8 / 1.0);

        public static final double kDriveEncoderTicksPerRev = 2048.0 * kDriveGearRatio;  // 2048 ticks for Falcon 500
        public static final double kDriveEncoderDistancePerPulse =
            (kWheelDiameter * Math.PI) / (double) kDriveEncoderTicksPerRev;
        
        public static final double kAngleEncoderTicksPerRev = 2048.0 * kAngleGearRatio;
        public static final double kAngleEncoderTicksPerDegree = kAngleEncoderTicksPerRev / 360.0;
        public static final double kAngleEncoderDegreesPerTick = 360.0 / kAngleEncoderTicksPerRev;

        public static final double kNominalBatteryVoltage = 12.0;

        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

        // swerve current limiting
        public static final boolean kAngleEnableCurrentLimit = true;
        public static final int kAngleCurrentLimit = 20;
        public static final int kAngleThresholdCurrent = 30;
        public static final double kAngleThresholdDuration = 0.2;

        public static final boolean kDriveEnableCurrentLimit = true;
        public static final int kDriveCurrentLimit = 35;
        public static final int kDriveThresholdCurrent = 60;
        public static final double kDriveThresholdDuration = 0.5;

        // motion magic config values
        // 90 degree turn is 6553 ticks with 12.8 gear ratio.  Full output of Falcon with no load is ~6300rpm.
        // If we target 60% output, result is ~63 rotations per second (~6.3 rotations per 100ms, or 12902 ticks/100ms).
        // Acceleration is units/100ms per second, so multiply by 20 to reach specified velocity in 50ms
        // note that angleKP needs to be high enough to reach the specified velocity
        public static final double kMotionCruiseVelocity = 12902;
        public static final double kMotionAcceleration = kMotionCruiseVelocity * 20;
        public static final int kMotionCurveStrength = 0;
        // when motion magic is used, what wheel angle delta is acceptable before adding drive power
        public static final double kAngleDeltaForDrive = 20.0;
        public static final double kAngleNeutralDeadband = 0.01;

        // angle motor PID values
        public static final double angleKP = 0.8;
        public static final double angleKI = 0.0;
        public static final double angleKD = 8.0;
        public static final double angleKF = 0.0;

        // angle motor peak output -- reevaluate PIDF values if changed
        public static final double maxSteerOutput = 1.0;

        // drive motor PID values
        public static final double driveKP = 0.0010;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        // drive motor characterization values
        // use robot characterization tool to determine these
        // 2022 season is expected to use the new sysid tool instead of prior year's python scripts
        // https://github.com/wpilibsuite/sysid
        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;

        // max swerve speed and rotation
        public static final double maxSpeed = 4.75; // meters per second, theoretical max is 16.2 ft/s or 4.9 m/s
        public static final double maxAngularVelocity = 1.5 * (2 * Math.PI); // 1.5 rotations/s (radians)

        // Talon FX neutral modes
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        // angle and drive motor inverts
        public static final boolean invertDriveMotor = true;
        public static final boolean invertAngleMotor = false;

        // angle encoder invert
        public static final boolean canCoderInvert = false;


        // module specific constants

        // Modules on each side should have wheels consistent with each other.
        // In other words, bevel gear should be facing the same direction across all modules.
        // Adjust invertDriveMotor if forward/backward are reversed.

        // front left module - module 0
        public static final class Module0 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 10;
            public static final double angleOffset = 324.76;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, invertDriveMotor);
        }

        // front right module - module 1
        public static final class Module1 {
            public static final int driveMotorID = 20;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 20;
            public static final double angleOffset = 47.19;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, invertDriveMotor);
        }
        
        // rear left module - module 2
        public static final class Module2 {
            public static final int driveMotorID = 30;
            public static final int angleMotorID = 31;
            public static final int canCoderID = 30;
            public static final double angleOffset = 77.78;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, invertDriveMotor);
        }

        // rear right module - module 3
        public static final class Module3 {
            public static final int driveMotorID = 40;
            public static final int angleMotorID = 41;
            public static final int canCoderID = 40;
            public static final double angleOffset = 344.44;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, invertDriveMotor);
        }

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Constants.Swerve.maxAngularVelocity;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }

}
