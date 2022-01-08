package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;


    public CTREConfigs() {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        // Swerve Angle Motor Configurations
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.kAngleEnableCurrentLimit, 
            Constants.Swerve.kAngleCurrentLimit, 
            Constants.Swerve.kAngleThresholdCurrent, 
            Constants.Swerve.kAngleThresholdDuration);

        swerveAngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.Swerve.angleKF;
        swerveAngleFXConfig.neutralDeadband = Constants.Swerve.kAngleNeutralDeadband;
        swerveAngleFXConfig.motionCruiseVelocity = Constants.Swerve.kMotionCruiseVelocity;
        swerveAngleFXConfig.motionAcceleration = Constants.Swerve.kMotionAcceleration;
        swerveAngleFXConfig.motionCurveStrength = Constants.Swerve.kMotionCurveStrength;
        
        swerveAngleFXConfig.peakOutputForward = Constants.Swerve.maxSteerOutput;
        swerveAngleFXConfig.peakOutputReverse = -Constants.Swerve.maxSteerOutput;

        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;


        // Swerve Drive Motor Configuration
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.kDriveEnableCurrentLimit, 
            Constants.Swerve.kDriveCurrentLimit, 
            Constants.Swerve.kDriveThresholdCurrent, 
            Constants.Swerve.kDriveThresholdDuration);

        swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = Constants.Swerve.kOpenLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.Swerve.kClosedLoopRamp;

        
        // Swerve CANCoder Configuration
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        

    }

}