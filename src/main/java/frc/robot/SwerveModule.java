package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

import frc.lib.util.SwerveModuleConstants;

//import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.ErrorCode;

public class SwerveModule {
    public int m_number;
    private double m_angleOffset;
    private TalonFX m_angleMotor;
    private TalonFX m_driveMotor;
    private CANCoder m_angleEncoder;
    private double m_lastAngleInTicks;
    private double m_lastVelocity;

    // for simulation
    //public TalonFXSimCollection m_angleMotorSim;
    //public TalonFXSimCollection m_driveMotorSim;


    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        m_number = moduleNumber;
        m_angleOffset = moduleConstants.m_angleOffset;
        
        // Angle Encoder Config
        m_angleEncoder = new CANCoder(moduleConstants.m_cancoderID);
        configAngleEncoder();

        // Angle Motor Config
        m_angleMotor = new TalonFX(moduleConstants.m_angleMotorID);
        configAngleMotor();

        // Drive Motor Config
        m_driveMotor = new TalonFX(moduleConstants.m_driveMotorID);
        configDriveMotor(moduleConstants.m_invertDriveMotor);

        m_lastAngleInTicks = m_angleMotor.getSelectedSensorPosition();
        m_lastVelocity = 0.0;

        if (RobotBase.isSimulation()) {
            // Object for simulated inputs into Talon.
            //m_angleMotorSim = m_angleMotor.getSimCollection();
            //m_driveMotorSim = m_driveMotor.getSimCollection();
        }
      
    }

    public static void checkCtreError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            throw new RuntimeException(String.format("%s: %s", message, errorCode.toString()));
        }
    }


    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        boolean bUseLastVelocity = false;
        double velocity = 0.0;

        if (m_angleMotor.hasResetOccurred()) {
            System.out.println("Angle motor reset detected! Recalibrating angle and setting status frame rates.");
            configAngleMotor();
            return;
        }

        double currentTicks = m_angleMotor.getSelectedSensorPosition();
        Rotation2d currentRotation = Rotation2d.fromDegrees(currentTicks * Constants.Swerve.kAngleEncoderDegreesPerTick);

        desiredState = SwerveModuleState.optimize(desiredState, currentRotation);

        // find the rotation difference between current module angle and new calculated angle
        Rotation2d rotationDelta = desiredState.angle.minus(currentRotation);

        // find the new setpoint position in ticks based on the difference
        double deltaTicks = rotationDelta.getDegrees() * Constants.Swerve.kAngleEncoderTicksPerDegree;
        double desiredTicks = currentTicks + deltaTicks;
        double angle = desiredTicks;

        // prevent rotating the module angle if speed is less then 1%.  Avoids jittering.
        if (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
        {
            angle = m_lastAngleInTicks;
            // Angle state at no velocity is likely the home position (0 degrees), and we aren't setting the angle since velocity
            // is too low.  Thus use last angle state and set velocity to zero.
            velocity = 0.0;
        }

        // uncomment below and comment line after if switching from motion magic to straight PID
        // m_angleMotor.set(TalonFXControlMode.Position, angle); 
        m_angleMotor.set(TalonFXControlMode.MotionMagic, angle);
        m_lastAngleInTicks = angle;

        // wait for angle to be within tolerance before setting new drive motor output
        if ((Math.abs(rotationDelta.getDegrees()) <= (Constants.Swerve.kAngleDeltaForDrive))) {
            bUseLastVelocity = true;
        }

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            m_driveMotor.set(TalonFXControlMode.PercentOutput, percentOutput);
        }
        else {
            // if angle was not close enough, do not set new output value on drive motor
            if (!bUseLastVelocity) {
                // Talon/Falcon API expects velocity in encoder unit change / 100ms, so convert meters/s to units/100ms.
                // Some platforms (such as RoboRio's ARM Cortex-A9) are slower for div than mul, so multiply instead.
                // Java compiler may be smart enough to convert divide to multiply for this constant, but use mul anyway.
                velocity = (desiredState.speedMetersPerSecond / Constants.Swerve.kDriveEncoderDistancePerPulse) * 0.1d;
            }
            else {
                velocity = m_lastVelocity;
            }
                
            m_driveMotor.set(TalonFXControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond) / Constants.Swerve.kNominalBatteryVoltage);
            m_lastVelocity = velocity;
        }
    }

    // consider calling this if we have a brownout
    private void resetToAbsolute() {
        double absolutePosition = (getCanCoderPosition() - m_angleOffset) * Constants.Swerve.kAngleEncoderTicksPerDegree;
        checkCtreError(m_angleMotor.setSelectedSensorPosition(absolutePosition, 0, Constants.Swerve.kTalonConfigTimeout), "Error setting angle Talon " + m_number + " sensor position");
    }

    private void configAngleEncoder() {        
        checkCtreError(m_angleEncoder.configFactoryDefault(Constants.Swerve.kTalonConfigTimeout), "Error configuring CANcoder " + m_number + " default settings");
        checkCtreError(m_angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig, Constants.Swerve.kTalonConfigTimeout), "Error configuring CANCoder " + m_number + " configuration");
        checkCtreError(m_angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, Constants.Swerve.kTalonCANStatusSlowRateInMs), "Error setting CANCoder " + m_number + " SensorData period");
    }

    private void configAngleMotor() {
        checkCtreError(m_angleMotor.configFactoryDefault(Constants.Swerve.kTalonConfigTimeout), "Error configuring angle Talon " + m_number + " default settings");
        checkCtreError(m_angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig, Constants.Swerve.kTalonConfigTimeout), "Error configuring angle Talon " + m_number + " configuration");
        m_angleMotor.setInverted(Constants.Swerve.invertAngleMotor);
        m_angleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        m_angleMotor.hasResetOccurred();    // toggle flag indicating device was reset (false for subsequent checks, unless reset actually occurred)
        
        // for motion magic - decrease default frame periods so data isn't stale
        checkCtreError(m_angleMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, Constants.Swerve.kTalonCANStatusFastRateInMs, Constants.Swerve.kTalonConfigTimeout), "Error setting angle Talon " + m_number + " status_10 period");
        checkCtreError(m_angleMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.Swerve.kTalonCANStatusFastRateInMs, Constants.Swerve.kTalonConfigTimeout), "Error setting angle Talon " + m_number + " status_13 period");
        
        // reduce CAN usage since we don't need this data as frequently
        checkCtreError(m_angleMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, Constants.Swerve.kTalonCANStatusVerySlowRateInMs, Constants.Swerve.kTalonConfigTimeout), "Error setting angle Talon " + m_number + " status_1 period");
        
        resetToAbsolute();
    }

    private void configDriveMotor(boolean invertDriveMotor) {        
        checkCtreError(m_driveMotor.configFactoryDefault(Constants.Swerve.kTalonConfigTimeout), "Error configuring drive Talon " + m_number + " default settings");
        checkCtreError(m_driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig, Constants.Swerve.kTalonConfigTimeout), "Error configuring drive Talon " + m_number + " configuration");
        m_driveMotor.setInverted(invertDriveMotor);
        m_driveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        m_driveMotor.hasResetOccurred();    // toggle flag indicating device was reset (false for subsequent checks, unless reset actually occurred)

        // reduce CAN usage since we don't need this data as frequently
        checkCtreError(m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, Constants.Swerve.kTalonCANStatusVerySlowRateInMs, Constants.Swerve.kTalonConfigTimeout), "Error setting angle Talon " + m_number + " status_1 period");

        checkCtreError(m_driveMotor.setSelectedSensorPosition(0, 0, Constants.Swerve.kTalonConfigTimeout), "Error setting drive Talon " + m_number + " sensor position");
    }

    public double getCanCoderPosition() {
        return m_angleEncoder.getAbsolutePosition();
    }

    /**
     * Returns the module's drive motor velocity in meters per second and angle in degrees
     * 
     * @return a SwerveModuleState instance
     */
    public SwerveModuleState getState() {
        // multiply by 10 as CTRE motor controllers provides velocity in units per 100ms, so convert to units/second
        double velocity = 10.0 * m_driveMotor.getSelectedSensorVelocity() * Constants.Swerve.kDriveEncoderDistancePerPulse;
        Rotation2d angle = Rotation2d.fromDegrees(m_angleMotor.getSelectedSensorPosition() * Constants.Swerve.kAngleEncoderDegreesPerTick);
        return new SwerveModuleState(velocity, angle);
    }

    /**
     * Returns the drive motor velocity in meters per second
     * 
     * @return velocity in m/s
     */
    public double getVelocity() {
        return (10.0 * m_driveMotor.getSelectedSensorVelocity() * Constants.Swerve.kDriveEncoderDistancePerPulse);
    }

    /**
     * Returns the angle of the module in degrees
     * @return angle of module in degrees
     */
    public double getAngle() {
        return (m_angleMotor.getSelectedSensorPosition() * Constants.Swerve.kAngleEncoderDegreesPerTick);
    }
    
    /**
     * Resets drive motor encoder to zero.  Call when resetting odometry position.
     */
    public void resetDriveEncoder() {
        checkCtreError(m_driveMotor.setSelectedSensorPosition(0, 0, Constants.Swerve.kTalonConfigTimeout), "Error setting drive Talon " + m_number + " sensor position");
    }
    
}