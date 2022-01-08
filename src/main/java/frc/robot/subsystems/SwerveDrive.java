package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Nat;

//import edu.wpi.first.hal.SimDouble;
//import edu.wpi.first.hal.simulation.SimDeviceDataJNI;


public class SwerveDrive extends SubsystemBase {
    public SwerveDrivePoseEstimator m_swervePoseEstimator;
    public SwerveModule[] m_swerveModules;
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    // The Field2d class shows the field in the sim GUI
    private Field2d m_fieldSim;

    private int m_periodicCounter = 0;

    private final ShuffleboardTab m_driveShuffleboardTab = Shuffleboard.getTab("Drivetrain");
    private final NetworkTableEntry m_xEntry = m_driveShuffleboardTab.add("X", 0.0)
                                                    .withPosition(0, 1)
                                                    .withSize(1, 1)
                                                    .getEntry();
    private final NetworkTableEntry m_yEntry = m_driveShuffleboardTab.add("Y", 0.0)
                                                    .withPosition(0, 2)
                                                    .withSize(1, 1)
                                                    .getEntry();
    private final NetworkTableEntry m_Heading = m_driveShuffleboardTab.add("Heading", 0.0)
                                                    .withPosition(0, 3)
                                                    .withSize(1, 1)
                                                    .getEntry();
    private final NetworkTableEntry m_Pose = m_driveShuffleboardTab.add("Pose", "No Data")
                                                    .withPosition(2, 0)
                                                    .withSize(4, 1)
                                                    .getEntry();
    private final NetworkTableEntry[] m_moduleVelocityEntryArray;
    private final NetworkTableEntry[] m_moduleAngleEntryArray;
    private final NetworkTableEntry[] m_moduleCANCoderEntryArray;


    public SwerveDrive() {
        try
        {
            // sleep (delay) while gyro is initialized -- not super friendly but only happens once at init
            Thread.sleep(500);
        }
        catch (InterruptedException e)
        {
        }

        zeroHeading();
        
        Pose2d initialPose = new Pose2d();
        m_swervePoseEstimator = new SwerveDrivePoseEstimator(getR2dHeading(), initialPose, Constants.Swerve.kSwerveKinematics, 
                                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.01),    // model state std deviations (x, y, theta)
                                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.01),                // local measurement (encoder/gyro) std deviations
                                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1));      // vision measurement std deviation (x, y, theta)

        m_swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Module0.constants),
            new SwerveModule(1, Constants.Swerve.Module1.constants),
            new SwerveModule(2, Constants.Swerve.Module2.constants),
            new SwerveModule(3, Constants.Swerve.Module3.constants)
        };

        resetOdometry(initialPose);

        m_fieldSim = new Field2d();
        SmartDashboard.putData("Field", m_fieldSim);

        if (RobotBase.isSimulation()) {
            simInit();
        }


        m_moduleVelocityEntryArray = new NetworkTableEntry[m_swerveModules.length];
        m_moduleAngleEntryArray = new NetworkTableEntry[m_swerveModules.length];
        m_moduleCANCoderEntryArray = new NetworkTableEntry[m_swerveModules.length];

        ShuffleboardLayout[] m_shuffleboardModuleLayouts = {
            m_driveShuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList),
            m_driveShuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList),
            m_driveShuffleboardTab.getLayout("Rear Left Module", BuiltInLayouts.kList),
            m_driveShuffleboardTab.getLayout("Rear Right Module", BuiltInLayouts.kList)
        };

        for (SwerveModule module : m_swerveModules) {
            ShuffleboardLayout layout = m_shuffleboardModuleLayouts[module.m_number].withPosition(2 + (module.m_number * 2), 1).withSize(2, 4);
            m_moduleVelocityEntryArray[module.m_number] = layout.add("Velocity", 0.0).getEntry();
            m_moduleAngleEntryArray[module.m_number] = layout.add("Angle", 0.0).getEntry();
            m_moduleCANCoderEntryArray[module.m_number] = layout.add("CANCoder", 0.0).getEntry();
        }
    }

    public void simInit() {
        // Object for simulated inputs into Talon.
        // only add actual controlled Talons -- no need to add followers

        /*
        m_leftDriveSim = m_leftMaster.getSimCollection();
        m_rightDriveSim = m_rightMaster.getSimCollection();
        
        m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
            DriveConstants.kDrivetrainPlant_High,
            DriveConstants.kDriveGearbox,
            DriveConstants.kDriveGearing,
            DriveConstants.kTrackwidthMeters,
            DriveConstants.kWheelDiameterMeters / 2.0,  // radius
            null);//VecBuilder.fill(0, 0, 0.00005, 0.05, 0.05, 0.005, 0.005));   // std deviations for measurements to introduce noise
        */
      }
    

    public void drive(double forward, double strafe, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.kSwerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    forward, 
                                    strafe, 
                                    rotation, 
                                    getR2dHeading()
                                )
                                : new ChassisSpeeds(
                                    forward, 
                                    strafe, 
                                    rotation)
                                );
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule module : m_swerveModules) {
            module.setDesiredState(swerveModuleStates[module.m_number], isOpenLoop);
        }
    }    

    // Used by SwerveControllerCommand in Auto
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for (SwerveModule module : m_swerveModules) {
            module.setDesiredState(desiredStates[module.m_number], false);
        }
    }    

    public Pose2d getPose() {
        return m_swervePoseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        for (SwerveModule module : m_swerveModules) {
            module.resetDriveEncoder();
        }
        m_swervePoseEstimator.resetPosition(pose, getR2dHeading());
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : m_swerveModules) {
            states[module.m_number] = module.getState();
        }
        return states;
    }

    /**
     * Zeroes the heading of the robot.
     * If calling zeroHeading, the pose MUST also be reset AFTER this call
     * or it will appear that the robot has changed position.
     */
    public void zeroHeading() {
        m_gyro.zeroYaw();
    }

    public Rotation2d getR2dHeading() {
        return (Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -360 to 360
     */
    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.Swerve.invertGyro ? -1.0 : 1.0);
    }

    @Override
    public void periodic(){

        // update pose estimator/odometry
        m_swervePoseEstimator.update(getR2dHeading(), getStates());

        // update shuffleboard values
        for (SwerveModule module : m_swerveModules) {
            m_moduleVelocityEntryArray[module.m_number].setDouble(module.getVelocity());
            m_moduleAngleEntryArray[module.m_number].setDouble(module.getAngle());
            
            // getting CANCoder position seems to be relatively expensive, so only update occasionally
            if (m_periodicCounter % 10 == 0) {
                m_moduleCANCoderEntryArray[module.m_number].setDouble(module.getCanCoderPosition());
            }
        }

        // get updated position
        Pose2d currentPose = getPose();

        m_fieldSim.setRobotPose(currentPose);

        // pose string creation is relatively expensive, so only update stats occasionally
        if (m_periodicCounter % 10 == 0) {
            m_xEntry.setDouble(currentPose.getX());
            m_yEntry.setDouble(currentPose.getY());
            m_Heading.setDouble(currentPose.getRotation().getDegrees());

            m_Pose.setString(currentPose.toString());
        }
 
        m_periodicCounter++;
    }

    @Override
    public void simulationPeriodic() {
        // add code here for swerve drivetrain simulator once WPILib has support
    
        // update simulated NavX
        //int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        //SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        //double degrees = -m_drivetrainSimulator.getHeading().getDegrees();
        //angle.set(degrees);
    }

}