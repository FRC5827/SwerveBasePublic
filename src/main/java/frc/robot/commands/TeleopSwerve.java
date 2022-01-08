package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private boolean m_fieldRelative;
    private boolean m_fieldRelativeBaseState;
    private boolean m_openLoop;
    private NetworkTableEntry m_fieldRelativeInitialStateShuffleboard;
    private NetworkTableEntry m_fieldRelativeStatusShuffleboard;
    private ShuffleboardTab m_driveShuffleboardTab;
    
    private SwerveDrive m_swerveDrive;
    private XboxController m_controller;
    private int m_translationAxis;
    private int m_strafeAxis;
    private int m_rotationAxis;

    private SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(Constants.kJoystickSlewRate);
    private SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(Constants.kJoystickSlewRate);
    private SlewRateLimiter rSpeedLimiter = new SlewRateLimiter(Constants.kJoystickSlewRate);


    // Driver control
    public TeleopSwerve(SwerveDrive swerveDrive, XboxController controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        m_swerveDrive = swerveDrive;
        addRequirements(swerveDrive);

        m_controller = controller;
        m_translationAxis = translationAxis;
        m_strafeAxis = strafeAxis;
        m_rotationAxis = rotationAxis;
        m_fieldRelative = fieldRelative;
        m_fieldRelativeBaseState = fieldRelative;
        m_openLoop = openLoop;

        // add button to toggle as well as status indicator
        m_driveShuffleboardTab = Shuffleboard.getTab("Drivetrain");
        m_fieldRelativeInitialStateShuffleboard = m_driveShuffleboardTab.add("Field Relative", fieldRelative)
                                                    .withPosition(0, 0)
                                                    .withSize(1, 1)
                                                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
        m_fieldRelativeStatusShuffleboard = m_driveShuffleboardTab.add("FieldRelativeStatus", fieldRelative)
                                                    .withPosition(1, 0)
                                                    .withSize(1, 1)
                                                    .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    }

    @Override
    public void execute() {
        // xbox controller provides negative values when pushing forward (y axis), so negate
        // negate strafe (left/right x axis stick) as we want positive when pushing left (positive y on field)
        // negate rotation as we want positive value when when pushing left (CCW is postive)
        double yAxis = -m_controller.getRawAxis(m_translationAxis);
        double xAxis = -m_controller.getRawAxis(m_strafeAxis);
        double rAxis = -m_controller.getRawAxis(m_rotationAxis);

        boolean dashboardSwitch = m_fieldRelativeInitialStateShuffleboard.getBoolean(true);
        if (dashboardSwitch) {
            m_fieldRelativeBaseState = true;
        }
        else
        {
            m_fieldRelativeBaseState = false;
        }

        // if bumper is being held, toggle field relative input
        if (m_controller.getBumper(Hand.kRight)) {
            m_fieldRelative = !m_fieldRelativeBaseState;
        }
        else {
            m_fieldRelative = m_fieldRelativeBaseState;
        }

        // update current field relative status on Shuffleboard
        m_fieldRelativeStatusShuffleboard.setBoolean(m_fieldRelative);

        // apply deadband
        yAxis = (Math.abs(yAxis) < Constants.kJoystickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.kJoystickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.kJoystickDeadband) ? 0 : rAxis;

        // square (cube) inputs
        yAxis = Math.abs(yAxis) * Math.abs(yAxis) * yAxis;
        xAxis = Math.abs(xAxis) * Math.abs(xAxis) * xAxis;
        rAxis = Math.abs(rAxis) * Math.abs(rAxis) * rAxis;

        // uncomment to scale down output
        //yAxis *= 0.5;
        //xAxis *= 0.5;
        //rAxis *= 0.5;

        // slew rate limiter
        yAxis = ySpeedLimiter.calculate(yAxis);
        xAxis = xSpeedLimiter.calculate(xAxis);
        rAxis = rSpeedLimiter.calculate(rAxis);
        
        // controller yAxis is forward movement, so is passed as x component of the translation
        // multiply [-1, 1] user input by velocities as these are desired chassis speeds
        double forward = yAxis * Constants.Swerve.maxSpeed;
        double strafe = xAxis * Constants.Swerve.maxSpeed;
        double rotation = rAxis * Constants.Swerve.maxAngularVelocity;

        m_swerveDrive.drive(forward, strafe, rotation, m_fieldRelative, m_openLoop);
    }
}
