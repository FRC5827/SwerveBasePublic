// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Controllers
    private final XboxController m_driverInput = new XboxController(0);

    // Drive Controls
    private final int m_translationAxis = XboxController.Axis.kLeftY.value;
    private final int m_strafeAxis = XboxController.Axis.kLeftX.value;
    private final int m_rotationAxis = XboxController.Axis.kRightX.value;

    // Driver Buttons
    private final JoystickButton m_zeroGyro = new JoystickButton(m_driverInput, XboxController.Button.kY.value);

    // Subsystems
    private final SwerveDrive m_swerveDrive = new SwerveDrive();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // default for WPILib is to output telemetry for all internal objects, which can be very costly
        // from both a memory allocation aspect as well as CPU time due to garbage collection,
        // so disable it here.
        LiveWindow.disableAllTelemetry();


        // set up Limelight or Photonvision here


        // Configure the button bindings
        configureButtonBindings();


        boolean fieldRelative = true;
        boolean openLoop = true;
        m_swerveDrive.setDefaultCommand(new TeleopSwerve(m_swerveDrive, m_driverInput, m_translationAxis, m_strafeAxis, m_rotationAxis, fieldRelative, openLoop));

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
      private void configureButtonBindings() {
          // Driver Buttons
          m_zeroGyro.whenPressed(new InstantCommand(() -> m_swerveDrive.zeroHeading()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(m_swerveDrive);
    }

    
    /**
     * Used to reset gyro heading and pose/odometry.  Useful at beginning of autononmous period,
     * particularly if the robot has been sitting powered on for some time as gyro may have drifted.
     *  
     * @param pose to reset to
     */
    public void resetHeadingAndOdometry(Pose2d pose)
    {
        m_swerveDrive.zeroHeading();
        m_swerveDrive.resetOdometry(pose);
    }
}
