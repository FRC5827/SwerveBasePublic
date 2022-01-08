package frc.lib.util;

public class SwerveModuleConstants {
    public final int m_driveMotorID;
    public final int m_angleMotorID;
    public final int m_cancoderID;
    public final double m_angleOffset;
    public final boolean m_invertDriveMotor;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     * @param invertDriveMotor
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, double angleOffset, boolean invertDriveMotor) {
        m_driveMotorID = driveMotorID;
        m_angleMotorID = angleMotorID;
        m_cancoderID = canCoderID;
        m_angleOffset = angleOffset;
        m_invertDriveMotor = invertDriveMotor;
    }
}
