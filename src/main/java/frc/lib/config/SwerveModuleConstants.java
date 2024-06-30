package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int canCoderID;
  public final Rotation2d angleOffset;
  public final boolean driveReversed;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param driveMotorID
   * @param angleMotorID
   * @param canCoderID
   * @param angleOffset
   * @param driveReversed
   */
  public SwerveModuleConstants(
      int driveMotorID, int angleMotorID,int cancoderID, Rotation2d angleOffset, boolean driveReversed) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.canCoderID=cancoderID;
    this.angleOffset = angleOffset;
    this.driveReversed = driveReversed;
  }
}
