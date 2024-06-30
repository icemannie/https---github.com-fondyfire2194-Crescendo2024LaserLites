// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class WheelRadiusCharacterization extends Command {
  private final SwerveSubsystem swerve;

  private final double characterizationSpeed = Units.degreesToRadians(25.0);
  private final SlewRateLimiter angularRateLimiter = new SlewRateLimiter(Units.degreesToRadians(20.0));

  private double lastGyroYaw = 0.0;
  private double accumYaw = 0.0;

  private double[] startWheelPositions;

  private double currentWheelRadius = 2.0;

  /** Creates a new WheelRadiusCharacterization. */
  public WheelRadiusCharacterization(SwerveSubsystem swerve) {
    this.swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startWheelPositions = swerve.getWheelRadiusCharacterizationPosition();
    lastGyroYaw = swerve.getHeading().getRadians();
    accumYaw = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angularSpeed = angularRateLimiter.calculate(characterizationSpeed);

    swerve.drive(0.0, 0.0, angularSpeed, false, true, true);

    double currentYaw = swerve.getHeading().getRadians();

    accumYaw += MathUtil.angleModulus(currentYaw - lastGyroYaw);
    lastGyroYaw = currentYaw;

    double averagePosition = 0.0;
    double[] wheelPositions = swerve.getWheelRadiusCharacterizationPosition();
    for (int i = 0; i < 4; i++) {
      averagePosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
    }
    averagePosition /= 4.0;

    currentWheelRadius = (accumYaw * SwerveConstants.flModuleOffset.getNorm()) / averagePosition;

    SmartDashboard.putNumber("Measured Wheel Radius", Units.metersToInches(currentWheelRadius));
    SmartDashboard.putNumber(
        "Measured Wheel Diameter", Units.metersToInches(currentWheelRadius) * 2);

        double averagePositionMeters = averagePosition*SwerveConstants.wheelRadius;

        SmartDashboard.putNumber(
          "Measured Distance Meters", averagePositionMeters);
          SmartDashboard.putNumber(
            "Measured Theorwtical Distance Meters", SwerveConstants.trackWidth.baseUnitMagnitude()*Math.PI);
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angularRateLimiter.reset(0.0);
    this.swerve.drive(0, 0.0, 0, false, false, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
