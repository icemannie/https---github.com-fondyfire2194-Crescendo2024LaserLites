// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pathplanner;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunPPath extends SequentialCommandGroup {
  /** Creates a new RunLoadedPPPath. */
  private final SwerveSubsystem m_swerve;
  private final PathPlannerPath m_path;

  public RunPPath(SwerveSubsystem swerve, PathPlannerPath path) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_swerve = swerve;
    m_path = path;
    addRequirements(m_swerve);
    m_swerve.setPathStarted();
    addCommands(
        AutoBuilder.followPath(m_path));
  }
}
