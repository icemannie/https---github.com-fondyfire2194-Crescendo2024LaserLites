// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Factories.CommandFactory;
import frc.robot.commands.Drive.AutoAlignSpeaker;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class CenterToShoot extends SequentialCommandGroup {

        public CenterToShoot(
                        CommandFactory cf,
                        PathPlannerPath path,
                        SwerveSubsystem swerve) {

                addCommands(
                                Commands.sequence(
                                                Commands.parallel(
                                                                new RunPPath(swerve, path),
                                                                cf.positionArmRunShooterSpecialCase(25, 3100, 15)),
                                                Commands.parallel(
                                                                cf.positionArmRunShooterByDistance(false, true),
                                                                new AutoAlignSpeaker(swerve, 1.5, true)),
                                                cf.transferNoteToShooterCommand()));
        }
}
