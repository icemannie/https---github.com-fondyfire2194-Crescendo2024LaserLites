// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CameraConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.Factories.CommandFactory;
import frc.robot.commands.Drive.CheckOKSwitchToDrive;
import frc.robot.commands.Drive.DriveToPickupNote;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/** Add your docs here. */
public class PickupUsingVision extends SequentialCommandGroup {

        public PickupUsingVision(
                        CommandFactory cf,
                        PathPlannerPath path,
                        TransferSubsystem transfer,
                        IntakeSubsystem intake,
                        SwerveSubsystem swerve,
                        double switchoverdistance,
                        int pipelineIndex) {

                addCommands(
                                Commands.runOnce(() -> LimelightHelpers.setPipelineIndex(
                                                CameraConstants.rearCamera.camname, pipelineIndex)),

                                Commands.race(
                                                new CheckOKSwitchToDrive(swerve, cf, switchoverdistance),

                                                new RunPPath(swerve, path)),

                                Commands.either(
                                                new DriveToPickupNote(swerve, transfer,
                                                                intake),
                                                Commands.none(),
                                                () -> swerve.noteSeen));

        }

}
