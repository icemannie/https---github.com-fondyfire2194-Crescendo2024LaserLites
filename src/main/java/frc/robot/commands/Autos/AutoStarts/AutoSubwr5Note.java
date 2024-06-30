// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.AutoStarts;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Factories.AutoFactory;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sbwfrpaths;
import frc.robot.Factories.PathFactory.sourcepaths;
import frc.robot.commands.Autos.Autos.CenterToShoot;
import frc.robot.commands.Autos.Autos.LookForAnotherNote;
import frc.robot.commands.Autos.Autos.PickupUsingVision;
import frc.robot.commands.Autos.Autos.SourceAutoCommands;
import frc.robot.commands.Autos.SubwfrStart.SubwooferAutoCommands;
import frc.robot.commands.Drive.AutoAlignSpeaker;
import frc.robot.commands.Drive.RotateToAngle;
import frc.robot.commands.Pathplanner.RunPPath;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.LLPipelines;

/** Add your docs here. */
public class AutoSubwr5Note extends SequentialCommandGroup {

        public AutoSubwr5Note(
                        CommandFactory cf,
                        PathFactory pf,
                        AutoFactory af,
                        SubwooferAutoCommands sac,
                        SwerveSubsystem swerve,
                        IntakeSubsystem intake,
                        TransferSubsystem transfer,
                        LimelightVision llv,
                        double switchoverdistance,
                        ArmSubsystem arm) {

                addCommands( // note
                                sac.setsbwrstart(swerve, cf),
                                Commands.race(
                                                Commands.waitSeconds(.75),
                                                cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                                Constants.subwfrShooterSpeed, 20)),
                                cf.transferNoteToShooterCommand(),

                                runPathPickupAndShoot(pf.pathMaps.get(sbwfrpaths.SubToNote3Fast.name()), swerve, arm,
                                                cf, pf),

                                runPathPickupAndShoot(pf.pathMaps.get(sbwfrpaths.Note3ToNote2Fast.name()), swerve, arm,
                                                cf, pf),

                                runPathPickupAndShoot(pf.pathMaps.get(sbwfrpaths.Note2ToNote1Fast.name()), swerve, arm,
                                                cf, pf),

                                Commands.parallel(

                                                new PickupUsingVision(cf,
                                                                pf.pathMaps.get(sbwfrpaths.Note1ToCenter1Fast
                                                                                .name()),
                                                                transfer, intake, swerve, 2.0,
                                                                LLPipelines.pipelines.NOTEDET1.ordinal()),

                                                cf.doIntake(5)),

                                //Commands.either(
                                new CenterToShoot(cf,
                                                pf.pathMaps.get(sbwfrpaths.Center1ToShootFast
                                                                .name()),
                                                swerve)
                                //getAnotherNote(swerve, transfer, intake, cf, pf),
                               // () -> false)//transfer.noteAtIntake())

                                 // This would pickup from center

                // if note in intake go shoot it or try to find one

                // Commands.either(
                // srcac.moveShootCenter4_5(cf, pf, swerve, !innerNoteFirst),
                // getAnotherNote(swerve, transfer, intake, cf, pf),
                // () -> transfer.noteAtIntake()));
                );
        }

        Command runPathPickupAndShoot(PathPlannerPath path, SwerveSubsystem swerve, ArmSubsystem arm, CommandFactory cf,
                        PathFactory pf) {
                return Commands.sequence(

                                Commands.parallel(
                                                new RunPPath(swerve, path), 
                                                cf.doIntake(5)),

                                cf.positionArmRunShooterByDistance(false, true), // might need delay
                                cf.transferNoteToShooterCommand());

        }

        Command getAnotherNote(SwerveSubsystem swerve, TransferSubsystem transfer, IntakeSubsystem intake,
                        CommandFactory cf, PathFactory pf) {

                return Commands.sequence(
                                Commands.runOnce(() -> transfer.simnoteatintake = false),
                                new RotateToAngle(swerve, 90), // -90
                                Commands.deadline(
                                                new LookForAnotherNote(swerve, transfer, intake),
                                                cf.doIntake(10)),
                                Commands.waitSeconds(.25),
                                Commands.either(
                                                Commands.sequence(
                                                                cf.autopathfind(AllianceUtil
                                                                                .getSourceClearStagePose(),
                                                                                SwerveConstants.pfConstraints, 0, 0),
                                                                Commands.waitSeconds(.25),
                                                                cf.autopathfind(AllianceUtil
                                                                                .getSourceShootPose(),
                                                                                SwerveConstants.pfConstraints, 0, 0),
                                                                Commands.parallel(
                                                                                cf.positionArmRunShooterByDistance(
                                                                                                false, true),
                                                                                new AutoAlignSpeaker(swerve, 1, true)),
                                                                cf.transferNoteToShooterCommand(),
                                                                new RunPPath(swerve, pf.pathMaps
                                                                                .get(sourcepaths.SourceShootToCenter4
                                                                                                .name())),
                                                                Commands.runOnce(() -> this.cancel())),
                                                Commands.runOnce(() -> this.cancel()),
                                                () -> transfer.noteAtIntake()));
        }

        // public Command tryOtherNote(PathFactory pf, CommandFactory cf,
        // SwerveSubsystem swerve,
        // TransferSubsystem transfer, boolean innerNoteFirst) {
        // return Commands.sequence(
        // Commands.parallel(
        // Commands.either(
        // new RunPPath(swerve,
        // pf.pathMaps.get(sourcepaths.Center5ToCenter4
        // .name())),
        // new RunPPath(swerve,
        // pf.pathMaps.get(sourcepaths.Center4ToCenter5
        // .name())),
        // () -> innerNoteFirst),
        // cf.doIntake(2)));
        // }
}