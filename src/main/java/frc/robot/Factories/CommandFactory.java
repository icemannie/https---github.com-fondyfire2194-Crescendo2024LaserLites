// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Pref;
import frc.robot.commands.Transfer.TransferIntakeToSensor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.ShootingData;

/** Add your docs here. */
public class CommandFactory {

        private final SwerveSubsystem m_swerve;

        private final IntakeSubsystem m_intake;

        private final TransferSubsystem m_transfer;

        private final ShooterSubsystem m_shooter;

        private final ArmSubsystem m_arm;

        private final ShootingData m_sd;

        Pose2d tempPose2d = new Pose2d();

        public int testNotesRun;

        public CommandFactory(SwerveSubsystem swerve, ShooterSubsystem shooter, ArmSubsystem arm,
                        IntakeSubsystem intake, TransferSubsystem transfer,
                        ShootingData sd) {
                m_swerve = swerve;
                m_shooter = shooter;
                m_arm = arm;
                m_intake = intake;
                m_transfer = transfer;
                m_sd = sd;
        }

        public Command autopathfind(Pose2d targetPose, PathConstraints pc, double goalendvelocity,
                        double rotationdelaydistance) {
                return AutoBuilder.pathfindToPose(
                                targetPose,
                                pc,
                                goalendvelocity,
                                rotationdelaydistance);
        }

        public Command autopathfind(Pose2d targetPose, double goalendvelocity, double rotationdelaydistance) {
                return AutoBuilder.pathfindToPose(
                                targetPose,
                                SwerveConstants.pfConstraints,
                                goalendvelocity,
                                rotationdelaydistance);
        }

        public Command autopathfind(Pose2d targetPose, double rotationdelaydistance) {
                return AutoBuilder.pathfindToPose(
                                targetPose,
                                SwerveConstants.pfConstraints,
                                0,
                                rotationdelaydistance);
        }

        public Command test(SwerveSubsystem swerve) {

                return new DeferredCommand(() -> {

                        if (swerve.notePoseCalculated)
                                return this.pathfindpickup();
                        else
                                return Commands.none();

                }, Set.of(swerve));

        }

        public Command pathfindpickup() {
                return Commands.parallel(
                                AutoBuilder.pathfindToPose(
                                                m_swerve.getPathfindPose(),
                                                SwerveConstants.pickUpConstraints,
                                                0,
                                                0),
                                doIntake(4));
        }

        public Command positionArmRunShooterByDistance(boolean lob, boolean endAtTargets) {
                return new FunctionalCommand(

                                () -> Commands.runOnce(() -> m_transfer.lobbing = lob),

                                () -> {
                                        if (lob) {

                                                double distance = m_swerve.getDistanceFromStage();
                                                m_shooter.startShooter(
                                                                Constants.shooterLobRPMMap.get(
                                                                                distance));
                                                m_arm.setTolerance(ArmConstants.angleTolerance);
                                                m_arm.setTarget(getLobArmAngleFromTarget(
                                                                distance));
                                        } else {

                                                m_swerve.targetdistance = m_swerve.getDistanceFromTarget(false,
                                                                false);
                                                m_shooter.startShooter(
                                                                m_sd.shooterRPMMap.get(m_swerve.targetdistance));
                                                m_arm.setTolerance(ArmConstants.angleTolerance);
                                                m_arm.setTarget(m_sd.armAngleMap.get(m_swerve.targetdistance));
                                        }
                                },

                                (interrupted) -> Commands.none(),

                                () -> endAtTargets && m_arm.getAtSetpoint() && m_shooter.bothAtSpeed(5));
        }

        public Command armFollowTargetDistance() {
                return Commands.parallel(
                                m_arm.setGoalCommand(m_sd.armAngleMap
                                                .get(m_swerve.targetPose.getTranslation().getNorm())),
                                Commands.runOnce(() -> m_arm.setTolerance(m_sd.armAngleMap.get(
                                                m_swerve.targetPose.getTranslation().getNorm()))));

        }

        public Command positionArmRunShooterSpecialCase(double armAngleDeg, double shooterSpeed, double rpmpct) {
                return Commands.parallel(
                                m_arm.setGoalCommand(Units.degreesToRadians(armAngleDeg)),
                                m_shooter.startShooterCommand(shooterSpeed, rpmpct));

        }

        public Command doIntake(double noNotetime) {
                return Commands.sequence(
                                armToIntake(),
                                m_intake.startIntakeCommand(),
                                new TransferIntakeToSensor(m_transfer, m_intake, m_swerve, noNotetime));
        }

        public Command doIntakeDelayed(double delaysecs, double noNoteTime) {
                return Commands.sequence(
                                Commands.waitSeconds(delaysecs),
                                doIntake(noNoteTime));
        }

        public Command armToIntake() {
                return m_arm.setGoalCommand(ArmConstants.pickupAngleRadians);
        }

        public Command transferNoteToShooterCommand() {
                return m_transfer.transferToShooterCommand();
        }

        public Command stopShooter() {
                return m_shooter.stopShooterCommand();
        }

        public Command setArmShooterValues(double armAngle, double shooterRPM) {
                return Commands.parallel(
                                m_arm.setGoalCommand(Units.degreesToRadians(armAngle)),
                                m_shooter.startShooterCommand(shooterRPM, 10));
        }

        public Command checkAtTargets(double pct) {
                return Commands.waitUntil(() -> m_shooter.bothAtSpeed(pct) && m_arm.getAtSetpoint());
        }

        public double getLobArmAngleFromTarget(double distance) {
                double opp = FieldConstants.stageHeight - ArmConstants.armPivotZ;
                return Math.atan(opp / distance);
        }

        public Command rumbleCommand(CommandXboxController controller) {
                return Commands.run(() -> {
                        if (m_swerve.getOnTarget())
                                controller.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
                        else
                                controller.getHID().setRumble(RumbleType.kLeftRumble, 0.0);

                        if (m_transfer.noteAtIntake())
                                controller.getHID().setRumble(RumbleType.kRightRumble, 1.0);
                        else
                                controller.getHID().setRumble(RumbleType.kRightRumble, 0.0);

                }).finallyDo(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0));
        }

        public Command setStartPosebyAlliance(Pose2d startPose) {
                tempPose2d = startPose;
                if (AllianceUtil.isRedAlliance())
                        tempPose2d = GeometryUtil.flipFieldPose(startPose);
                return Commands.runOnce(() -> m_swerve.resetPoseEstimator(tempPose2d));
        }

        public Command resetAll() {
                return Commands.parallel(
                                m_shooter.stopShooterCommand(),
                                m_intake.stopIntakeCommand(),
                                m_transfer.stopTransferCommand(),
                                m_arm.setGoalCommand(ArmConstants.pickupAngleRadians)
                                                .withName("Reset All"))
                                .asProxy();
        }

        public Command doAmpShot() {
                return Commands.sequence(
                                m_arm.setGoalCommand(Units.degreesToRadians(90)),
                                m_shooter.startShooterCommand(
                                                Pref.getPref("AmpTopRPM"), Pref.getPref("AmpBottomRPM")),
                                Commands.waitUntil(() -> m_arm.getAtSetpoint()),
                                m_arm.setGoalCommand(Units.degreesToRadians(Pref.getPref("AmpArmDegrees"))),
                                Commands.waitUntil(() -> m_arm.getAtSetpoint()),
                                Commands.parallel(
                                                m_transfer.transferToShooterCommandAmp(),
                                                Commands.sequence(
                                                                new WaitCommand(Pref.getPref("AmpArmIncrementDelay")),
                                                                m_arm.setGoalCommand(
                                                                                Units.degreesToRadians(Pref.getPref(
                                                                                                "AmpArmDegrees"))
                                                                                                + Units.degreesToRadians(
                                                                                                                Pref.getPref("AmpDegreeIncrement"))),
                                                                new WaitCommand(2))),

                                Commands.parallel(
                                                m_shooter.stopShooterCommand(),
                                                m_arm.setGoalCommand(ArmConstants.pickupAngleRadians)));

        }
}
