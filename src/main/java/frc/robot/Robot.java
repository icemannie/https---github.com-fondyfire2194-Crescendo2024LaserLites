// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.LLPipelines;
import frc.robot.utils.LimelightHelpers;
import monologue.Annotations.Log;
import monologue.Logged;
import monologue.Monologue;

public class Robot extends TimedRobot implements Logged {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private double m_disableStartTime;

  private double brakeOffTime = 3;

  private double m_startDelay;

  @Log.NT.Once
  private double startTime;

  private boolean autoHasRun;

  int tstctr;

  private int canivorectr;

  private double startupTimeSeconds;

  @Override
  public void robotInit() {
    startTime = Timer.getFPGATimestamp();

    if (RobotBase.isReal()) {
      DriverStation.startDataLog(DataLogManager.getLog());

      // Map<Integer, String> motorNameMap = new HashMap<>();

      // motorNameMap.put(SwerveConstants.Mod0.driveMotorID, "Front Left Drive");
      // motorNameMap.put(SwerveConstants.Mod0.angleMotorID, "Front Left Turn");

      // motorNameMap.put(SwerveConstants.Mod1.driveMotorID, "Front Right Drive");
      // motorNameMap.put(SwerveConstants.Mod1.angleMotorID, "Front Right Turn");

      // motorNameMap.put(SwerveConstants.Mod2.driveMotorID, "Back Left Drive");
      // motorNameMap.put(SwerveConstants.Mod2.angleMotorID, "Back Left Turn");

      // motorNameMap.put(SwerveConstants.Mod3.driveMotorID, "Back Right Drive");
      // motorNameMap.put(SwerveConstants.Mod3.angleMotorID, "Back Right Turn");

      // motorNameMap.put(CANIDConstants.armID, "Arm");

      // motorNameMap.put(CANIDConstants.transferID, "Transfer");

      // motorNameMap.put(CANIDConstants.topShooterID, "Shooter Top");
      // motorNameMap.put(CANIDConstants.bottomShooterID, "Shooter Bottom");

      // motorNameMap.put(CANIDConstants.intakeID, "Intake");

      // motorNameMap.put(CANIDConstants.climberIDLeft, "Climber Left");
      // motorNameMap.put(CANIDConstants.climberIDRight, "Climber Right");

      // URCL.start(motorNameMap);
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);

    }

    m_robotContainer = new RobotContainer();
    if (RobotBase.isSimulation()) {
      SmartDashboard.putBoolean("Skip1", m_robotContainer.m_transfer.skipFirstNoteInSim);
      SmartDashboard.putBoolean("Skip2", m_robotContainer.m_transfer.skipSecondNoteInSim);
      SmartDashboard.putBoolean("Skip3", m_robotContainer.m_transfer.skipThirdNoteInSim);

    }
    Monologue.setupMonologue(m_robotContainer, "/Monologue", false, true);

    DriverStation.startDataLog(DataLogManager.getLog());
    // Monologue.setupMonologue(this, "/Monologue", false, true);

    // FollowPathCommand.warmupCommand().schedule();

    // System.gc();

    // PathfindingCommand.warmupCommand().schedule();

    m_robotContainer.m_pf.sourceFilesOK = m_robotContainer.m_pf.checkSourceFilesExist();

    startupTimeSeconds = Timer.getFPGATimestamp() - startTime;
    DataLogManager.log("Startup Time (ms): " + startupTimeSeconds * 1000.0);

  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();

    m_robotContainer.m_arm.periodicRobot();

    // setFileOnly is used to shut off NetworkTables broadcasting for most logging
    // calls.
    // Basing this condition on the connected state of the FMS is a suggestion only.
    // Monologue.setFileOnly(DriverStation.isDSAttached());
    // This method needs to be called periodically, or no logging annotations will
    // process properly.
    Monologue.updateAll();

    if (RobotBase.isReal()) {

      canivorectr++;
      SmartDashboard.putNumber("CVCTS", canivorectr);
      if (canivorectr >= 100) {
        m_robotContainer.logCanivore();
        canivorectr = 0;
      }
      m_robotContainer.busUtil = m_robotContainer.canInfo.BusUtilization;
    }

    SmartDashboard.putBoolean("FieldRelative", m_robotContainer.fieldRelative.getAsBoolean());
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    autoHasRun = false;
    m_robotContainer.m_swerve.drive(0, 0, 0, false, true, false);
    m_robotContainer.m_arm.disable();
    m_robotContainer.m_arm.enableArm = false;
    if (m_robotContainer.m_arm.getCanCoderDeg() < 26)
      m_robotContainer.m_arm.armMotor.setIdleMode(IdleMode.kCoast);

    m_robotContainer.m_swerve.flUpdate.setLLRobotorientation();
    m_robotContainer.m_swerve.frUpdate.setLLRobotorientation();
    m_robotContainer.m_swerve.flUpdate.setUseMegatag2(false);
    m_robotContainer.m_swerve.frUpdate.setUseMegatag2(false);

  }

  @Override
  public void disabledPeriodic() {

    // turn off drive brakes if they are on and robot is not moving
    // allows easier manual pushing of robot

    if (m_robotContainer.m_swerve.driveIsBraked() && m_robotContainer.m_swerve.isStopped()
        && m_disableStartTime == 0)
      m_disableStartTime = Timer.getFPGATimestamp();

    if (m_disableStartTime != 0 && Timer.getFPGATimestamp() > m_disableStartTime
        + brakeOffTime) {
      m_robotContainer.m_swerve.setIdleMode(false);
    }
    if (Timer.getFPGATimestamp() > startupTimeSeconds + 5)
      m_robotContainer.checkAutoSelectLoop.poll();

  }

  @Override
  public void disabledExit() {

  }

  @Override
  public void autonomousInit() {
    m_robotContainer.m_swerve.inhibitVision = false;
    // m_robotContainer.m_swerve.actualstartPose =
    // m_robotContainer.m_swerve.getPose();
    m_robotContainer.m_swerve.absoluteResetFrontModuleEncoders();
    m_robotContainer.m_swerve.absoluteResetBackModuleEncoders();

    m_robotContainer.m_swerve.flUpdate.setUseMegatag2(true);
    m_robotContainer.m_swerve.frUpdate.setUseMegatag2(true);
    m_robotContainer.m_arm.armMotor.setIdleMode(IdleMode.kBrake);
    if (RobotBase.isSimulation()) {
      m_robotContainer.m_intake.isIntaking1 = false;
      m_robotContainer.m_intake.isIntaking2 = false;
      m_robotContainer.m_intake.isIntaking3 = false;

    }

    m_robotContainer.m_arm.enable();
    m_robotContainer.m_arm.enableArm = true;

    LimelightHelpers.setPipelineIndex(CameraConstants.frontLeftCamera.camname,
        LLPipelines.pipelines.APRILTAGALL0.ordinal());
    LimelightHelpers.setPipelineIndex(CameraConstants.frontRightCamera.camname,
        LLPipelines.pipelines.APRILTAGALL0.ordinal());
    LimelightHelpers.setPipelineIndex(CameraConstants.rearCamera.camname,
        LLPipelines.pipelines.NOTEDET1.ordinal());

    m_robotContainer.m_swerve.setIdleMode(true);

    m_startDelay = m_robotContainer.m_startDelayChooser.getSelected();

    startTime = Timer.getFPGATimestamp();

    if (RobotBase.isSimulation()) {
      m_robotContainer.m_transfer.simnoteatintake = true;// robot has initial note
      m_robotContainer.m_transfer.skipFirstNoteInSim = SmartDashboard.getBoolean("Skip1", false);
      m_robotContainer.m_transfer.skipSecondNoteInSim = SmartDashboard.getBoolean("Skip2", false);
    }
    if (m_robotContainer.m_af.validChoice) {

      m_autonomousCommand = m_robotContainer.m_af.getAutonomousCommand();

      SmartDashboard.putString("Auto//AUTOCMS", m_autonomousCommand.toString());
      SmartDashboard.putBoolean("Auto//AutoHasRun", autoHasRun);
      if (!autoHasRun && Timer.getFPGATimestamp() > startTime + m_startDelay
          && m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
        autoHasRun = true;
      }
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    m_robotContainer.m_shooter.stopMotors();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    autoHasRun = false;

    m_robotContainer.m_shooter.stopMotors();
    m_robotContainer.m_intake.stopMotor();
    m_robotContainer.m_transfer.stopMotor();

  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.m_swerve.absoluteResetFrontModuleEncoders();
    m_robotContainer.m_swerve.absoluteResetBackModuleEncoders();

    m_robotContainer.m_swerve.actualstartPose = m_robotContainer.m_swerve.getPose();

    // if (RobotBase.isSimulation() || !autoHasRun) {
    // m_robotContainer.m_swerve.resetPoseEstimator(new Pose2d(8, 1, new
    // Rotation2d(0)));
    // m_robotContainer.m_transfer.simnoteatintake = false;
    // }
    m_robotContainer.m_arm.armMotor.setIdleMode(IdleMode.kBrake);

    m_robotContainer.m_swerve.setIdleMode(true);
    m_robotContainer.m_arm.enable();
    m_robotContainer.m_arm.enableArm = true;
    m_robotContainer.m_arm.setGoal(m_robotContainer.m_arm.getAngleRadians());

    m_robotContainer.m_shooter.stopMotors();
    m_robotContainer.m_intake.stopMotor();
    m_robotContainer.m_transfer.stopMotor();

    LimelightHelpers.setPipelineIndex(CameraConstants.frontLeftCamera.camname,
        LLPipelines.pipelines.APRILTAGALL0.ordinal());
    LimelightHelpers.setPipelineIndex(CameraConstants.frontRightCamera.camname,
        LLPipelines.pipelines.APRILTAGALL0.ordinal());
    LimelightHelpers.setPipelineIndex(CameraConstants.rearCamera.camname,
        LLPipelines.pipelines.NOTEDET1.ordinal());
    m_robotContainer.m_swerve.flUpdate.setUseMegatag2(true);
    m_robotContainer.m_swerve.frUpdate.setUseMegatag2(true);
    m_robotContainer.m_swerve.inhibitVision = false;
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.getPPAutCommand().schedule();
    m_robotContainer.m_swerve.absoluteResetFrontModuleEncoders();
    m_robotContainer.m_swerve.absoluteResetBackModuleEncoders();

  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
    double a1 = AllianceUtil.flipFieldAngle(FieldConstants.centerNotesPickup[1]).getRotation().getDegrees();
    double a2 = AllianceUtil.flipFieldAngle(FieldConstants.centerNotesPickup[2]).getRotation().getDegrees();

    SmartDashboard.putNumber("A1", a1);
    SmartDashboard.putNumber("A2", a2);
  }
}
