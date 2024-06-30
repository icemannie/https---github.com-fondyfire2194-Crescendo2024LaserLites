// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class LimelightTagsUpdate {

    private final String m_camname;
    private final SwerveSubsystem m_swerve;

    private boolean m_useMegaTag2 = false;
    boolean rejectUpdate;

    public LimelightTagsUpdate(String camname, SwerveSubsystem swerve) {
        m_camname = camname;
        m_swerve = swerve;
    }

    public void setUseMegatag2(boolean on) {
        m_useMegaTag2 = on;
    }

    public void setLLRobotorientation() {
        LimelightHelpers.SetRobotOrientation(m_camname,
                m_swerve.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees(),
                // m_swerve.getHeadingDegrees(),
                m_swerve.getGyroRate(), 0, 0, 0, 0);
    }

    public void execute() {

        if (m_useMegaTag2) {

            setLLRobotorientation();

            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_camname);

            double distanceLimelightToEstimator = mt2.pose.getTranslation().getDistance(m_swerve.getPoseEstimator().getEstimatedPosition().getTranslation());

            SmartDashboard.putNumber("OffDistance", distanceLimelightToEstimator);
            rejectUpdate = mt2.tagCount == 0 || Math.abs(m_swerve.getGyroRate()) > 720
                    || (mt2.tagCount == 1 && mt2.rawFiducials.length == 1 && mt2.rawFiducials[0].ambiguity > .7
                            && mt2.rawFiducials[0].distToCamera > 3);

            if (!rejectUpdate && !m_swerve.inhibitVision) {
                m_swerve.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(1.0, 1.0, 9999999));
                m_swerve.getPoseEstimator().addVisionMeasurement(
                        mt2.pose,
                        mt2.timestampSeconds);
            }
        }

        else {

            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_camname);

            rejectUpdate = mt1.tagCount == 0
                    || mt1.tagCount == 1 && mt1.rawFiducials.length == 1 && mt1.rawFiducials[0].ambiguity > .7
                            && mt1.rawFiducials[0].distToCamera > 3;

            if (!rejectUpdate) {
                m_swerve.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 1));
                m_swerve.getPoseEstimator().addVisionMeasurement(
                        mt1.pose,
                        mt1.timestampSeconds);
            }
        }
    }

}
