// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisaoConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class VisionSubsystem extends SubsystemBase {

  static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public static PhotonCamera Limelight = new PhotonCamera("Limelight");
  private PhotonPoseEstimator LimelightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.LOWEST_AMBIGUITY, VisaoConstants.LimelightToCam); // TESTAR MULTI_TAG PNP

  public static PhotonCamera Arducam = new PhotonCamera("Arducam");
  private PhotonPoseEstimator ArducamPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.LOWEST_AMBIGUITY, VisaoConstants.ArducamToCam); // TESTAR MULTI_TAG PNP

  public VisionSubsystem() {
    Limelight.setLED(VisionLEDMode.kOn);

  }

  public Optional<EstimatedRobotPose> getEstimatedPoseLime() {
    return LimelightPoseEstimator.update(Limelight.getLatestResult());
  }

  public Optional<EstimatedRobotPose> getEstimatedPoseArducam() {
    return ArducamPoseEstimator.update(Arducam.getLatestResult());
  }

  public PhotonPipelineResult LimeGetLatestResult() {
    return Limelight.getLatestResult();
  }

  public double DistanceToTarget(Pose2d robotPose, Pose2d targetPose) {
    return PhotonUtils.getDistanceToPose(robotPose, targetPose);

  }

  public void periodic() {

  }

  public boolean TagIntakeCoral() {
    var latestResult = ArducamGetLatestResult();

    if (!latestResult.hasTargets()) {
        return false;
    }

    var bestTarget = latestResult.getBestTarget();
    if (bestTarget == null) { // Prevents NullPointerException
        return false;
    }

    int fiducialId = bestTarget.getFiducialId();
    double distance = bestTarget.getBestCameraToTarget().getX();

    return (fiducialId == 13 || fiducialId == 12) && distance < 1.7;
}


  public PhotonPipelineResult ArducamGetLatestResult() {
    return Arducam.getLatestResult();
  }

  public Rotation2d IdTag() {

    if (LimeGetLatestResult().hasTargets()) {
      var bestTarget = LimeGetLatestResult().getBestTarget();
      if (LimeGetLatestResult().getBestTarget() != null) {
        return aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).get().getRotation().toRotation2d();

      }
    }
    return new Rotation2d(0); // Default rotation if no target is found

  }

  public Rotation3d getTagRotation(PhotonTrackedTarget target) {

    if (LimeGetLatestResult().hasTargets()) {
      var bestTarget = LimeGetLatestResult().getBestTarget();
      if (LimeGetLatestResult().getBestTarget() != null) {
        Transform3d transform = target.getBestCameraToTarget();
        // Retorna a rotação (Yaw, Pitch, Roll)

        return transform.getRotation();

      }

    }
    return new Rotation3d();

  }

}
