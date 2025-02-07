// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisaoConstants;

/** Add your docs here. */
public class VisionSubsystem extends SubsystemBase{
    
    static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    
    static PhotonCamera Limelight = new PhotonCamera("Limelight");
    private PhotonPoseEstimator LimelightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, VisaoConstants.LimelightToCam); //TESTAR MULTI_TAG PNP
    
    static PhotonCamera Arducam = new PhotonCamera("Arducam");
    private PhotonPoseEstimator ArducamPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, VisaoConstants.ArducamToCam); //TESTAR MULTI_TAG PNP

    public VisionSubsystem(){
      // LimelightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      // ArducamPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<EstimatedRobotPose> getEstimatedPoseLime() {
      //LimelightPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return LimelightPoseEstimator.update(Limelight.getLatestResult());
    }

    public Optional<EstimatedRobotPose> getEstimatedPoseArducam( ) {
      //ArducamPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return ArducamPoseEstimator.update(Arducam.getLatestResult());
    }


    public PhotonPipelineResult LimeGetLatestResult() {
      return Limelight.getLatestResult();
    }

    public void periodic(){
    }

    public PhotonPipelineResult ArducamGetLatestResult() {
      return Arducam.getLatestResult();
    }
    
}
