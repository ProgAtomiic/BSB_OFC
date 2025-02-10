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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisaoConstants;

/** Add your docs here. */
public class VisionSubsystem extends SubsystemBase{
    
    static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    
    public static PhotonCamera Limelight = new PhotonCamera("Limelight");
    private PhotonPoseEstimator LimelightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, VisaoConstants.LimelightToCam); //TESTAR MULTI_TAG PNP
    
    public static PhotonCamera Arducam = new PhotonCamera("Arducam");
    private PhotonPoseEstimator ArducamPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, VisaoConstants.ArducamToCam); //TESTAR MULTI_TAG PNP
    
    // how many degrees back is your limelight rotated from perfectly vertical?
    double AnguloMontagemCamera = 25.0; 
    // distance from the center of the Limelight lens to the floor
    double AlturaLenteCamera = 20.0; 
    // distance from the target to the floor
    double AlturaAlvo = 60.0; 

    double distanceFromLimelightToGoalInches;
    public VisionSubsystem(){
      // LimelightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      // ArducamPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<EstimatedRobotPose> getEstimatedPoseLime() {
    return LimelightPoseEstimator.update(Limelight.getLatestResult());
    }

    public Optional<EstimatedRobotPose> getEstimatedPoseArducam( ) {
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

    public double DistanciaDaTag(PhotonCamera Camera){
    //calculate distance
      return  distanceFromLimelightToGoalInches = (AlturaAlvo - AlturaLenteCamera) / Math.tan(Units.degreesToRadians((AnguloMontagemCamera + Camera.getLatestResult().getBestTarget().pitch)));

    }
    
}
