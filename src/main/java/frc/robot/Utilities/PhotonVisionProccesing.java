// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class PhotonVisionProccesing {

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    PhotonCamera camera;

    Transform3d kRobotToCam;

    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);


    public PhotonVisionProccesing(String camName, Transform3d camToCenter) {
        camera = new PhotonCamera(camName);
        kRobotToCam = camToCenter;
    }


    public boolean foundTag() {

        var result = camera.getLatestResult();

        return result.hasTargets();
        
    }

    public Pose2d GetBestPose(Pose2d currentPose) {

        var result = camera.getLatestResult();
        
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
        

            if ( aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent() ) {
                Pose3d newPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), kRobotToCam);
                
                SmartDashboard.putBoolean("FoundTag", true);
                
                return newPose.toPose2d();
            }
        }

        SmartDashboard.putBoolean("FoundTag", false);

        return currentPose;

    }
}
