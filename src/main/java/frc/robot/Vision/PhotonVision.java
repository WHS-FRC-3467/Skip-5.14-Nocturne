/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.PhotonVisionConstants.front_left_cam;
//import frc.robot.Constants.PhotonVisionConstants.top_right_cam;
import frc.robot.Robot;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class PhotonVision extends SubsystemBase {

    PhotonCamera camera;
    Boolean hasTargets;
    String cam_name;

    public PhotonVision(CommandSwerveDrivetrain drivetrain, int cam_num) {

        if (cam_num == 1) {
            cam_name = "front_left_cam";
        } else if (cam_num == 0) {
            cam_name = "top_right_cam";
        }

        camera = new PhotonCamera(cam_name);

    }

    @Override
    public void periodic() {

        /* Get the latest pipeline result.
        it returns a container with all information about currently detected targets from a PhotonCamera
        and is guaranteed to be from the same timestamp.
        */
        var result = camera.getLatestResult();
        // MUST ALWAYS check if the result has a target before getting targets or else you may get a null pointer exception.
        // ALso must use the same result in every subsequent call in that loop
        hasTargets = result.hasTargets();
        // If camera has target, then get a list of tracked targets from a pipeline result.
        // Contains info such as yaw, pitch, area, and robot relative pose
        if (hasTargets) {
            // Get a list of currently tracked targets.
            List<PhotonTrackedTarget> targets = result.getTargets();
            
            // Get the current best target.
            PhotonTrackedTarget target = result.getBestTarget();

            // Get information from target.
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            double area = target.getArea();
            double skew = target.getSkew();
            //Transform2d pose = target.getCameraToTarget();
            //List<TargetCorner> corners = target.getCorners();

        }
    }

}