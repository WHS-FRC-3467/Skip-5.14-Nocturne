// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;


public class FieldCentricAiming {
    private Pose2d _speakerPosition;
    
    public FieldCentricAiming() {
        getSpeakerPos();
    }

    public Pose2d getSpeakerPos() {
        if (_speakerPosition == null) {
            if (DriverStation.getAlliance() != null && !DriverStation.getAlliance().isEmpty()) {
                _speakerPosition = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? Constants.BLUE_SPEAKER : Constants.RED_SPEAKER;
            }
        }
        return _speakerPosition;
    }

    public double getDistToSpeaker(Translation2d robotPose) {
        return getSpeakerPos().getTranslation().getDistance(robotPose);
    }

    public Rotation2d getAngleToSpeaker(Translation2d pose) {
        return getSpeakerPos().getTranslation().minus(pose).getAngle();
    }

}
