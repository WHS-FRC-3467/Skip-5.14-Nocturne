// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Util.RectanglePoseArea;
import static frc.robot.Constants.LimelightConstants.*;

public class Limelight extends SubsystemBase {
    CommandSwerveDrivetrain m_drivetrain;
    Alliance alliance;
    private String ll = kCameraName;
    private Boolean enable = false;
    private double tx;
    private double ty;
    private Boolean hasTarget = false;

    /** Creates a new Limelight. */
    public Limelight(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;

    }

    @Override
    public void periodic() {
        if (enable) {
            if (LimelightHelpers.getTV(kCameraName)) {
                LimelightHelpers.Results result = LimelightHelpers.getLatestResults(ll).targetingResults;
                if (result.valid && LimelightHelpers.getTA(kCameraName) > 0) {
                    hasTarget = true;
                    //m_drivetrain.setNoteAngle(new Rotation2d(tx));
                } else {
                    hasTarget = false;
                    //m_drivetrain.setNoteAngle(null);
                }
            }
            if (RobotConstants.kIsTuningMode) {
                SmartDashboard.putBoolean("Limelight has note detected", hasTarget);
            }

        }
    }

    public void useLimelight(boolean enable) {
        this.enable = enable;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

}
