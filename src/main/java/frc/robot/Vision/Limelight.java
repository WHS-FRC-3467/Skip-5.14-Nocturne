// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
//import static frc.robot.Constants.LimelightConstants.*;

public class Limelight extends SubsystemBase {

    private Boolean enable = false;
    private Boolean hasTarget = false;

    /** Creates a new Limelight. */
    // constructor goes here

    @Override
    public void periodic() {

    }

    public void useLimelight(boolean enable) {
        this.enable = enable;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

}
