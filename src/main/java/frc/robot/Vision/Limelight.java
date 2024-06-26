// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
//import static frc.robot.Constants.LimelightConstants.*;

public class Limelight extends SubsystemBase {

    private Boolean enable;
    private Boolean hasTarget = false;
    public String m_llName;
    double tx;

    /** Creates a new Limelight. */
    // constructor goes here
    public Limelight(String limelight) {
        m_llName = limelight;

        this.useLimelight(RobotConstants.k_shouldUseLimelight);

        LimelightHelpers.setLEDMode_PipelineControl(m_llName);
        LimelightHelpers.setLEDMode_ForceBlink(m_llName);
        LimelightHelpers.setCropWindow(m_llName, -1, 1, -1, 1);
        tx = LimelightHelpers.getTX(m_llName);
    }

    @Override
    public void periodic() {

        // Smart Dashboard stuff goes here

        // Basic Targeting Data
        //NetworkTableInstance.getDefault().getTable(m_llName).getEntry("<variablename>").getDouble(0);
        // variable name can be tv, tx, ty, txnc  ... etc
    }

    public void useLimelight(boolean enable) {
        this.enable = enable;
    }

    public boolean hasTarget() {
        // 1 if valid target exists. 0 if no valid targets exist
        double tv = NetworkTableInstance.getDefault().getTable(m_llName).getEntry("tv").getDouble(0);
        if (tv == 1) {
            hasTarget = true;
        } else {
            hasTarget = false;
        }
        return hasTarget;
    }

    public LimelightHelpers.LimelightResults getLatestLLResults() {
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults(m_llName);
        return llresults;
    }

}
