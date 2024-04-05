// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Util.Setpoints;
import frc.robot.Util.ShooterPreset;
import frc.robot.Util.VisionLookUpTable;

public class AutoLookUpShot extends Command {

    Setpoints m_setpoints;
    CommandSwerveDrivetrain m_drivetrain;
    ArmSubsystem m_armSubsystem;
    ShooterSubsystem m_shooterSubsystem;
    ShooterPreset m_shotInfo;
    VisionLookUpTable m_VisionLookUpTable;
    DoubleSupplier m_distance;
    boolean m_isDone;
    double m_maxShotDist;
    boolean m_shootingOnTheMove;
    

    /** Constructor - Creates a new prepareToShoot. */
    public AutoLookUpShot(CommandSwerveDrivetrain drivetrain, ArmSubsystem armSub, ShooterSubsystem shootSub, 
                            DoubleSupplier distance, double maxShotDist, boolean shootingOnTheMove) {
        m_drivetrain = drivetrain;
        m_armSubsystem = armSub;
        m_shooterSubsystem = shootSub;
        //m_VisionLookUpTable = new VisionLookUpTable();
        m_distance = distance;
        m_maxShotDist = maxShotDist;
        m_setpoints = RobotConstants.LOOKUP;
        m_shootingOnTheMove = shootingOnTheMove;

        addRequirements(armSub, shootSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!m_armSubsystem.isEnabled()) {m_armSubsystem.enable();};
        //System.out.println("SCHEDULED");
        m_isDone = false;
        m_VisionLookUpTable = new VisionLookUpTable();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double distance = m_distance.getAsDouble();
        m_shotInfo = m_VisionLookUpTable.getShooterPreset(distance);
        
        m_setpoints.arm = m_shotInfo.getArmAngle();
        m_setpoints.shooterLeft = m_shotInfo.getLeftShooter();
        m_setpoints.shooterRight = m_shotInfo.getRightShooter();

        if (RobotConstants.kIsArmTuningMode) {
            SmartDashboard.putNumber("LookUp Distance", distance);
            SmartDashboard.putNumber("Shot Info Angle", m_setpoints.arm);
        }

        m_armSubsystem.updateArmSetpoint(m_setpoints);

        // Bring Shooter to requested speed
        m_shooterSubsystem.runShooter(m_setpoints.shooterLeft, m_setpoints.shooterRight);

        if(m_armSubsystem.isArmJointAtSetpoint() && m_shooterSubsystem.areWheelsAtSpeed() && m_drivetrain.isAtFutureAngle() && !m_drivetrain.isRotatingFast()) {
            if((!m_shootingOnTheMove) || (m_shootingOnTheMove && distance < m_maxShotDist)) {
                m_isDone = true;
            }
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop the Shooter but keep the Arm control going
            // Actually lets not stop the shooter right now
        //m_shooterSubsystem.stopShooter();

    }

    // Command never ends on its own - it has to be interrupted.
    @Override
    public boolean isFinished() {
        return m_isDone;
    }
}
