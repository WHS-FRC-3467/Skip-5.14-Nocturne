// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
//import frc.robot.Subsystems.Stage.StageSubsystem;
import frc.robot.Util.Setpoints;
//import frc.robot.Util.Setpoints.GameState;

public class prepareShooterOnly extends Command{

    Setpoints m_setpoints;
    //StageSubsystem m_stageSubsystem;
    ShooterSubsystem m_shooterSubsystem;
    BooleanSupplier m_haveNote;
    boolean m_runShooter;
    boolean m_isDone;

    /** Constructor - Creates a new prepareShooterOnly. */
    public prepareShooterOnly(double leftShootSpeed, double rightShootSpeed, ShooterSubsystem shootSub) {

        m_shooterSubsystem = shootSub;
        m_setpoints.shooterLeft = leftShootSpeed;
        m_setpoints.shooterRight = rightShootSpeed;

        addRequirements(shootSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // If Shooter setpoints are zero, don't bother to check if it is up to speed
        m_runShooter = (m_setpoints.shooterLeft != 0.0 || m_setpoints.shooterRight != 0.0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Set Shooter setpoints...
        m_shooterSubsystem.setShooterSetpoints(m_setpoints);
        // ... then run the Shooter
        m_shooterSubsystem.runShooter();

        // End command if shooter wheels are at speed
        if (m_shooterSubsystem.areWheelsAtSpeed()) {
            m_isDone = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //m_shooterSubsystem.stopShooter();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isDone;
    }
}
