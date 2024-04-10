// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;
//import frc.robot.Subsystems.Stage.StageSubsystem;
import frc.robot.Util.Setpoints;
//import frc.robot.Util.Setpoints.GameState;

public class subThatShot extends Command{

    Setpoints m_setpoints;
    StageSubsystem m_stageSubsystem;
    ShooterSubsystem m_shooterSubsystem;
    BooleanSupplier m_haveNote;
    boolean m_runShooter;
    boolean m_isDone = false;

    /** Constructor - Creates a new prepareShooterOnly. */
    public subThatShot(ShooterSubsystem shootSub, StageSubsystem stageSub) {
        m_setpoints = Constants.RobotConstants.SUBWOOFER;
        m_shooterSubsystem = shootSub;
        m_stageSubsystem = stageSub;
        addRequirements(shootSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_isDone = false;

        // If Shooter setpoints are zero, don't bother to check if it is up to speed
       
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
            m_stageSubsystem.feedWithBeam().schedule();
            System.out.println("SUB SHOT READY");
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
