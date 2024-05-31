// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.Subsystems.Intake.IntakeSubsystem;
// figure out how to use java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.StageConstants;
import frc.robot.Subsystems.Stage.StageSubsystem;

public class intakeNote extends Command {

    private final IntakeSubsystem m_intakeSub;
    private final StageSubsystem m_stageSub;
    private boolean m_isDone;

    /** Creates a new intakeNote. */
    public intakeNote(IntakeSubsystem intakeSubsystem, StageSubsystem stageSubsystem) {

        m_intakeSub = intakeSubsystem;
        m_stageSub = stageSubsystem;
        addRequirements(m_intakeSub, m_stageSub);
        m_isDone = false;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // run intake and stage. Then set m_is to true if the stage subsystem's
        // beambreak is equal to true
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intakeSub.stopIntake();
        // then stop stage
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_isDone == true) {
            return true;
        }
        return false;
    }
}
