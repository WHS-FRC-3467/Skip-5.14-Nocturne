// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import frc.robot.Subsystems.Intake.IntakeSubsystem;
// figure out how to use java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.StageConstants;
import frc.robot.Subsystems.Stage.StageSubsystem;

public class IntakeDefault extends Command {

    private final IntakeSubsystem m_intakeSub;

    /** Creates a new IntakeDefault. */
    public IntakeDefault(IntakeSubsystem intakeSubsystem) {

        m_intakeSub = intakeSubsystem;
        addRequirements(m_intakeSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intakeSub.stopIntake();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
