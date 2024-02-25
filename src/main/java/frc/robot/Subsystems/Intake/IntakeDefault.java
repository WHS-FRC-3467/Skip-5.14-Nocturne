// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.StageConstants;
import frc.robot.Subsystems.Stage.StageSubsystem;

public class IntakeDefault extends Command {

  DoubleSupplier m_fwd, m_rev;
  IntakeSubsystem m_intake;
  StageSubsystem m_stage;

  /** Creates a new IntakeDefault. */
  public IntakeDefault(IntakeSubsystem intake, StageSubsystem stage, DoubleSupplier fwd, DoubleSupplier rev){
    m_intake = intake;
    m_stage = stage;
    m_fwd = fwd;
    m_rev = rev;
//    addRequirements(m_intake, m_stage);
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed;
 
    // Run both Intake and Stage, with Stage running slightly slower both ways
    if ((speed = m_fwd.getAsDouble()) > 0.1) {
        m_intake.runIntake(speed);
 //       m_stage.runStage(speed * StageConstants.kIntakeSpeed);

    } else if ((speed = m_rev.getAsDouble()) > 0.1) {
        m_intake.runIntake(-speed);
 //       m_stage.runStage(-speed * StageConstants.kIntakeSpeed);

    } else {
        m_intake.stopIntake();
  //      m_stage.stopStage();

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntake();
    m_stage.stopStage();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

