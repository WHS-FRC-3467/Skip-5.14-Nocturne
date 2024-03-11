// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PreCheck;

import edu.wpi.first.wpilibj2.command.Command;

public class preCheck extends Command {
    /////////////////////////////////////////////////////////////
    // Command: PreCheck
    // Description: We will run through each subsystem and record
    //              all of our current draw from each mechanism
    //              and store them into a .txt file for analysis
    /////////////////////////////////////////////////////////////
    // Drivebase = m_drivebase
    // Intake = m_intake
    // Shooter = m_shooter
    // Stage = m_stage
    // Arm = m_arm
    // Boolean Pass = false
  public preCheck() { // Drivebase, Intake, Shooter, Stage, Arm
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
