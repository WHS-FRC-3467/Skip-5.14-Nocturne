// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;

public class calibrateLookupTable extends Command {
    CommandSwerveDrivetrain m_drivetrain;
    driveToPose m_driveToPose;
    
    Translation2d currentTarget;
    Rotation2d currentAngle;
    ArrayList<Translation2d> aimingPositions;
    
  /** Creates a new calibrateLookupTable. */
  public calibrateLookupTable(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_driveToPose = new driveToPose(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveToPose.setTarget(currentTarget);
    m_driveToPose.setAngle(currentAngle);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
