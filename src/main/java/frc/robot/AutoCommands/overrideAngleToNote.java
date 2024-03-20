// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Vision.Limelight;
import frc.robot.Vision.LimelightHelpers;

public class overrideAngleToNote extends Command {
    Limelight m_limelight;
    CommandSwerveDrivetrain m_drivetrain;
    Rotation2d fieldRelRotation2d = new Rotation2d();
    Rotation2d angleToNote = new Rotation2d();
  /** Sets a pathplanner rotational override to point the drivetrain towards a note
   * @param drivetrain CommandSwerveDrivetrain instance
   * @param limelight Limelight instance
   * @see Limelight
   */
  public overrideAngleToNote(CommandSwerveDrivetrain drivetrain ,Limelight limelight) {
    m_limelight = limelight;
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelight.hasTarget()) {
        angleToNote = Rotation2d.fromDegrees(LimelightHelpers.getTX(Constants.LimelightConstants.kCameraName));
        fieldRelRotation2d = m_drivetrain.getRotation().minus(angleToNote);
        m_drivetrain.setOverrideAngle(fieldRelRotation2d);
    } else {
        m_drivetrain.setOverrideAngle(null);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setOverrideAngle(null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
