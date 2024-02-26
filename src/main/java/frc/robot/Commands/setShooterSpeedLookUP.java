// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Util.Setpoints;
import frc.robot.Util.ShooterPreset;
import frc.robot.Util.VisionLookUpTable;

public class setShooterSpeedLookUP extends Command {
    Setpoints m_setpoints;
    ShooterSubsystem m_shooterSubsystem;
    ShooterPreset m_shotInfo;
    DoubleSupplier m_distance;
    VisionLookUpTable m_VisionLookUpTable;
  /** Creates a new setShooterSpeedLookUP. */
  public setShooterSpeedLookUP(ShooterSubsystem shootSub, DoubleSupplier distance) {
    // Use addRequirements() here to declare subsystem dependencies.
        m_shooterSubsystem = shootSub;
        m_VisionLookUpTable = new VisionLookUpTable();
        m_distance = distance;

        addRequirements(shootSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = m_distance.getAsDouble();
    m_shotInfo = m_VisionLookUpTable.getShooterPreset(distance);

    m_setpoints.shooterLeft = m_shotInfo.getLeftShooter();
    m_setpoints.shooterRight = m_shotInfo.getRightShooter();
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
