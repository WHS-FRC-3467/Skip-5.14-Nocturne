// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;

public class autoShootNote extends Command {
  /** Creates a new autoShootNote. */
  ArmSubsystem m_arm;
  ShooterSubsystem m_shooter;
  StageSubsystem m_stage;
  CommandSwerveDrivetrain m_drivetrain;
  boolean m_isDone;

  public autoShootNote(ArmSubsystem armSub, ShooterSubsystem shootSub, StageSubsystem stageSub, CommandSwerveDrivetrain drivesub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = armSub;
    m_shooter = shootSub;
    m_stage = stageSub;
    m_drivetrain = drivesub;
    addRequirements(stageSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_arm.isArmJointAtSetpoint() && m_shooter.areWheelsAtSpeed() && m_drivetrain.isAtAngle()) {
        m_stage.feedNote2ShooterCommand().schedule();
        m_isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isDone;
  }
}
