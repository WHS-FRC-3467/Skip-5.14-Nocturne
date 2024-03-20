// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.velocityOffset;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;


public class MoveAndShoot extends ParallelRaceGroup {
  /** Creates a new LookAndShoot. */
  CommandSwerveDrivetrain m_drivetrain;
  IntakeSubsystem m_intake;
  StageSubsystem m_stage;
  ArmSubsystem m_arm;
  ShooterSubsystem m_shooter;
  DoubleSupplier m_distance;

  /**
   * Shoot on move with shooting when ready
   * 
   * @param drivetrain
   * @param stage
   * @param arm
   * @param shooter
   * @param distance
   */
  public MoveAndShoot(CommandSwerveDrivetrain drivetrain, StageSubsystem stage,
          ArmSubsystem arm, ShooterSubsystem shooter, DoubleSupplier distance) {
      m_drivetrain = drivetrain;
      m_stage = stage;
      m_arm = arm;
      m_shooter = shooter;
    m_distance = distance;
    addCommands(new AutoLookUpShot(m_drivetrain, m_arm, m_shooter, m_distance)
            //.andThen(new WaitCommand(0.05))
            .andThen(m_stage.feedNote2ShooterCommand())
            .andThen(m_arm.prepareForIntakeCommand()));
    addCommands(new velocityOffset(m_drivetrain, () -> m_stage.isStageRunning()));
}
}
