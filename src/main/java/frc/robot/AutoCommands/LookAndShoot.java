// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Commands.velocityOffset;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;
import frc.robot.Vision.PhotonVision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LookAndShoot extends ParallelRaceGroup {
  /** Creates a new LookAndShoot. */
  CommandSwerveDrivetrain m_drivetrain;
  IntakeSubsystem m_intake;
  StageSubsystem m_stage;
  ArmSubsystem m_arm;
  ShooterSubsystem m_shooter;
  PhotonVision m_photonVision;
  DoubleSupplier m_distance;

  public LookAndShoot(CommandSwerveDrivetrain drivetrain, IntakeSubsystem intake, StageSubsystem stage
                        , ArmSubsystem arm, ShooterSubsystem shooter, PhotonVision photonVision, DoubleSupplier distance) {
    m_drivetrain = drivetrain;
    m_intake = intake;
    m_stage = stage;
    m_arm = arm;
    m_shooter = shooter;
    m_photonVision = photonVision;
    m_distance = distance;
    addCommands(new AutoLookUpShot(m_arm, m_shooter, m_distance));
    addCommands(new velocityOffset(m_drivetrain, () -> 0.0));
    addCommands(new autoShootNote(m_arm, m_shooter, m_stage));
  }
}
