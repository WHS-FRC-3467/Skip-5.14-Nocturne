// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
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
  SwerveRequest.FieldCentricFacingAngle m_head;
  double m_alliance;

  public LookAndShoot(CommandSwerveDrivetrain drivetrain, IntakeSubsystem intake, StageSubsystem stage
                        , ArmSubsystem arm, ShooterSubsystem shooter, PhotonVision photonVision, DoubleSupplier distance,
                            SwerveRequest.FieldCentricFacingAngle head, double alliance) {
    m_drivetrain = drivetrain;
    m_intake = intake;
    m_stage = stage;
    m_arm = arm;
    m_shooter = shooter;
    m_photonVision = photonVision;
    m_distance = distance;
    m_head = head;
    head.HeadingController.setP(20);
    head.HeadingController.setI(75);
    head.HeadingController.setD(6);
    addCommands(new AutoLookUpShot(m_arm, m_shooter, m_distance).andThen(new WaitCommand(0.05)).andThen(m_stage.feedNote2ShooterCommand()).andThen(m_arm.prepareForIntakeCommand()));
    addCommands(new velocityOffset(m_drivetrain, () -> m_stage.isStageRunning()));
    if (m_drivetrain.getState().speeds.vxMetersPerSecond < 0.01 || m_drivetrain.getState().speeds.vyMetersPerSecond < 0.01) {
        addCommands(m_drivetrain.applyRequest(
        () -> m_head.withVelocityX(0.0 * Constants.maxSpeed * m_alliance)
                .withVelocityY(0.0 * Constants.maxSpeed * m_alliance)
                .withTargetDirection(m_drivetrain.getVelocityOffset())
                .withDeadband(Constants.maxSpeed * 0.1)
                .withRotationalDeadband(0)));
    }
    
    //addCommands(until(()->m_arm.isArmJointAtSetpoint() && m_shooter.areWheelsAtSpeed()).andThen(m_stage.feedNote2ShooterCommand()));
  }
}
