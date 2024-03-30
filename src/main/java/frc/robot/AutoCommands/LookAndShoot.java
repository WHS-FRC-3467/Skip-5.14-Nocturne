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
    DoubleSupplier m_distance;
    SwerveRequest.FieldCentricFacingAngle m_head;
    double m_alliance;

    public LookAndShoot(CommandSwerveDrivetrain drivetrain, IntakeSubsystem intake, StageSubsystem stage,
            ArmSubsystem arm, ShooterSubsystem shooter, DoubleSupplier distance,
            SwerveRequest.FieldCentricFacingAngle head, double alliance) {
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_stage = stage;
        m_arm = arm;
        m_shooter = shooter;
        m_distance = distance;
        m_head = head;

        addCommands(new AutoLookUpShot(m_drivetrain, m_arm, m_shooter, m_distance, 5.0, false)
                .andThen(m_stage.feedNote2ShooterCommand())
                .andThen(m_arm.prepareForIntakeCommand()));
        addCommands(new velocityOffset(m_drivetrain, () -> m_stage.isStageRunning()));
        addCommands(m_drivetrain.applyRequest(
                () -> m_head.withVelocityX(0.0)
                        .withVelocityY(0.0)
                        .withTargetDirection(m_drivetrain.getVelocityOffset())
                        .withDeadband(Constants.maxSpeed * 0.1)
                        .withRotationalDeadband(0)));

    }
}
