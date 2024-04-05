// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.AutoCommands.autoIntakeNote;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;
import frc.robot.Vision.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class autoCollectNote extends ParallelRaceGroup {

    CommandSwerveDrivetrain m_drivetrain;
    IntakeSubsystem m_intake;
    StageSubsystem m_stage;
    Limelight m_limelight;
    SwerveRequest.FieldCentricFacingAngle m_head;

    /** Creates a new autoCollectNote. */
    public autoCollectNote(CommandSwerveDrivetrain drivetrain, IntakeSubsystem intake, StageSubsystem stage,
            Limelight limelight, SwerveRequest.FieldCentricFacingAngle head) {
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_stage = stage;
        m_limelight = limelight;
        m_head = head;
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(new autoRunToNote(m_drivetrain, m_limelight, m_head).withTimeout(2.00));
        addCommands(new autoIntakeNote(m_intake, m_stage));
    }
}
