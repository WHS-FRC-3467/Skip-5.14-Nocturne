// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;

/* Local */
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Util.Setpoints;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;
import frc.robot.Constants;

public class autoShoot extends Command {
  /** Creates a new autoShoot. */
  ArmSubsystem m_armSubsystem;
  StageSubsystem m_stageSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  Setpoints m_setpoints;
  boolean m_isDone;
  public autoShoot(ArmSubsystem arm, StageSubsystem stage, IntakeSubsystem intake, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = arm;
    m_stageSubsystem = stage;
    m_intakeSubsystem = intake;
    m_shooterSubsystem = shooter;
    addRequirements(arm, stage, intake, shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Do we have a note
    if (m_stageSubsystem.isNoteInStage()) {
        m_stageSubsystem.stopStage();
        // Set arm Setpoint
        System.out.println("Trying to prep to shoot");
        m_shooterSubsystem.setShooterSetpoints(Constants.RobotConstants.WING);
        // After we have a Note in the Stage, bring Arm to requested position
        m_armSubsystem.updateArmSetpoint(Constants.RobotConstants.WING);
        // Spin up Shooter
        m_shooterSubsystem.runShooter();
        // Check Shooter Speed
        //System.out.println(m_shooterSubsystem.areWheelsAtSpeed());
        if (m_shooterSubsystem.areWheelsAtSpeed() && m_armSubsystem.isArmJointAtSetpoint()) {
            // Feed Note to Shooter
            System.out.println("SHOOTING");
            m_stageSubsystem.ejectFront(1);
            //m_isDone = true;
        }
    } else {
        //m_armSubsystem.prepareForIntakeCommand();
        m_armSubsystem.updateArmSetpoint(RobotConstants.INTAKE);
        //m_intakeSubsystem.runIntakeCommand();
        m_intakeSubsystem.runIntake(IntakeConstants.kIntakeSpeed);
        m_stageSubsystem.runStage(.6);

    }
  }

    
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isDone;
  }
}