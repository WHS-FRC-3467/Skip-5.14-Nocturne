// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Util.FieldCentricAiming;
import frc.robot.Util.Setpoints;
import frc.robot.Util.TunableNumber;
import frc.robot.Util.Setpoints.GameState;

public class calibrateLookupTable extends Command {
    CommandSwerveDrivetrain m_drivetrain;
    ArmSubsystem m_arm;
    driveToPose m_driveToPose;
    FieldCentricAiming m_fieldCentricAiming = new FieldCentricAiming();
    Command driveCommand;

    Translation2d currentTarget;
    Rotation2d currentAngle;
    ArrayList<Translation2d> aimingPositions = new ArrayList<>();
    Setpoints currentSetpoint;
    TunableNumber indexer = new TunableNumber("Lookup Table: Calibration Index", 0);
    TunableNumber angleTuner = new TunableNumber("Lookup Table: Angle", 0);
    TunableNumber leftSpeedTuner = new TunableNumber("Lookup Table: Left Shooter Speed", 0);
    TunableNumber rightSpeedTuner = new TunableNumber("Lookup Table: Right Shooter Speed", 0);

    /** Creates a new calibrateLookupTable. */
    public calibrateLookupTable(CommandSwerveDrivetrain drivetrain, ArmSubsystem arm) {
        m_drivetrain = drivetrain;
        m_arm = arm;
        m_fieldCentricAiming = new FieldCentricAiming();
        currentSetpoint = new Setpoints(0, 0.4, 0, 0, GameState.LOOKUP);
       
        

        SmartDashboard.putData("Lookup Table: Next Index", new InstantCommand(() -> incIndex()));
        SmartDashboard.putData("Lookup Table: Print Setpoint", new InstantCommand(() -> printSetpoints()));

        

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        aimingPositions.add(new Translation2d(m_fieldCentricAiming.getSpeakerPos().getX() - 1.5,
                m_fieldCentricAiming.getSpeakerPos().getY()));
        aimingPositions.add(new Translation2d(m_fieldCentricAiming.getSpeakerPos().getX() - 2,
                m_fieldCentricAiming.getSpeakerPos().getY()));
        aimingPositions.add(new Translation2d(m_fieldCentricAiming.getSpeakerPos().getX() - 3,
                m_fieldCentricAiming.getSpeakerPos().getY()));
        aimingPositions.add(new Translation2d(m_fieldCentricAiming.getSpeakerPos().getX() - 4,
                m_fieldCentricAiming.getSpeakerPos().getY()));
        aimingPositions.add(new Translation2d(m_fieldCentricAiming.getSpeakerPos().getX() - 5,
                m_fieldCentricAiming.getSpeakerPos().getY()));
        aimingPositions.add(new Translation2d(m_fieldCentricAiming.getSpeakerPos().getX() - 6,
                m_fieldCentricAiming.getSpeakerPos().getY()));
        aimingPositions.add(new Translation2d(m_fieldCentricAiming.getSpeakerPos().getX() - 7,
                m_fieldCentricAiming.getSpeakerPos().getY()));


        aimingPositions.add(new Translation2d(m_fieldCentricAiming.getSpeakerPos().getX() - 3,
                m_fieldCentricAiming.getSpeakerPos().getY()));
        aimingPositions.add(new Translation2d(m_fieldCentricAiming.getSpeakerPos().getX() - 3,
                m_fieldCentricAiming.getSpeakerPos().getY() + 1.45));
        aimingPositions.add(new Translation2d(m_fieldCentricAiming.getSpeakerPos().getX() - 3,
                m_fieldCentricAiming.getSpeakerPos().getY() - 1.45));
        
        driveCommand = new driveToPose(m_drivetrain,getTranslationTarget(),getRotationTarget());
        driveCommand.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentSetpoint.arm = angleTuner.get();
        currentSetpoint.shooterLeft = leftSpeedTuner.get();
        currentSetpoint.shooterRight = rightSpeedTuner.get();

        m_arm.updateArmSetpoint(currentSetpoint);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveCommand.cancel();;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public Translation2d getTranslationTarget() {
        return aimingPositions.get((int) indexer.get());
    }

    public Rotation2d getRotationTarget() {
        return m_fieldCentricAiming.getAngleToSpeaker(aimingPositions.get((int) indexer.get()));
    }

    public void incIndex() {
        if (aimingPositions.size() > indexer.get() + 1) {
            indexer.set(indexer.get() + 1);
        } else {
            indexer.set(0);
        }

    }

    public void printSetpoints() {
        System.out.println("Current Good Setpoint");
        System.out.print(" Dist from Speaker ");
        System.out.print(m_fieldCentricAiming.getDistToSpeaker(m_drivetrain.getState().Pose.getTranslation()));
        System.out.print(" Angle of Arm ");
        System.out.print(angleTuner.get());
        System.out.print(" Left Shooter Speed ");
        System.out.print(leftSpeedTuner.get());
        System.out.print(" Right Shooter Speed ");
        System.out.print(rightSpeedTuner.get());
        System.out.println("");
    }
}
