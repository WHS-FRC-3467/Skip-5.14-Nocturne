// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Util.FieldCentricAiming;
import frc.robot.Util.Setpoints;
import frc.robot.Util.TunableNumber;
import frc.robot.Util.Setpoints.GameState;

public class calibrateLookupTable extends Command {
    CommandSwerveDrivetrain m_drivetrain;
    ArmSubsystem m_arm;
    ShooterSubsystem m_shooter;
    driveToPose m_driveToPose;
    FieldCentricAiming m_fieldCentricAiming = new FieldCentricAiming();
    Command driveCommand;

    Pose2d speakerPos;
    double calibrateAngleToSpeaker = Units.degreesToRadians(5); 

    Translation2d currentTarget;
    Rotation2d currentAngle;
    ArrayList<Translation2d> aimingPositions = new ArrayList<>();
    Setpoints currentSetpoint;
    TunableNumber indexer = new TunableNumber("Lookup Table: Calibration Index", 0);
    TunableNumber angleTuner = new TunableNumber("Lookup Table: Angle", 0);
    TunableNumber leftSpeedTuner = new TunableNumber("Lookup Table: Left Shooter Speed", 0);
    TunableNumber rightSpeedTuner = new TunableNumber("Lookup Table: Right Shooter Speed", 0);

    /** Creates a new calibrateLookupTable. */
    public calibrateLookupTable(CommandSwerveDrivetrain drivetrain, ArmSubsystem arm, ShooterSubsystem shooter) {
        m_drivetrain = drivetrain;
        m_arm = arm;
        m_shooter = shooter;
        m_fieldCentricAiming = new FieldCentricAiming();
        currentSetpoint = new Setpoints(0, 0.4, 0, 0, GameState.LOOKUP);
       
        if (Constants.RobotConstants.kIsAutoAimTuningMode) {
            SmartDashboard.putData("Lookup Table: Next Index", new InstantCommand(() -> incIndex()));
            SmartDashboard.putData("Lookup Table: Print Setpoint", new InstantCommand(() -> printSetpoints()));
        }     

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        speakerPos = m_fieldCentricAiming.getSpeakerPos();
        int allianceInvert = 1;
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) { //Want the robot to move position from blue and negative from red
            allianceInvert *= -1;
        }
        // Creates positions to calibrate from every .5 meters
        for (double dist = 2; dist <= 7; dist = dist + .5) {
            aimingPositions.add(new Translation2d(speakerPos.getX() - Math.cos(calibrateAngleToSpeaker)*dist*allianceInvert,
                                                  speakerPos.getY() + Math.sin(calibrateAngleToSpeaker)*dist));
        }
        
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
        if (currentSetpoint.shooterLeft > 0) {
            m_shooter.runShooter(currentSetpoint.shooterLeft, currentSetpoint.shooterRight);
        } else {
            m_shooter.stopShooter();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.stopShooter();
        driveCommand.cancel();
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
