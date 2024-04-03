// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Util.Setpoints;
import frc.robot.Util.TunableNumber;
import frc.robot.Util.Setpoints.GameState;

public class driveToTrap extends Command {
    CommandSwerveDrivetrain m_drivetrain;
    ShooterSubsystem m_shooter;

    driveToPose m_driveToPose;
    Command driveCommand;

    Translation2d currentTarget;
    Rotation2d currentAngle;
    Setpoints currentSetpoint;

    TunableNumber trap_dist = new TunableNumber("Trap Dist", .6);
    Pose2d trap_target = new Pose2d();
    Pose3d trap_location;
    

    TunableNumber leftSpeedTuner = new TunableNumber("Trap : Left Shooter Speed", 20);
    TunableNumber rightSpeedTuner = new TunableNumber("Trap: Right Shooter Speed", 20);

    double offset = 0;
    double dist_to_trap = 999;
    double current_dist;
    int closest_trap = 0;
    int current_tag = 0;


    /** Creates a new calibrateLookupTable. */
    public driveToTrap(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter) {
        m_drivetrain = drivetrain;
        m_shooter = shooter;
        currentSetpoint = new Setpoints(0, 0.4, 0, 0, GameState.TRAP);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        dist_to_trap = 999;
        current_dist = 999;
        closest_trap = 0;
        current_tag = 0;
        trap_location = new Pose3d();

        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            for (int i = 0; i < 3; i++) {
                current_tag = i+14;
                Pose2d current_trap_location = AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(current_tag).get().toPose2d();
                current_dist = m_drivetrain.getState().Pose.getTranslation().getDistance(current_trap_location.getTranslation());
                if (dist_to_trap > current_dist) {
                    dist_to_trap = current_dist;
                    closest_trap = current_tag;
                }
            }
        } else {
            for (int i = 0; i < 3; i++) {
                current_tag = i+11;
                Pose2d current_trap_location = AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(current_tag).get().toPose2d();
                current_dist = m_drivetrain.getState().Pose.getTranslation().getDistance(current_trap_location.getTranslation());
                if (dist_to_trap > current_dist) {
                    dist_to_trap = current_dist;
                    closest_trap = current_tag;
                }
            }
        }
        switch(closest_trap) {
            case 11: offset = 120; break;
            case 12: offset = -120; break;
            case 13: offset = 0; break;
            case 14: offset = 180; break;
            case 15: offset = -60; break; 
            case 16: offset = 60; break;
            default: System.out.println("INVALID TRAP"); break;
        }
        
        trap_location = AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(closest_trap).get();
        if (Constants.RobotConstants.kIsTrapTuningMode) {
            System.out.println(closest_trap);
            System.out.println(offset);

        }

        trap_target = trap_location.toPose2d().plus(new Transform2d(new Translation2d(trap_dist.get(), 0), Rotation2d.fromDegrees(0)));
        driveCommand = new driveToPose(m_drivetrain,trap_target.getTranslation(),Rotation2d.fromDegrees(offset));
        driveCommand.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentSetpoint.shooterLeft = leftSpeedTuner.get();
        currentSetpoint.shooterRight = rightSpeedTuner.get();
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
        return trap_target.getTranslation();
    }

    public Rotation2d getRotationTarget() {
        return trap_target.getRotation();
    }
}
