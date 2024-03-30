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
import frc.robot.Vision.PhotonVision;

public class driveToTrap extends Command {
    CommandSwerveDrivetrain m_drivetrain;
    PhotonVision m_photonvision;
    ShooterSubsystem m_shooter;

    driveToPose m_driveToPose;
    Command driveCommand;

    Translation2d currentTarget;
    Rotation2d currentAngle;
    Setpoints currentSetpoint;

    TunableNumber trap_dist = new TunableNumber("Trap Dist", .625);
    Pose2d trap_target = new Pose2d();
    Pose3d trap_location = AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(11).get();

    TunableNumber leftSpeedTuner = new TunableNumber("Trap : Left Shooter Speed", 20);
    TunableNumber rightSpeedTuner = new TunableNumber("Trap: Right Shooter Speed", 20);

    Rotation2d offset = new Rotation2d(-Math.PI/2);

    /** Creates a new calibrateLookupTable. */
    public driveToTrap(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter, PhotonVision photonvision) {
        m_drivetrain = drivetrain;
        m_shooter = shooter;
        m_photonvision = photonvision;
        currentSetpoint = new Setpoints(0, 0.4, 0, 0, GameState.TRAP);
       
        if (Constants.RobotConstants.kIsAutoAimTuningMode) {

        }     

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Rotation2d temp = Rotation2d.fromDegrees(trap_location.getRotation().toRotation2d().getDegrees()+240);
        trap_target = trap_location.toPose2d().plus(new Transform2d(new Translation2d(trap_dist.get(), 0), temp));
        driveCommand = new driveToPose(m_drivetrain,getTranslationTarget(),getRotationTarget());
        driveCommand.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //System.out.println(trap_location);
        System.out.println(trap_target); 
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
