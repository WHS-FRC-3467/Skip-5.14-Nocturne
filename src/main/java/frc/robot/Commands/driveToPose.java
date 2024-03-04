// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Util.FieldCentricAiming;

public class driveToPose extends Command {
    CommandSwerveDrivetrain m_drivetrain;
    FieldCentricAiming m_FieldCentricAiming;
    double xSpeed;
    double ySpeed;
    double omegaSpeed;
    Pose2d robotPose;
    boolean isFinished;

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(8, 8);

    private final ProfiledPIDController xController = new ProfiledPIDController(4, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(4, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRATINTS);

    private final SwerveRequest.FieldCentricFacingAngle swerveRequestFacing = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagic)
    .withVelocityX(0.0)
    .withVelocityY(0.0);

    public driveToPose(CommandSwerveDrivetrain drivetrain) {
        xController.setTolerance(0.1);
        yController.setTolerance(0.1);
        m_drivetrain = drivetrain;
        m_FieldCentricAiming = new FieldCentricAiming();
        swerveRequestFacing.HeadingController = new PhoenixPIDController(10, 0, 0);
        swerveRequestFacing.HeadingController.setTolerance(0.01);

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        robotPose = m_drivetrain.getState().Pose;
        var speakerPose = m_FieldCentricAiming.getSpeakerPos();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
        xController.setGoal(speakerPose.getX()-2);
        yController.setGoal(speakerPose.getY());
        isFinished = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        robotPose = m_drivetrain.getState().Pose;
        xSpeed = xController.calculate(robotPose.getX());
        if (xController.atGoal()) {xSpeed = 0;}

        ySpeed = yController.calculate(robotPose.getY());
        if (yController.atGoal()) {ySpeed = 0;}

        m_drivetrain.setControl(swerveRequestFacing.withVelocityX(xSpeed)
                    .withVelocityY(ySpeed)
                    .withTargetDirection(Rotation2d.fromDegrees(0)));

    }

    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
