// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;

public class driveToPose extends Command {
    CommandSwerveDrivetrain m_drivetrain;
    double xSpeed;
    double ySpeed;
    double omegaSpeed;
    Pose2d robotPose;
    Translation2d targetTranslation = new Translation2d(0, 0);
    Rotation2d targetAngle = new Rotation2d(0);
    boolean isFinished;

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(4, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(4, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(8, 8);

    private final ProfiledPIDController xController = new ProfiledPIDController(5, 0.5, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(5, 0.5, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(3, .5, 0, OMEGA_CONSTRATINTS);

    private final SwerveRequest.FieldCentricFacingAngle swerveRequestFacing = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagic)
    .withVelocityX(0.0)
    .withVelocityY(0.0);

    public driveToPose(CommandSwerveDrivetrain drivetrain, Translation2d target, Rotation2d angle) {
        m_drivetrain = drivetrain;
        targetTranslation = target;
        targetAngle = angle;
        xController.setTolerance(0.01);
        yController.setTolerance(0.01);
        swerveRequestFacing.HeadingController = new PhoenixPIDController(10, 0, 2.25);
        swerveRequestFacing.HeadingController.setTolerance(0.01);
        

        addRequirements(drivetrain);
    }

    public driveToPose(CommandSwerveDrivetrain drivetrain, DoubleSupplier x,DoubleSupplier y, DoubleSupplier a) {
        m_drivetrain = drivetrain;
        targetTranslation = new Translation2d(x.getAsDouble(), y.getAsDouble());
        targetAngle = new Rotation2d(a.getAsDouble());
        xController.setTolerance(0.1);
        yController.setTolerance(0.1);
        swerveRequestFacing.HeadingController = new PhoenixPIDController(10, 0, 2.25);
        swerveRequestFacing.HeadingController.setTolerance(0.01);
        

        addRequirements(drivetrain);
    }

    

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println(targetTranslation);
        robotPose = m_drivetrain.getState().Pose;
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
        xController.setGoal(targetTranslation.getX());
        yController.setGoal(targetTranslation.getY());
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
                    .withTargetDirection(targetAngle));

    }

    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }

   public void setTarget(Translation2d pose) {
      targetTranslation = pose;
  }

  public void setAngle(Rotation2d angle) {
    targetAngle = angle;

  } 
}
