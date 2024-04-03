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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1, .5);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1, .5);

    private final ProfiledPIDController xController = new ProfiledPIDController(10, 2, 2, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(10, 2, 2, Y_CONSTRAINTS);
    

    private final SwerveRequest.FieldCentricFacingAngle swerveRequestFacing = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagic)
    .withVelocityX(0.0)
    .withVelocityY(0.0);

    public driveToPose(CommandSwerveDrivetrain drivetrain, Translation2d target, Rotation2d angle) {
        m_drivetrain = drivetrain;
        targetTranslation = target;
        targetAngle = angle;
        xController.setTolerance(0.02);
        yController.setTolerance(0.02);
        swerveRequestFacing.HeadingController = new PhoenixPIDController(20, 0, 1.5);
        swerveRequestFacing.HeadingController.setTolerance(0.01);
        SmartDashboard.putData("driveToPose xController",xController);
        SmartDashboard.putData("driveToPose yController",yController);
        SmartDashboard.putData("driveToPoseHead",swerveRequestFacing.HeadingController);
        SmartDashboard.putBoolean("driveToPose xController at Target", xController.atGoal());
        SmartDashboard.putBoolean("driveToPose yController at Target", yController.atGoal());
        

        addRequirements(drivetrain);
    }



    

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //System.out.println(targetTranslation);
        //System.out.println(targetAngle);
        robotPose = m_drivetrain.getState().Pose;
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
        xController.setGoal(targetTranslation.getX());
        yController.setGoal(targetTranslation.getY());
        isFinished = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        xController.setGoal(targetTranslation.getX());
        yController.setGoal(targetTranslation.getY());
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
