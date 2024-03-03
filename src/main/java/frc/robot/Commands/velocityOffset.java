// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Stage.StageSubsystem;
import frc.robot.Util.FieldCentricAiming;


public class velocityOffset extends Command {

    CommandSwerveDrivetrain m_drivetrain;
    StageSubsystem m_stageSubsystem;
    FieldCentricAiming m_FieldCentricAiming;

    boolean m_isDone;

    Pose2d currentRobotPose;
    Translation2d currentRobotTranslation;
    Rotation2d currentRobotRotation;
    Double currentAngleToSpeaker;

    Pose2d futureRobotPose2d;
    Translation2d futureRobotTranslation;
    Rotation2d futureRotation2d;
    Double futureAngleToSpeaker;

    ChassisSpeeds speeds;
    Double correctionAngle;
    Double timeUntilShot;
    Double xDelta;
    Double yDelta;
    Translation2d moveDelta;
    Rotation2d correctedPose;
    DoubleSupplier m_trigger;
    Timer shotTimer;
    Boolean ranOnce;
    Double correctedDistance;


    /** Creates a new velocityOffset. */
    public velocityOffset(CommandSwerveDrivetrain drivetrain, DoubleSupplier triggerAxis) {
        m_drivetrain = drivetrain;
        m_trigger = triggerAxis;
        shotTimer = new Timer();
        ranOnce = false;
        m_FieldCentricAiming = new FieldCentricAiming();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_isDone = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        //Starts shot timer after trigger press
        if (m_trigger.getAsDouble() > Constants.ControllerConstants.triggerThreashold) {
            if (!ranOnce) {
                shotTimer.start();
                ranOnce = true;
            }
        }

        currentRobotTranslation = m_drivetrain.getState().Pose.getTranslation(); //Get current translation of the drivetrain
        currentRobotRotation = m_FieldCentricAiming.getAngleToSpeaker(currentRobotTranslation); //Calculate angle relative to the speaker from current pose
        speeds = m_drivetrain.getFieldRelativeChassisSpeeds(); //Get current drivetrain velocities in field relative terms

        //timeToShoot represents the time it take a note to leave the shooter after button is pressed
        //timeUntil shot calculates the time until the note leaves based on the constant and time since button press
        timeUntilShot = Constants.ShooterConstants.kTimeToShoot - shotTimer.get();
        if (timeUntilShot < 0) {
            timeUntilShot = 0.00;
        }

        //Calculate change in x/y distance due to time and velocity
        xDelta = timeUntilShot*(speeds.vxMetersPerSecond);
        yDelta = timeUntilShot*(speeds.vyMetersPerSecond);
        moveDelta = new Translation2d(xDelta,yDelta);

        //futureRobotPose is the position the robot will be at timeUntilShot in the future
        futureRobotTranslation = currentRobotTranslation.plus(moveDelta);
        //Angle to the speaker at future position
        futureRotation2d = m_FieldCentricAiming.getAngleToSpeaker(futureRobotTranslation);

        //The amount to add to the current angle to speaker to aim for the future
        //correctionAngle = currentAngleToSpeaker - futureAngleToSpeaker;
        correctedPose = currentRobotRotation.minus(futureRotation2d);
        correctedPose = (correctedPose.times(-1)).plus(m_FieldCentricAiming.getAngleToSpeaker(currentRobotTranslation));

        // Get the future distance to speaker
        correctedDistance = m_FieldCentricAiming.getDistToSpeaker(futureRobotTranslation);
        m_drivetrain.setVelocityOffset(correctedPose,correctedDistance); //Pass the offsets to the drivetrain
        

        
 
        if (Constants.RobotConstants.kIsAutoAimTuningMode) {
            //SmartDashboard.putNumber("Robot Angle To Speaker",m_drivetrain.calcAngleToSpeaker());
            //SmartDashboard.putNumber("Robot Dist To Speaker",m_drivetrain.calcDistToSpeaker());
            //SmartDashboard.putNumber("xDelta", xDelta);
            //SmartDashboard.putNumber("yDelta", yDelta);
            //SmartDashboard.putNumber("futureang", futureAngleToSpeaker);
            SmartDashboard.putNumber("Correction Angle", correctedPose.getDegrees());
            SmartDashboard.putNumber("timeUntilShot", timeUntilShot);
            SmartDashboard.putNumber("futureDist", correctedDistance);
            //SmartDashboard.putNumber("Rot2Speaker", m_drivetrain.RotToSpeaker().getDegrees());
            SmartDashboard.putNumber("pose rot", m_drivetrain.getState().Pose.getRotation().getDegrees());
            
            //SmartDashboard.putNumber("time Const", Constants.ShooterConstants.timeToShoot);
            //SmartDashboard.putNumber("currentTime", shotTimer.get());
            //SmartDashboard.putNumber("trig", m_trigger.getAsDouble());
        }

              

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shotTimer.stop();
        shotTimer.reset();
        ranOnce = false;
        //m_isDone = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isDone;
    }

    public Rotation2d getCorrectedTarget() {
        return this.correctedPose;
    }

}
