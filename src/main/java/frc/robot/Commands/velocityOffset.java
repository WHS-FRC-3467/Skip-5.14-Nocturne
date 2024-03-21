// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Stage.StageSubsystem;
import frc.robot.Util.FieldCentricAiming;
import frc.robot.Util.TunableNumber;

public class velocityOffset extends Command {

    CommandSwerveDrivetrain m_drivetrain;
    StageSubsystem m_stageSubsystem;
    FieldCentricAiming m_FieldCentricAiming;

    boolean m_isDone;

    Timer shotTimer;
    Boolean ranOnce;

    Pose2d currentRobotPose;
    Translation2d currentRobotTranslation;
    Rotation2d currentAngleToSpeaker;

    Pose2d futureRobotPose2d;
    Translation2d futureRobotTranslation;
    Rotation2d futureAngleToSpeaker;

    ChassisSpeeds speeds;
    Translation2d moveDelta;
    Translation2d armDelta;

    /**The calculated the time until the note leaves based on the constant and time since button press */
    Double timeUntilShot; 
    BooleanSupplier m_isShooting; 

    Double correctedDistance;
    Rotation2d correctedRotation;

    Translation2d lockedMove;
    Translation2d lockedArm;

    TunableNumber timeToShoot = new TunableNumber("timeToShoot", Constants.ShooterConstants.kTimeToShoot);
    TunableNumber timeToScore = new TunableNumber("timeToScore", Constants.ShooterConstants.kTimeToScore);
    



    /** Calculates the velocity compensated target to shoot at 
     * @param drivetrain CommandSwerveDrivetrain instance
     * @param isShooting Whether the shot has started
     * @see FieldCentricAiming
    */
    public velocityOffset(CommandSwerveDrivetrain drivetrain, BooleanSupplier isShooting) {
        m_drivetrain = drivetrain;
        m_isShooting = isShooting;
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
/*         if (m_isShooting.getAsBoolean()) {
            if (!ranOnce) {
                shotTimer.start();
                ranOnce = true;
            }
        } */
        //Get current translation of the drivetrain
        currentRobotTranslation = m_drivetrain.getState().Pose.getTranslation(); 
        //Calculate angle relative to the speaker from current pose
        currentAngleToSpeaker = m_FieldCentricAiming.getAngleToSpeaker(currentRobotTranslation); 
        //Get current drivetrain velocities in field relative terms
        speeds = m_drivetrain.getFieldRelativeChassisSpeeds(); 
        
        timeUntilShot = Constants.ShooterConstants.kTimeToShoot - shotTimer.get();
        if (timeUntilShot < 0) {
            timeUntilShot = 0.00;
        }
        
        //Calculate change in x/y distance due to time and velocity
        //moveDelta = new Translation2d(timeUntilShot*(speeds.vxMetersPerSecond),timeUntilShot*(speeds.vyMetersPerSecond));
        moveDelta = new Translation2d(timeToShoot.get() *(speeds.vxMetersPerSecond),timeToShoot.get() *(speeds.vyMetersPerSecond));
        armDelta = new Translation2d(timeToScore.get() *(speeds.vxMetersPerSecond),timeToScore.get() *(speeds.vyMetersPerSecond));

        if (m_isShooting.getAsBoolean()) {
            if (!ranOnce) {
                lockedMove = moveDelta;
                lockedArm = armDelta;
                shotTimer.start();
                ranOnce = true;
            } else {
                moveDelta = lockedMove;
                armDelta = lockedArm;
            }

        }

        //futureRobotPose is the position the robot will be at timeUntilShot in the future
        futureRobotTranslation = currentRobotTranslation.plus(moveDelta);
        // Angle to the speaker at future position
        futureAngleToSpeaker = m_FieldCentricAiming.getAngleToSpeaker(futureRobotTranslation);


        //The amount to add to the current angle to speaker to aim for the future
        correctedRotation = futureAngleToSpeaker;
        //correctedRotation = currentAngleToSpeaker; //Uncomment to disable future pose aiming
        // Get the future distance to speaker
        //correctedDistance = m_FieldCentricAiming.getDistToSpeaker(futureRobotTranslation);
        correctedDistance = m_FieldCentricAiming.getDistToSpeaker(currentRobotTranslation.plus(armDelta));
        m_drivetrain.setVelocityOffset(correctedRotation,correctedDistance); //Pass the offsets to the drivetrain
        // Pass angle as an override for path planner rotation to do shoot on the move during path execution
        m_drivetrain.setOverrideAngle(correctedRotation); 
    
        if (Constants.RobotConstants.kIsAutoAimTuningMode) {
            SmartDashboard.putNumber("Robot Angle To Speaker",currentAngleToSpeaker.getDegrees());
            SmartDashboard.putNumber("Robot Dist To Speaker",m_FieldCentricAiming.getDistToSpeaker(currentRobotTranslation));
            SmartDashboard.putNumber("xDelta", speeds.vxMetersPerSecond);
            SmartDashboard.putNumber("yDelta", speeds.vyMetersPerSecond);
            SmartDashboard.putNumber("futureang", futureAngleToSpeaker.getDegrees());
            SmartDashboard.putNumber("futureDist", correctedDistance);
            SmartDashboard.putNumber("Correction Angle", correctedRotation.getDegrees());
            SmartDashboard.putNumber("timeUntilShot", timeUntilShot);
            SmartDashboard.putNumber("Angle to ADD", futureAngleToSpeaker.minus(currentAngleToSpeaker).getDegrees());
            SmartDashboard.putNumber("Angle error", currentAngleToSpeaker.minus(m_drivetrain.getState().Pose.getRotation()).getDegrees());
            if (timeUntilShot< 0.02 && timeUntilShot > 0) {
                SmartDashboard.putNumber("Angle error at t=0", currentAngleToSpeaker.minus(m_drivetrain.getState().Pose.getRotation()).getDegrees());
            }
            
        }

              

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shotTimer.stop();
        shotTimer.reset();
        ranOnce = false;
        //m_isDone = true;
        m_drivetrain.setOverrideAngle(null);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isDone;
    }

}
