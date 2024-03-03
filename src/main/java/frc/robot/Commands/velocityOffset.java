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


public class velocityOffset extends Command {

    CommandSwerveDrivetrain m_drivetrain;
    StageSubsystem m_stageSubsystem;

    boolean m_isDone;

    Pose2d robotPose;
    Translation2d currentPos;
    Double currentAngleToSpeaker;
    Translation2d futureRobotPose;
    Double futureAngleToSpeaker;
    ChassisSpeeds speeds;
    Alliance alliance;
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
    Pose2d futureRobotPose2d;
    Rotation2d currentRotation2d;
    Rotation2d futureRotation2d;

    /** Creates a new velocityOffset. */
    public velocityOffset(CommandSwerveDrivetrain drivetrain, DoubleSupplier triggerAxis) {
        m_drivetrain = drivetrain;
        //m_stageSubsystem = stage;
        m_trigger = triggerAxis;
        shotTimer = new Timer();
        ranOnce = false;
        //addRequirements(stage);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //System.out.println("Starting Velocity Offset correction command");
        m_isDone = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        //Starts shot timer after trigger press
        //TODO: Sync trigger threashold with driver controller bindings
        if (m_trigger.getAsDouble() > Constants.ControllerConstants.triggerThreashold) {
            if (!ranOnce) {
                
                //System.out.println("Starting Internal Timer");
                shotTimer.start();
                ranOnce = true;
            }
        }

        //Get current translation of the drivetrain
        currentPos = m_drivetrain.getState().Pose.getTranslation();
        //Calculate angle relative to the speaker from current pose
        //currentAngleToSpeaker = m_drivetrain.compAngleToSpeaker(currentPos).getDegrees();
        currentRotation2d = m_drivetrain.compAngleToSpeaker(currentPos);
        //currentAngleToSpeaker = m_drivetrain.calcAngleToSpeaker(currentPos);
        //Get current drivetrain velocities in field relative terms
        speeds = m_drivetrain.getFieldRelativeChassisSpeeds();

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
        futureRobotPose = currentPos.plus(moveDelta);
        //Angle to the speaker at future position
        //futureAngleToSpeaker = m_drivetrain.calcAngleToSpeaker();
        //futureAngleToSpeaker = m_drivetrain.compAngleToSpeaker(futureRobotPose).getDegrees();
        futureRotation2d = m_drivetrain.compAngleToSpeaker(futureRobotPose);

        //The amount to add to the current angle to speaker to aim for the future
        //correctionAngle = currentAngleToSpeaker - futureAngleToSpeaker;
        correctedPose = currentRotation2d.minus(futureRotation2d);
        correctedPose = (correctedPose.times(-1)).plus(m_drivetrain.compAngleToSpeaker(currentPos));

        //correctedPose = Rotation2d.fromDegrees(-correctionAngle).plus(m_drivetrain.RotToSpeaker());
        // Wrap the input using Modulus to prevent un-needed 180deg spins
         
/*          if (m_drivetrain.getAlliance() == DriverStation.Alliance.Red) {
            correctedPose = Rotation2d.fromDegrees(MathUtil.inputModulus(correctedPose.getDegrees(), -180, 180));
            //correctedPose = Rotation2d.fromDegrees(correctedPose.getDegrees() * -1);
        } */

        // Get the future distance to speaker
        correctedDistance = m_drivetrain.calcDistToSpeaker(futureRobotPose);
        //Pass the offsets to the drivetrain
        //m_drivetrain.setVelOffset(correctedPose,correctedDistance);
        m_drivetrain.setVelOffset(correctedPose,correctedDistance);
        

        
 
        if (Constants.RobotConstants.kIsAutoAimTuningMode) {
            SmartDashboard.putNumber("Robot Angle To Speaker",m_drivetrain.calcAngleToSpeaker());
            SmartDashboard.putNumber("Robot Dist To Speaker",m_drivetrain.calcDistToSpeaker());
            //SmartDashboard.putNumber("xDelta", xDelta);
            //SmartDashboard.putNumber("yDelta", yDelta);
            //SmartDashboard.putNumber("futureang", futureAngleToSpeaker);
            SmartDashboard.putNumber("Correction Angle", correctedPose.getDegrees());
            SmartDashboard.putNumber("timeUntilShot", timeUntilShot);
            SmartDashboard.putNumber("futureDist", correctedDistance);
            SmartDashboard.putNumber("Rot2Speaker", m_drivetrain.RotToSpeaker().getDegrees());
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
