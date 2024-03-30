// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;
import frc.robot.Util.FieldCentricAiming;
import frc.robot.Util.Setpoints;
import frc.robot.Util.ShooterPreset;
import frc.robot.Util.TunableNumber;
import frc.robot.Util.VisionLookUpTable;

public class smartShootOnMove extends Command {

    CommandSwerveDrivetrain m_drivetrain;
    IntakeSubsystem m_intake;
    StageSubsystem m_stage;
    ArmSubsystem m_arm;
    ShooterSubsystem m_shooter;
    FieldCentricAiming m_FieldCentricAiming;

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

    Rotation2d lockedRotation;
    double lockedDistance;

    TunableNumber timeToShoot = new TunableNumber("Smart timeToShoot", .15);
    TunableNumber timeToBeReady = new TunableNumber("Smart timeToBeReady", .5);

    double m_maxShotDist = 3.5;
    Timer shotTimer;
    boolean timerIsRunning = false;

    ShooterPreset m_shotInfo;
    VisionLookUpTable m_VisionLookUpTable;
    Setpoints m_setpoints;

    boolean m_isFinished = false;

    /** Creates a new smartShootOnMove. */
    public smartShootOnMove(CommandSwerveDrivetrain drivetrain, StageSubsystem stage,
            ArmSubsystem arm, ShooterSubsystem shooter, double maxShotDist) {

        m_drivetrain = drivetrain;
        m_stage = stage;
        m_arm = arm;
        m_shooter = shooter;
        m_maxShotDist = maxShotDist;
        m_FieldCentricAiming = new FieldCentricAiming();
        m_VisionLookUpTable = new VisionLookUpTable();
        m_setpoints = RobotConstants.LOOKUP;
        shotTimer = new Timer();

        addRequirements(m_arm,m_shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("SCHEDULED");
        
        m_isFinished = false;
        timerIsRunning = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentRobotTranslation = m_drivetrain.getState().Pose.getTranslation();
        currentAngleToSpeaker = m_FieldCentricAiming.getAngleToSpeaker(currentRobotTranslation); 
        // Get current drivetrain velocities in field relative terms
        speeds = m_drivetrain.getFieldRelativeChassisSpeeds();

        // Calculate change in x/y distance due to time and velocity
        moveDelta = new Translation2d(timeToBeReady.get() * (speeds.vxMetersPerSecond),
                                      timeToBeReady.get() * (speeds.vyMetersPerSecond));

        futureRobotTranslation = currentRobotTranslation.plus(moveDelta);
        futureAngleToSpeaker = m_FieldCentricAiming.getAngleToSpeaker(futureRobotTranslation);
        
        correctedDistance = m_FieldCentricAiming.getDistToSpeaker(futureRobotTranslation);
        correctedRotation = futureAngleToSpeaker;

        if (timerIsRunning) {
            System.out.println(shotTimer.get());
            if (shotTimer.hasElapsed(timeToBeReady.get() / 2)) {
                if (!m_shooter.isShooterAtSpeed()) {
                    System.out.println("SHOOTER ISNT READY");
                    if (!m_arm.isArmJointAtSetpoint()) {
                        System.out.println("ARM ISNT READY");
                        if (m_drivetrain.isRotatingFast()) {
                            System.out.println("DRIVE ROT ISNT READY");
                            

                        }

                    }

                }
                if (!m_shooter.isShooterAtSpeed() || !m_arm.isArmJointAtSetpoint() || m_drivetrain.isRotatingFast()) {
                    System.out.println("WONT BE READY, RESTARTING SHOT");
                            shotTimer.stop();
                            shotTimer.reset();
                            timerIsRunning = false;
                }

            }
            if (shotTimer.hasElapsed(timeToBeReady.get()-timeToShoot.get()) && !m_stage.isStageRunning()) {
                System.out.println("STARTING STAGE");
                m_stage.feedNote2ShooterCommand().schedule();
                SmartDashboard.putNumber("Angle error at t=0", currentAngleToSpeaker.minus(m_drivetrain.getState().Pose.getRotation()).getDegrees());SmartDashboard.putNumber("Angle error at t=0", currentAngleToSpeaker.minus(m_drivetrain.getState().Pose.getRotation()).getDegrees());
            }

            if (shotTimer.hasElapsed(timeToBeReady.get())) {
                System.out.println("NOTE SHOULD BE SHOOTING NOW");
                
                m_isFinished = true;
            }

        } else {
            if (correctedDistance <= m_maxShotDist) {
                System.out.println("STARTING READYUP TIMER");
                shotTimer.start();
                timerIsRunning = true;
            }
            lockedRotation = correctedRotation;
            lockedDistance = correctedDistance;

        }


        m_shotInfo = m_VisionLookUpTable.getShooterPreset(lockedDistance);
        m_setpoints.arm = m_shotInfo.getArmAngle();
        m_setpoints.shooterLeft = m_shotInfo.getLeftShooter();
        m_setpoints.shooterRight = m_shotInfo.getRightShooter();
        m_arm.updateArmSetpoint(m_setpoints);
        m_shooter.runShooter(m_setpoints.shooterLeft, m_setpoints.shooterRight);

        m_drivetrain.setVelocityOffset(lockedRotation, lockedDistance); // Pass the offsets to the drivetrain
        m_drivetrain.setOverrideAngle(lockedRotation);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shotTimer.stop();
        shotTimer.reset();
        m_shooter.stopShooter();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
