// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.Util.Setpoints.GameState;

public class smartShoot extends Command {

    CommandSwerveDrivetrain m_drivetrain;
    IntakeSubsystem m_intake;
    StageSubsystem m_stage;
    ArmSubsystem m_arm;
    ShooterSubsystem m_shooter;
    FieldCentricAiming m_FieldCentricAiming;

    Pose2d currentRobotPose;
    Translation2d currentRobotTranslation;
    Rotation2d currentAngleToTarget;

    Pose2d futureRobotPose2d;
    Translation2d futureRobotTranslation;
    Rotation2d futureAngleToTarget;

    ChassisSpeeds speeds;
    Translation2d moveDelta;
    Translation2d armDelta;

    /**
     * The calculated the time until the note leaves based on the constant and time
     * since button press
     */
    Double timeUntilShot;
    BooleanSupplier m_isShooting;

    Double correctedDistance;
    Rotation2d correctedRotation;

    Rotation2d lockedRotation;
    double lockedDistance;

    TunableNumber timeToShoot = new TunableNumber("Smart timeToShoot", Constants.RobotConstants.kTimeToShoot);
    TunableNumber timeToBeReady = new TunableNumber("Smart timeToBeReady", Constants.RobotConstants.kTimeToReady);

    boolean m_isShootOnTheMove;
    Timer shotTimer;
    boolean timerIsRunning = false;

    ShooterPreset m_shotInfo;
    VisionLookUpTable m_VisionLookUpTable;
    Setpoints m_setpoints;

    int m_target = 0; //Speaker = 0, Feed = 1
    double maxShotDist;

    boolean m_isFinished = false;

    /** Creates a new smartShootOnMove. */
    public smartShoot(CommandSwerveDrivetrain drivetrain, StageSubsystem stage,
            ArmSubsystem arm, ShooterSubsystem shooter, boolean isShootOnTheMove, int target) {

        m_drivetrain = drivetrain;
        m_stage = stage;
        m_arm = arm;
        m_shooter = shooter;
        m_isShootOnTheMove = isShootOnTheMove;
        m_FieldCentricAiming = new FieldCentricAiming();
        m_VisionLookUpTable = new VisionLookUpTable();
        m_setpoints = RobotConstants.LOOKUP;
        m_target = target;
        shotTimer = new Timer();

        if (target == 1) {
            maxShotDist = Constants.RobotConstants.robotMaxFeedShotDist;
        } else {
            if (isShootOnTheMove) {
                maxShotDist = Constants.RobotConstants.robotMaxDynamicShotDist;
            } else {
                maxShotDist = Constants.RobotConstants.robotMaxStaticShotDist;
            }
        }

        addRequirements(m_arm, m_stage, m_shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("SMART SHOOT SCHEDULED");
        m_arm.enable();
        m_isFinished = false;
        timerIsRunning = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentRobotTranslation = m_drivetrain.getState().Pose.getTranslation();
        currentAngleToTarget = m_FieldCentricAiming.getTargetAngle(currentRobotTranslation,m_target);
        // Get current drivetrain velocities in field relative terms
        speeds = m_drivetrain.getFieldRelativeChassisSpeeds();

        // Calculate change in x/y distance due to time and velocity
        moveDelta = new Translation2d(timeToBeReady.get() * (speeds.vxMetersPerSecond),
                timeToBeReady.get() * (speeds.vyMetersPerSecond));

        futureRobotTranslation = currentRobotTranslation.plus(moveDelta);
        futureAngleToTarget = m_FieldCentricAiming.getTargetAngle(futureRobotTranslation,m_target);

        correctedDistance = m_FieldCentricAiming.getTargetDist(futureRobotTranslation,m_target);
        correctedRotation = futureAngleToTarget;

        if (timerIsRunning) {
            System.out.println(shotTimer.get());
            if (shotTimer.hasElapsed(timeToBeReady.get() / 2)) {
                if (!m_shooter.isShooterAtSpeed() || !m_arm.isArmJointAtSetpoint() || m_drivetrain.isRotatingFast()
                        || !m_drivetrain.isAtFutureAngle()
                        || (!m_isShootOnTheMove && m_drivetrain.isMoving())) {
                    System.out.println("WONT BE READY, RESTARTING SHOT");
                    shotTimer.stop();
                    shotTimer.reset();
                    timerIsRunning = false;
                    if (!m_shooter.isShooterAtSpeed()) {
                        System.out.println("Shooter is not at speed");
                    }
                    if (m_drivetrain.isRotatingFast()) {
                        System.out.println("Drivetrain is rotating too fast");
                    }
                    if (!m_arm.isArmJointAtSetpoint()) {
                        System.out.println("Arm is not at setpoint");
                    }
                    if (!m_drivetrain.isAtFutureAngle()) {
                        System.out.println("Drivetrain is not at angle");
                    }
                    if (!m_isShootOnTheMove && m_drivetrain.isMoving()) {
                        System.out.println("Drivetrain is moving during static shot");
                    }
                }

            }
            if (shotTimer.hasElapsed(timeToBeReady.get() - timeToShoot.get()) && !m_stage.isStageRunning()) {
                System.out.println("Starting stage");
                // m_stage.feedNote2ShooterCommand().schedule();
                m_stage.ejectFront();
                SmartDashboard.putNumber("Angle error at t=0",
                        currentAngleToTarget.minus(m_drivetrain.getState().Pose.getRotation()).getDegrees());
            }

            if (shotTimer.hasElapsed(timeToBeReady.get())) {
                System.out.println("Shot should be complete now");

            }
            if (!m_stage.isNoteInStage()) {
                m_isFinished = true;
            }

        } else {
            if (correctedDistance <= maxShotDist) {
                System.out.println("STARTING READYUP TIMER FOR DYNAMIC SHOT ");
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
        if (!DriverStation.isAutonomousEnabled()) {
            m_shooter.stopShooter();
        }
        m_stage.stopStage();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
