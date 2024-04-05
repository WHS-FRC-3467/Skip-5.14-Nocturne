// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.Constants.LimelightConstants.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Util.CommandXboxPS5Controller;
import frc.robot.Vision.Limelight;
import frc.robot.Vision.LimelightHelpers;

public class autoRunToNote extends Command {
    CommandSwerveDrivetrain m_drivetrain;
    Limelight m_limelight;
    SwerveRequest.FieldCentricFacingAngle m_head;
    SwerveRequest.RobotCentric m_forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    CommandXboxPS5Controller m_driverCtrl;
    double tx = 0;
    double omegaSpeed;

    boolean m_isFinished = false;
    boolean m_isOverLine = false;

    private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(.1, .01);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(.095, 0, 0, OMEGA_CONSTRATINTS);

    /** Creates a new aimAtNote. */
    public autoRunToNote(CommandSwerveDrivetrain drivetrain, Limelight limelight, SwerveRequest.FieldCentricFacingAngle head) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;
        m_head = head;
        omegaController.setTolerance(1);
        omegaController.setGoal(0);
        //SmartDashboard.putData("Note PID",omegaController);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Constants.RobotConstants.kIsAutoAimTuningMode) {
            SmartDashboard.putData("Note Detect PID",omegaController);
        }
        omegaController.reset(tx);
        m_isFinished = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("AUTO NOTE RUNNING");
        tx = LimelightHelpers.getTX(kCameraName);
        omegaSpeed = omegaController.calculate(tx);
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }

        if (m_limelight.hasTarget() && !m_isOverLine) {
            m_drivetrain.setControl(m_forwardStraight
                    .withVelocityX(-Constants.maxSpeed * (1 - Math.abs(tx) / 32) * .50) // Constants.halfSpeed
                    .withVelocityY(0)
                    .withRotationalRate(omegaSpeed));
        }
        if (DriverStation.getAlliance().get() != null && DriverStation.isAutonomousEnabled()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                if (m_drivetrain.getState().Pose.getX() > Constants.FieldConstants.BLUE_AUTO_PENALTY_LINE) {
                    System.out.println("Find Note1");
                    m_isOverLine = true;
                }
            } else {
                if (m_drivetrain.getState().Pose.getX() < Constants.FieldConstants.RED_AUTO_PENALTY_LINE) {
                    System.out.println("Find Note2");
                    m_isOverLine = true;
                }
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("BBBBBBBBBBBBBBBBBBBBBBBBBBB");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
