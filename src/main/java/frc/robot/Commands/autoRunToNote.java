// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.Constants.LimelightConstants.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
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
    double ty = 0;
    double omegaSpeed;

    private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(.1, .01);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(.1, 0, 0, OMEGA_CONSTRATINTS);

    /** Creates a new aimAtNote. */
    public autoRunToNote(CommandSwerveDrivetrain drivetrain, Limelight limelight, SwerveRequest.FieldCentricFacingAngle head) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;
        m_head = head;
        omegaController.setTolerance(1);
        omegaController.setGoal(0);
        SmartDashboard.putData("Note PID",omegaController);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //SmartDashboard.putData("Note Detect PID",m_head.HeadingController);
        omegaController.reset(tx);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        tx = LimelightHelpers.getTX(kCameraName);
        ty = LimelightHelpers.getTY(kCameraName);

        omegaSpeed = omegaController.calculate(tx);
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }

        if (m_limelight.hasTarget()) {
            /* if ((Math.abs(tx) > 5.5 && Math.abs(ty) > 10.00) || Math.abs(tx) > 15) {
                m_drivetrain.setControl(m_head
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withTargetDirection(m_drivetrain.getRotation().plus(Rotation2d.fromDegrees(-tx)))
                        //.withDeadband(Constants.maxSpeed * 0.1)
                        //.withRotationalDeadband(Units.degreesToRadians(0)));
                );

            } else {
                m_drivetrain.setControl(m_forwardStraight
                        .withVelocityX(-Constants.maxSpeed * 0.3) //Constants.halfSpeed
                        .withVelocityY(0)
                        .withRotationalRate(0)
                        .withDeadband(Constants.maxSpeed * 0.1)
                        .withRotationalDeadband(Units.degreesToRadians(2)));
            } */
            m_drivetrain.setControl(m_forwardStraight
                        .withVelocityX(-Constants.maxSpeed * (1 - Math.abs(tx)/32) * .5) //Constants.halfSpeed
                        .withVelocityY(0)
                        .withRotationalRate(omegaSpeed)
                        .withDeadband(Constants.maxSpeed * 0.1)
                        );
        }

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
}
