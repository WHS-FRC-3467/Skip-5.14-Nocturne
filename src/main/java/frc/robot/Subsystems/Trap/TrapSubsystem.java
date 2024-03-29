// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Trap;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Commands.driveToPose;
import frc.robot.Util.TunableNumber;
import frc.robot.sim.PhysicsSim;

public class TrapSubsystem extends SubsystemBase {
    CommandSwerveDrivetrain m_drivetrain;
    TalonSRX m_blowerMotor = new WPI_TalonSRX(CanConstants.ID_Blower);
    Timer m_runningTimer = new Timer();
    TunableNumber trap_dist = new TunableNumber("Trap Dist", 1);
    Pose2d trap_target = new Pose2d();
    Pose3d trap_location = AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(11).get();

    /** Creates a new TrapSubsystem. */
    public TrapSubsystem(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_blowerMotor.configFactoryDefault();

        m_blowerMotor.setInverted(true);

        m_blowerMotor.setNeutralMode(NeutralMode.Coast);

        m_blowerMotor.configNominalOutputForward(0.0, 30);
        m_blowerMotor.configNominalOutputReverse(0.0, 30);
        m_blowerMotor.configPeakOutputForward(1.0, 30);
        m_blowerMotor.configPeakOutputReverse(1.0, 30);

        m_blowerMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
        m_blowerMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
        m_blowerMotor.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
        
        
    }

    public void simulationInit() {
        /* If running in Simulation, setup simulated Falcons */
        PhysicsSim.getInstance().addTalonSRX(m_blowerMotor, 1.0, 89975.0);
    }

    @Override
    public void periodic() {
        
        trap_target = trap_location.toPose2d().plus(new Transform2d(new Translation2d(trap_dist.get(), 0), trap_location.getRotation().toRotation2d()));
        //System.out.println(trap_target);
        // This method will be called once per scheduler run
    }

    public void startBlower() {
        m_blowerMotor.set(ControlMode.PercentOutput, 1.0);
        m_runningTimer.start();
    }

    public void stopBlower() {
        m_blowerMotor.set(ControlMode.PercentOutput, 0.0);
        m_runningTimer.stop();
        m_runningTimer.reset();
    }

    public Pose2d getTrapTarget() {
        return trap_target;
    }

    public boolean isRunning() {
        return m_runningTimer.hasElapsed(2); // Wait for blower to get to speed
    }

    public Command startBlowerCommand() {
        return new InstantCommand(() -> startBlower(), this);
    }

    public Command stopBlowerCommand() {
        return new InstantCommand(() -> stopBlower(), this);
    }

}
