package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanConstants;
//import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.Constants.StageConstants;
import frc.robot.Util.Setpoints;
import frc.robot.Util.TunableNumber;
import frc.robot.sim.PhysicsSim;

public class ShooterSubsystem extends SubsystemBase {

    // Initialize devices
    TalonFX m_shooterLeft = new TalonFX(18);
    TalonFX m_shooterRight = new TalonFX(19);

    // Declare ShooterState Variables - keep track of what the shooter is doing
    ShooterState m_ShooterState = ShooterState.STOP;
    ShooterState m_GoalState = ShooterState.STOP;

    public ShooterSubsystem() {
        
        var talonFXConfigurator = m_shooterLeft.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();

        // enable stator current limit
        limitConfigs.StatorCurrentLimit = 120;
        limitConfigs.StatorCurrentLimitEnable = true;

        talonFXConfigurator.apply(limitConfigs);
        
        m_shooterRight.setControl(new Follower(m_shooterLeft.getDeviceID(), true));

    }

    @Override
    public void periodic() {

    }

    public boolean isShooterAtSpeed(int speed) {
        // .get gets the set speed from 0 to 1. 319/3 is the free speed of a Falcon 500 (rotations per second)
        // TalonFX getVelocity() gets the Velocity of the device in mechanism rotations per second
        if ((m_shooterLeft.get() * 319 / 3 - ShooterConstants.k_SHOOTER_VELOCITY_TOLERANCE < m_shooterLeft.getVelocity().getValueAsDouble())
         && (m_shooterLeft.get() * 319 / 3 + ShooterConstants.k_SHOOTER_VELOCITY_TOLERANCE > m_shooterLeft.getVelocity().getValueAsDouble())){
            return true;
        }
        return false;
    }
    
    public void runShooter(double speed) {
        // Actually tell motors to run at the speed
        if (speed >= 0.1) {
            m_shooterLeft.set(speed);
        }
    }

    public void runShooter() {
        m_shooterLeft.set(0.8);
    }
    public void reverseShooter(double speed) {
        if (speed <= -0.1) {
            m_shooterLeft.set(speed);
        }
    }

    public void stopShooter() {
        m_shooterLeft.stopMotor();
    }

        /**
     * Example command factory method.
     *
     * @return runShooter(speed)
     */

    public Command runShooterCommand(double speed) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> runShooter(speed));
    }

        /**
         * Speed is set to 0.8
         * @return runShooter()
         */
    public Command runShooterCommand() {
        return runOnce(() -> runShooter());
    }

    public Command reverseShooterCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(() -> reverseShooter(Constants.ShooterConstants.k_SHOOTER_REV_VELOCITY));

    }

    public Command stopShooterCommand() {
        return runOnce(() -> stopShooter());
    }

}