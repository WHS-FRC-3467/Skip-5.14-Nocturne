
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.sim.PhysicsSim;

public class IntakeSubsystem extends SubsystemBase {

    /* Initialize Talons */
    TalonFX m_intakeMotor = new TalonFX(CanConstants.ID_IntakeMotor);
    TalonFX m_intakeFollower = new TalonFX(CanConstants.ID_IntakeFollower);
    
    /* Current Limits config */
    private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

    /* Run Intake motor using Duty Cycle (-1.0 -> 1.0) */
    private final DutyCycleOut m_speed = new DutyCycleOut(0);

    /* Neutral output control for stopping the Intake motor */
    private final NeutralOut m_brake = new NeutralOut();

    /* Flag to tell if intake is running */
    private boolean m_intakeRunning = false;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {

        /* Configure the Intake motor */
        var m_configuration = new TalonFXConfiguration();
        var m_followerConfiguration = new TalonFXConfiguration();

        /* Set Intake motor to Brake */
        m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_followerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Set the motor direction */
        m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Configure the motor to use a supply limit of 60 amps IF we exceed 80 amps for over 0.1 second
        m_currentLimits.SupplyCurrentLimit = 20; // Limit to 60 amps
        m_currentLimits.SupplyCurrentThreshold = 40; // If we exceed 80 amps
        m_currentLimits.SupplyTimeThreshold = 0.1; // For at least 0.1 second
        m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it

        m_currentLimits.StatorCurrentLimit = 70; // Limit stator to 70 amps
        m_currentLimits.StatorCurrentLimitEnable = true; // And enable it
        m_configuration.CurrentLimits = m_currentLimits;
        m_followerConfiguration.CurrentLimits = m_currentLimits;

        /* Config the peak outputs */
        m_configuration.Voltage.PeakForwardVoltage = 12.0;
        m_configuration.Voltage.PeakReverseVoltage = -12.0;
        m_followerConfiguration.Voltage.PeakForwardVoltage = 12.0;
        m_followerConfiguration.Voltage.PeakReverseVoltage = -12.0;

        /* Apply Intake motor configs */
        //m_intakeMotor.getConfigurator().apply(m_configuration);

        // optimize StatusSignal rates for the Talon
/*         m_intakeMotor.getDutyCycle().setUpdateFrequency(100);
        m_intakeMotor.getSupplyCurrent().setUpdateFrequency(100);
        m_intakeMotor.getStatorCurrent().setUpdateFrequency(100);
        m_intakeMotor.optimizeBusUtilization();

        
        m_intakeFollower.getDutyCycle().setUpdateFrequency(100);
        m_intakeFollower.getSupplyCurrent().setUpdateFrequency(100);
        m_intakeFollower.getStatorCurrent().setUpdateFrequency(100);
        m_intakeFollower.optimizeBusUtilization(); */

        /*
         * Apply the configurations to the motors, and set one to follow the other in
         * the opposite direction
         */
        m_intakeMotor.getConfigurator().apply(m_configuration);
        m_intakeFollower.getConfigurator().apply(m_followerConfiguration);
        m_intakeFollower.setControl(new Follower(m_intakeMotor.getDeviceID(), false));
    }

    public void simulationInit() {
        /* If running in Simulation, setup simulated Talons */
        PhysicsSim.getInstance().addTalonFX(m_intakeMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(m_intakeFollower, 0.001);
        // PhysicsSim.getInstance().addTalonSRX(m_centeringMotor, 1.0, 89975.0);
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        if (RobotConstants.kIsIntakeTuningMode) {
            SmartDashboard.putNumber("Intake Current Draw", m_intakeMotor.getSupplyCurrent().getValueAsDouble());
            // SmartDashboard.putNumber("Intake Center Current Draw",
            // m_centeringMotor.getSupplyCurrent());
        }
    }

    public void simulationPeriodic() {
        // If running in simulation, update the sims
        PhysicsSim.getInstance().run();
    }

    /**
     * 
     * @param speed speed to set intake motor at (-1,1)
     */
    public void runIntake(double speed) {
        m_intakeMotor.setControl(m_speed.withOutput(speed));
        m_intakeRunning = true;
    }

    public void stopIntake() {
        m_intakeMotor.setControl(m_brake);
        m_intakeRunning = false;
    }

    public boolean isIntakeRunning() {
        return m_intakeRunning;
    }
    
    /*
     * Command Factories
     */
    public Command runIntakeCommand() {
        return new RunCommand(() -> this.runIntake(IntakeConstants.kIntakeSpeed), this);
    }

    public Command stopIntakeCommand() {
        return new InstantCommand(() -> this.stopIntake(), this);
    }

    public Command ejectIntakeCommand() {
        return new RunCommand(() -> this.runIntake(IntakeConstants.kEjectSpeed), this);
    }

}
