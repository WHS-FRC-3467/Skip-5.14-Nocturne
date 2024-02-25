
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
    TalonSRX m_centeringMotor = new WPI_TalonSRX(CanConstants.ID_IntakeCtrRoller);

    /* Current Limits config */
    //private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

    /* Run Intake motor using Duty Cycle (-1.0 -> 1.0) */
    private final DutyCycleOut m_speed = new DutyCycleOut(0);

    /* Neutral output control for stopping the Intake motor */
    private final NeutralOut m_brake = new NeutralOut();

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {

        /* Configure the Intake motor */
        var m_configuration = new TalonFXConfiguration();

        /* Set Intake motor to Brake */
        m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        /* Set the motor direction */
        m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
/*
        // Configure the motor to use a supply limit of 5 amps IF we exceed 10 amps for over 1 second
        m_currentLimits.SupplyCurrentLimit = 5; // Limit to 5 amps
        m_currentLimits.SupplyCurrentThreshold = 10; // If we exceed 10 amps
        m_currentLimits.SupplyTimeThreshold = 1.0; // For at least 1 second
        m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it

        m_currentLimits.StatorCurrentLimit = 30; // Limit stator to 30 amps
        m_currentLimits.StatorCurrentLimitEnable = true; // And enable it
        m_configuration.CurrentLimits = m_currentLimits;
*/

        /* Config the peak outputs */
        m_configuration.Voltage.PeakForwardVoltage = 12.0;
        m_configuration.Voltage.PeakReverseVoltage = -12.0;

        /* Apply Intake motor configs */
        m_intakeMotor.getConfigurator().apply(m_configuration);

        // optimize StatusSignal rates for the Talon
        m_intakeMotor.getDutyCycle().setUpdateFrequency(100);
        m_intakeMotor.optimizeBusUtilization();
 
        // Set Centering motors to factory defaults
        m_centeringMotor.configFactoryDefault();

        // Set centering motors direction
        m_centeringMotor.setInverted(true);

        // Set Centering motors to Coast
        m_centeringMotor.setNeutralMode(NeutralMode.Coast);

        // Config centering motor current limit
        m_centeringMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(
                true,
                15,
                20,
                0.10
            )
        );

        /* Config the peak and nominal outputs */
        m_centeringMotor.configNominalOutputForward(0.0, 30);
        m_centeringMotor.configNominalOutputReverse(0.0, 30);
        m_centeringMotor.configPeakOutputForward(1.0, 30);
        m_centeringMotor.configPeakOutputReverse(1.0, 30);

        // slows unneeded CAN status fames
        m_centeringMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
        m_centeringMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
        m_centeringMotor.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
    }

    public void simulationInit() {
        /* If running in Simulation, setup simulated Talons */
        PhysicsSim.getInstance().addTalonFX(m_intakeMotor, 0.001);
        PhysicsSim.getInstance().addTalonSRX(m_centeringMotor, 1.0, 89975.0);
    }



    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        if (RobotConstants.kIsIntakeTuningMode) {
            SmartDashboard.putNumber("Intake Current Draw", m_intakeMotor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Intake Center Current Draw", m_centeringMotor.getSupplyCurrent());
        }
    }

    public void simulationPeriodic() {
        // If running in simulation, update the sims
            PhysicsSim.getInstance().run();
    }

    /**
     * 
     * @param speed speed to set intake motor at
     */
    public void runIntake(double speed) {
        m_intakeMotor.setControl(m_speed.withOutput(speed));
        m_centeringMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void stopIntake() {
        m_intakeMotor.setControl(m_brake);
        m_centeringMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    /*
     * Command Factories
     */
    public Command runIntakeCommand() {
        return new RunCommand(()->this.runIntake(IntakeConstants.kIntakeSpeed), this);
    }

    public Command stopIntakeCommand() {
        return new InstantCommand(()->this.stopIntake(), this);
    }

    public Command ejectIntakeCommand() {
        return new RunCommand(()->this.runIntake(IntakeConstants.kEjectSpeed), this);
    }

}
