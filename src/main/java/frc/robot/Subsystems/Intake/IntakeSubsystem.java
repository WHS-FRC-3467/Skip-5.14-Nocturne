// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;

public class IntakeSubsystem extends SubsystemBase {

    // Initalize Motors and Beam Break
    TalonFX m_intakeLead = new TalonFX(CanConstants.k_INTAKE_LEFT_CAN_ID);
    TalonFX m_intakeFollow = new TalonFX(CanConstants.k_INTAKE_RIGHT_CAN_ID);
    DigitalInput m_intakeBeam = new DigitalInput(DIOConstants.k_INTAKE_BEAM_BREAK);

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean isIntakeAtSpeed() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    public void intakeForward(double speed) {
        // Actually tell motors to run at the speed
        if (speed >= 0.1) {
            speed += 1;
        }
    }

    public void intakeReverse(double speed) {
        if (speed >= 0.1) {
            speed -= 1;
        }
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */

    public Command intakeFwdCommand(double speed) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> intakeForward(speed));
    }

    public Command intakeRevCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(() -> intakeReverse(1));

    }
}
