// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/*
 * This Default command always runs in the background for the Arm controller and permits
 * the Operator to seamlessly switch out of PID mode and control the Arm with a joystick.
 */

public class ArmDefault extends Command {

    /** Creates a new ArmDefault. */
    ArmSubsystem m_arm;
    BooleanSupplier m_joyMode;
    DoubleSupplier m_valueSrc;

    public ArmDefault(ArmSubsystem arm, BooleanSupplier joyMode, DoubleSupplier valueSrc) {
        m_arm = arm;
        m_valueSrc = valueSrc;
        m_joyMode = joyMode;
        addRequirements(m_arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // If Operator is holding a particular button, then disable PID and go to Manual control mode
        if (m_joyMode.getAsBoolean()) {
            if (m_arm.isEnabled()) m_arm.disable();
            m_arm.setArmVoltage(12.0 * m_valueSrc.getAsDouble() * 0.3);
        // Else, switch to Profiled PID control
        } else {
            if (!m_arm.isEnabled()) m_arm.enable();
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
