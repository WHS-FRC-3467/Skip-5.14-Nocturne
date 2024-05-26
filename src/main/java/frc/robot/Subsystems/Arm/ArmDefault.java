// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;

/*
 * This Default command always runs in the background for the Arm controller and permits
 * the Operator to seamlessly switch out of PID mode and control the Arm with a joystick.
 */

public class ArmDefault extends Command {

    /** Creates a new ArmDefault. */
    ArmSubsystem m_arm;

    public ArmDefault(ArmSubsystem arm) {
        m_arm = arm;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

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
