// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** A Class to hold setpoints for both the Shooter and the Arm, since they work in tandem.
 * 
 * @param arm The Arm setpoint
 * @param shooter The Shooter setpoint
 * @param state The GameState that these setpoints define
 * 
 */
public class Setpoints {
    public double arm;
    public double tolerance;
    public double shooterLeft;
    public double shooterRight;
    public GameState state;

    public Setpoints(double arm, double tolerance, double shooterLeft, double shooterRight, GameState state) {
        this.arm = arm;
        this.tolerance = tolerance;
        this.shooterLeft = shooterLeft;
        this.shooterRight = shooterRight;
        this.state = state;
    }

    public enum GameState {
        STOWED, INTAKE, SUBWOOFER, AMP, CLIMB, HARMONY, AIM, FEED, CUSTOM
    }

    // Display the commanded Arm state on the dashboard
    public static void displayArmState(GameState state) {
        SmartDashboard.putString("Arm State", state.toString());
    }
}