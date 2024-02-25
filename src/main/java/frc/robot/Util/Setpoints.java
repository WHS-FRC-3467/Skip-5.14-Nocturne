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
        STOWED, INTAKE, SUBWOOFER, AMP, PODIUM, WING, PREPCLIMB, CLIMB, TRAP, LOOKUP
    }

    // Display the commanded Arm state on the dashboard
    public static void displayArmState(GameState state) {

        switch (state) {
            case STOWED:
                SmartDashboard.putString("Arm State", "STOWED");
                break;
            case INTAKE:
                SmartDashboard.putString("Arm State", "INTAKE");
                break;
            case SUBWOOFER:
                SmartDashboard.putString("Arm State", "SUBWOOFER");
                break;
            case AMP:
                SmartDashboard.putString("Arm State", "AMP");
                break;
            case PODIUM:
                SmartDashboard.putString("Arm State", "PODIUM");
                break;
            case WING:
                SmartDashboard.putString("Arm State", "WING");
                break;
            case PREPCLIMB:
                SmartDashboard.putString("Arm State", "PREPCLIMB");
                break;
            case CLIMB:
                SmartDashboard.putString("Arm State", "CLIMB");
                break;
            case TRAP:
                SmartDashboard.putString("Arm State", "TRAP");
                break;
            case LOOKUP:
                SmartDashboard.putString("Arm State", "LOOKUP");
                break;
            default:
                SmartDashboard.putString("Arm State", "OTHER");
        }
    }


}