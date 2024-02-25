package frc.robot.Subsystems.LED;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;


public class TimeLED extends Command {


  /** Creates a new LEDDefault. */
  LEDSubsystem m_led;
  public TimeLED(LEDSubsystem led) {
    m_led = led;    
    addRequirements(m_led);
    // Use addRequirements() here to declare subsystem dependencies.
  }


    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
  }


  @Override
  public boolean runsWhenDisabled(){
    return true;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override 
  public void execute() {
    if(RobotConstants.kIsTuningMode){
      // SmartDashboard.putBoolean("Is vision Mode", m_limelight.inVisionMode());
    }
    if(DriverStation.isDisabled()){
      m_led.ledColors();
    }

    double matchTime = Timer.getMatchTime();
        if (matchTime > 215.0) {
            //led green
            m_led.setColor(0, 255, 0);
        } 
        else if (matchTime > 60.0) {
            //led yellow
            m_led.setColor(230, 230, 0);
        }
        else if (matchTime > 30.0) {
            //led orange
            m_led.setColor(255, 153, 53);
        }
        else if (matchTime > 20.0) {
            //led red
            m_led.setColor(255, 0, 0);
        }
        else if (matchTime > 10.0) {
            //led white
            m_led.setColor(255, 255, 255);
        }
        else if (matchTime > 5.0) {
            //led flash white
            m_led.setColor(255, 255, 255);
    
        }
        
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}