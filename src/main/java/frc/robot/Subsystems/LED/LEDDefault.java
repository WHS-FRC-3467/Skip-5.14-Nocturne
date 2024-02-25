package frc.robot.Subsystems.LED;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;


public class LEDDefault extends Command {
  /** Creates a new LEDDefault. */
  LEDSubsystem m_led;
  IntakeSubsystem m_intake;
  ShooterSubsystem m_shooter;
  public LEDDefault(LEDSubsystem led, IntakeSubsystem intake, ShooterSubsystem shooter) {
    m_led = led;
    m_intake = intake;
    m_shooter = shooter;    
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

/*    // might have to change Blinkin2 values 
    else if(false){
    //flash orange if intake is extending       
      m_led.setColor(255, 153, 53);

    }

    else if (false){
    // set purple
        m_led.setColor(138, 0, 230);
    }
*/
    




    /* 
    
    else if(GamePiece.getGamePiece() == GamePieceType.Cube){
      //Set color to purple
      m_led.setColor(186, 0, 255);
    }
    else if(GamePiece.getGamePiece() == GamePieceType.Cone){
      m_led.setColor(255,191,0);
    }
    else if (GamePiece.getGamePiece() == GamePieceType.None){
      m_led.setColor(255,0,0);
    }

    */
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