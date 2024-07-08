package frc.robot.Subsystems.Stage;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.StageConstants;
//import frc.robot.Constants.StageConstants;
import frc.robot.sim.PhysicsSim;
import frc.robot.Util.ThriftyNova;

public class StageSubsystem extends SubsystemBase {

    // Initialize devices
    ThriftyNova thrifty_nova = new ThriftyNova(CanConstants.k_STAGE_CAN_ID);
    DigitalInput m_stageBeamBreak = new DigitalInput(DIOConstants.k_INTAKE_BEAM_BREAK);

    /** Creates a new StageSubsystem. */
    public StageSubsystem() {

    }

    @Override
    public void periodic() {

    }

    /**
     * 
     * @param speed speed to set Stage motor at
     */

    public boolean isAtSpeed(int speed) {
        if ((speed - StageConstants.k_STAGE_VELOCITY_TOLERANCE < thrifty_nova.getVelocity())
         && (speed + StageConstants.k_STAGE_VELOCITY_TOLERANCE > thrifty_nova.getVelocity())){
            return true;
        }
        return false;
    }
    
    public void runStage() {
        thrifty_nova.setPercentOutput(StageConstants.k_STAGE_VELOCITY);
    }
    public void reverseStage() {
        thrifty_nova.setPercentOutput(StageConstants.k_STAGE_REV_VELOCITY);
    }

    public void stopStage() {
        thrifty_nova.setPercentOutput(0.0);
    }

    // getCurrentDraw() to get the current draw of a thrifty nova

    /*
     * Command Factories
     */

             /**
         * Speed is set to 0.8
         * @return runShooter()
         */
    public Command runStageCommand() {
        return runOnce(() -> runStage());
    }

    public Command reverseStageCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(() -> reverseStage());

    }

    public Command stopStageCommand() {
        return runOnce(() -> stopStage());
    }

}
