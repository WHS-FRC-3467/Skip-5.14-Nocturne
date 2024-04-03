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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.StageConstants;
import frc.robot.sim.PhysicsSim;

public class StageSubsystem extends SubsystemBase {

    // Initialize devices
    TalonSRX m_stageMotor = new WPI_TalonSRX(CanConstants.ID_StageMotor);
    DigitalInput m_stageBeamBreak = new DigitalInput(DIOConstants.kStageBeamBreak);
    boolean m_noteInStage = false;
    BooleanSupplier m_noteInStageSupplier;
    boolean m_stageRunning = false;

    /** Creates a new StageSubsystem. */
    public StageSubsystem() {

        // Set motor to factory defaults
        m_stageMotor.configFactoryDefault();

        // Invert motor?
        m_stageMotor.setInverted(true);

        // Set motor to Brake
        m_stageMotor.setNeutralMode(NeutralMode.Brake);

        // Config current limit
        // m_stageMotor.configSupplyCurrentLimit(new
        // SupplyCurrentLimitConfiguration(true, 15, 20, 0.10));

        /* Config the peak and nominal outputs */
        m_stageMotor.configNominalOutputForward(0.0, 30);
        m_stageMotor.configNominalOutputReverse(0.0, 30);
        m_stageMotor.configPeakOutputForward(1.0, 30);
        m_stageMotor.configPeakOutputReverse(1.0, 30);

        // slows unneeded CAN status fames
        m_stageMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
        m_stageMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
        m_stageMotor.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);

    }

    public void simulationInit() {
        /* If running in Simulation, setup simulated Falcons */
        PhysicsSim.getInstance().addTalonSRX(m_stageMotor, 1.0, 89975.0);
    }

    @Override
    public void periodic() {

        // Check for change in beam break
        // Sensor returns true when beam NOT broken
        m_noteInStage = m_stageBeamBreak.get() ? false : true;

        SmartDashboard.putBoolean("Note In Stage?", m_noteInStage);

        if (RobotConstants.kIsStageTuningMode) {
            SmartDashboard.putNumber("Stage Current Draw", m_stageMotor.getSupplyCurrent());
        }
    }

    public void simulationPeriodic() {
        // If running in simulation, update the sims
        PhysicsSim.getInstance().run();
    }

    /**
     * 
     * @param speed speed to set Stage motor at
     */
    public void runStage(double speed) {
        m_stageMotor.set(ControlMode.PercentOutput, speed);
        m_stageRunning = true;
    }

    public void runStage() {
        m_stageMotor.set(ControlMode.PercentOutput, StageConstants.kIntakeSpeed);
        m_stageRunning = true;
    }

    public void stopStage() {
        m_stageMotor.set(ControlMode.PercentOutput, 0.0);
        m_stageRunning = false;
    }

    // Do not use if the shooter's target velocity is zero.
    public void ejectFront(double speed) {
        this.runStage(speed);
    }

    public void ejectFrontDebug(double speed) {
        this.runStage(speed);
    }

    public void ejectFrontDebug2(double speed) {
        this.runStage(speed);
    }


    public void ejectBack(double speed) {
        this.runStage((-1.0) * speed);
    }

    public boolean isNoteInStage() {
        return m_noteInStage;
    }

    public boolean isStageRunning() {
        return m_stageRunning;
    }

    /*
     * Command Factories
     */

    // Pass the Note to the Shooter
    public ConditionalCommand feedNote2ShooterCommand() {
        return new ConditionalCommand(feedWithBeam(),feedWithTimeout(),() -> m_noteInStage); 
    }

    public Command feedWithTimeout() {
        return new RunCommand(() -> this.ejectFrontDebug2(StageConstants.kFeedToShooterSpeed), this)
                .withTimeout(0.5) // run for 1.5 seconds
                .andThen(() -> this.stopStage());

    }

    public Command feedWithBeam() {
        return new RunCommand(() -> this.ejectFrontDebug(StageConstants.kFeedToShooterSpeed), this)
                .until(() -> !isNoteInStage()) // run until there is NOT a Note in the Stage
                .andThen(() -> this.stopStage());
    }

    public Command feedStageCommand() {
        return new RunCommand(() -> this.ejectFront(.8), this)
                .until(() -> isNoteInStage()) // run until there is NOT a Note in the Stage
                .andThen(() -> this.stopStage());
    }

}
