package frc.robot.Subsystems.Stage;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.StageConstants;
import frc.robot.sim.PhysicsSim;

import frc.robot.Util.shootTimer;
public class StageSubsystem extends SubsystemBase {

    // Initialize devices
    TalonSRX m_stageMotor = new WPI_TalonSRX(CanConstants.ID_StageMotor);
    DigitalInput m_stageBeamBreak = new DigitalInput(DIOConstants.kStageBeamBreak);
    shootTimer m_timer;
    boolean m_noteInStage = false;
    long startShootTime;
    boolean hasStarted = false;
    boolean hasRun = false;
    /* Timer */
    //Timer m_timer = new Timer();

    /** Creates a new StageSubsystem. */
    public StageSubsystem(shootTimer shootTimer) {
        m_timer = shootTimer;
        startShootTime = 0;
        // Set motor to factory defaults
        m_stageMotor.configFactoryDefault();

        // Invert motor?
        m_stageMotor.setInverted(false);

        // Set motor to Brake
        m_stageMotor.setNeutralMode(NeutralMode.Brake);

        // Config current limit
        //m_stageMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 20, 0.10));

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
        hasStarted = true;
        SmartDashboard.putBoolean("hasStarted", hasStarted);
        SmartDashboard.putBoolean("hasRun", hasRun);
        // Default action is to hold the note in place if sensor detects note
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
    }

    public void runStage() {
        m_stageMotor.set(ControlMode.PercentOutput, StageConstants.kIntakeSpeed);
    }

    public void stopStage() {
        //System.out.println(m_timer.get());
        System.out.println(m_timer.giveTime());
        m_timer.timerState(false);
        m_timer.resetTimer();
        //this.startShootTime = 0;
        this.hasRun = false;
        m_stageMotor.set(ControlMode.PercentOutput, 0.0);
    }

    // Do not use if the shooter's target velocity is zero.
    public void ejectFront(double speed) {
        //System.out.println("CHECKING FOR TIME");
        if (this.hasStarted && !this.hasRun) {
            //this.startShootTime = System.currentTimeMillis();
            //m_timer.start();
            m_timer.timerState(true);

            System.out.println("STARTING TIMER");
            System.out.println(m_timer.giveTime());
            this.hasRun = true;
        } 
        if (m_noteInStage) {
            this.runStage(speed);
        }
    }

    public void ejectBack(double speed) {
        if (m_noteInStage) {
            this.runStage((-1.0) * speed);
        }
    }

    public boolean isNoteInStage() {
        return m_noteInStage;
    }

    /*
     * Command Factories
     */

    // To Intake a Note, drive the Stage until the sensor says we have a Note
    public Command intakeNoteCommand() {
        return new RunCommand(()-> this.runStage(StageConstants.kIntakeSpeed), this)
            .until(()->this.isNoteInStage())
            .andThen(()->this.stopStage());
    }
    
    // Pass the Note to the Shooter
    public Command feedNote2ShooterCommand() {        
        if (true) {
            return new RunCommand(() -> this.ejectFront(StageConstants.kFeedToShooterSpeed), this)
                .withTimeout(StageConstants.kFeedToShooterTime)
                .andThen(()->this.stopStage());
        }
        return new RunCommand(()->this.stopStage());
    }

    public double getTimeOfShot() {
        System.out.println("BBBBBBBBBBBBB");
        System.out.printf("%.3f",m_timer.giveTime());
        return Constants.ShooterConstants.timeToShoot - m_timer.giveTime();
    }

    // Feed the Note to the Amp
    public Command feedNote2AmpCommand() {
        return new RunCommand(() -> this.ejectFront(StageConstants.kFeedToAmpSpeed), this)
            .withTimeout(StageConstants.kFeedToAmpTime)
            .andThen(()->this.stopStage());
    }

    // Feed the Note backwards to the Amp
    public Command ejectNote2AmpCommand() {
        return new RunCommand(() -> this.ejectBack(StageConstants.kFeedToAmpSpeed), this)
            .withTimeout(StageConstants.kFeedToAmpTime)
            .andThen(()->this.stopStage());
    }

    // Feed the Note to the Trap
    public Command feedNote2TrapCommand() {
        return new RunCommand(() -> this.ejectFront(StageConstants.kFeedToTrapSpeed), this)
            .withTimeout(StageConstants.kFeedToTrapTime)
            .andThen(()->this.stopStage());
    }

    // Command to just stop the Stage
    public Command stopStageCommand() {
        
        return new InstantCommand(() -> this.stopStage());
    }

}
