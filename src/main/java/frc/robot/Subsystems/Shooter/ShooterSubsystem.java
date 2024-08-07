package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.Setpoints;
import frc.robot.Util.TunableNumber;
import frc.robot.sim.PhysicsSim;

public class ShooterSubsystem extends SubsystemBase {

    /* Hardware */
    TalonFX m_motorLeft = new TalonFX(CanConstants.ID_ShooterLeft);
    TalonFX m_motorRight = new TalonFX(CanConstants.ID_ShooterRight);

    /*
     * Gains for shooter tuning
     */
    // An error of 1 rotation per second results in 2V output
    private static TunableNumber m_kP = new TunableNumber("Shooter kP", 0.3);
    // An error of 1 rotation per second increases output by 0.5V every second
    private static TunableNumber m_kI = new TunableNumber("Shooter kI", 0.0);
    // A change of 1 rotation per second squared results in 0.01 volts output
    private static TunableNumber m_kD = new TunableNumber("Shooter kD", 0.0);
    // Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor
    // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts/RPS
    private static TunableNumber m_kV = new TunableNumber("Shooter kV", 0.125);

    // Shooter Velocity setpoints (in RPS)
    private static TunableNumber m_ShooterSetpointL = new TunableNumber("Shooter Setpoint L", 0.0);
    private static TunableNumber m_ShooterSetpointR = new TunableNumber("Shooter Setpoint R", 0.0);

    private static TunableNumber trap_speedL = new TunableNumber("Trap Speed L", 22);
    private static TunableNumber trap_speedR = new TunableNumber("Trap Speed R", 22);

    // Current Limits
    private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

    /*
     * Setup the Velocity PID control object Start at velocity 0, disable FOC, no feed forward, use slot 0
     */
    private final VelocityVoltage m_voltageVelocityLeft = new VelocityVoltage(0.0, 0.0, false, 0.0, 0, false, false, false);
    private final VelocityVoltage m_voltageVelocityRight = new VelocityVoltage(0.0, 0.0, false, 0.0, 0, false, false, false);

    /* Neutral output control for disabling the Shooter */
    private final NeutralOut m_brake = new NeutralOut();

    // Which side of the shooter?
    public enum kShooterSide {
        kLEFT,
        kRIGHT
    };

    // Shooter state
    public enum kShooterState {
        kOFF,
        kIDLE,
        kSPOOLING,
        kREADY
    }
    kShooterState m_state = kShooterState.kOFF;

    double m_idleSpeed = ShooterConstants.kShooterIdleSpeed;

    public ShooterSubsystem() {
        SmartDashboard.putData("POWER", new PowerDistribution());

        /* Configure the motors */
        var leadConfiguration = new TalonFXConfiguration();

        /* set motors to Coast */
        leadConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Configure the lead Talon to use a supply limit of 60 amps IF we exceed 80 amps for over 0.1 second
        m_currentLimits.SupplyCurrentLimit = 60; // Limit to 60 amps
        m_currentLimits.SupplyCurrentThreshold = 80; // If we exceed 80 amps
        m_currentLimits.SupplyTimeThreshold = 0.1; // For at least 0.1 second
        m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it

        m_currentLimits.StatorCurrentLimit = 70; // Limit stator to 70 amps
        m_currentLimits.StatorCurrentLimitEnable = true; // And enable it
        leadConfiguration.CurrentLimits = m_currentLimits;

        /* Config the peak outputs */
        leadConfiguration.Voltage.PeakForwardVoltage = 12.0;
        leadConfiguration.Voltage.PeakReverseVoltage = 0.0; // Don't go in reverse

        /* Update Shooter Gains from TunableNumbers */
        leadConfiguration.Slot0.kP = m_kP.get();
        leadConfiguration.Slot0.kI = m_kI.get();
        leadConfiguration.Slot0.kD = m_kD.get();
        leadConfiguration.Slot0.kV = m_kV.get();

        /* Apply configs */
        leadConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_motorLeft.getConfigurator().apply(leadConfiguration);
        leadConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_motorRight.getConfigurator().apply(leadConfiguration);    

    }

    public void simulationInit() {
        /* If running in Simulation, setup simulated Falcons */
        PhysicsSim.getInstance().addTalonFX(m_motorLeft, 0.001);
        PhysicsSim.getInstance().addTalonFX(m_motorRight, 0.001);
    }


    @Override
    public void periodic() {
 
        boolean wheelsAtSpeed = areWheelsAtSpeed();
        SmartDashboard.putBoolean("Shooter at Speed?", wheelsAtSpeed);

        if (RobotConstants.kIsShooterTuningMode) {
            // Put actual velocities to smart dashboard
            SmartDashboard.putNumber("Shooter Velocity L", getShooterVelocity(kShooterSide.kLEFT));
            SmartDashboard.putNumber("Shooter Velocity R", getShooterVelocity(kShooterSide.kRIGHT));
        }

        if (m_state == kShooterState.kSPOOLING && wheelsAtSpeed) {
            // If Shooter is spooling up, signal when it's at speed 
            m_state = kShooterState.kREADY;

        } else if (m_state == kShooterState.kREADY && !wheelsAtSpeed) {
            // If Shooter is ready, signal if it falls off the target speed 
            m_state = kShooterState.kSPOOLING;
        }
    }

    public void simulationPeriodic() {
        // If running in simulation, update the sims
        PhysicsSim.getInstance().run();
    }


    /**
     * Update Shooter Gains from TunableNumbers
     */
    public void updateGains() {
        var slot0 = new Slot0Configs();

        slot0.kP = m_kP.get();
        slot0.kI = m_kI.get();
        slot0.kD = m_kD.get();
        slot0.kV = m_kV.get();

        m_motorLeft.getConfigurator().apply(slot0);
        m_motorRight.getConfigurator().apply(slot0);
    }

    /**
     * @param targetVelocity the velocity in RPS of the shooter Right
     * @param targetVelocityL the velocity in RPS of the shooter Left
     */
    public void runShooter(double targetVelocityL, double targetVelocityR) {
        // Save Velocity setpoints
        m_ShooterSetpointL.set(targetVelocityL);
        m_ShooterSetpointR.set(targetVelocityR);
        m_motorLeft.setControl(m_voltageVelocityLeft.withVelocity(targetVelocityL));
        m_motorRight.setControl(m_voltageVelocityRight.withVelocity(targetVelocityR));
        if (m_state != kShooterState.kREADY) {
            m_state = kShooterState.kSPOOLING;
        }
    }

    public void runShooter() {
        // Get Velocity setpoint from TunableNumber
        m_motorLeft.setControl(m_voltageVelocityLeft.withVelocity(m_ShooterSetpointL.get()));
        m_motorRight.setControl(m_voltageVelocityRight.withVelocity(m_ShooterSetpointR.get()));
        if (m_state != kShooterState.kREADY) {
            m_state = kShooterState.kSPOOLING;
        }
    }

    public void runTrap() {
        // Get Velocity setpoint from TunableNumber
        m_motorLeft.setControl(m_voltageVelocityLeft.withVelocity(trap_speedL.get()));
        m_motorRight.setControl(m_voltageVelocityRight.withVelocity(trap_speedR.get()));
        if (m_state != kShooterState.kREADY) {
            m_state = kShooterState.kSPOOLING;
        }
    }

    public void stopShooter() {
        m_motorLeft.setControl(m_brake);
        m_motorRight.setControl(m_brake);
        m_state = kShooterState.kOFF;
    }

    public void runIdle() {
        m_ShooterSetpointL.set(m_idleSpeed);
        m_ShooterSetpointR.set(m_idleSpeed);
        m_motorLeft.setControl(m_voltageVelocityLeft.withVelocity(m_idleSpeed));
        m_motorRight.setControl(m_voltageVelocityRight.withVelocity(m_idleSpeed));
        m_state = kShooterState.kIDLE;
    }

    public void setIdleShooterSpeed(double speed) {
        m_idleSpeed = speed;
    }


    /**
     * @param setpoints - Reference to a Setpoints class instance
     */
    public void setShooterSetpoints(Setpoints setpoints) {

        m_ShooterSetpointL.set(setpoints.shooterLeft);
        m_ShooterSetpointR.set(setpoints.shooterRight);

    }

    /**
     * @param int side - the side of the shooter to query (0 = left, 1 = right)
     * @return the velocity of the specified shooter side in RPS
     */
    public double getShooterVelocity(kShooterSide side) {
        switch(side) {
        case kLEFT:
            return m_motorLeft.getVelocity().getValueAsDouble();
        case kRIGHT:
            return m_motorRight.getVelocity().getValueAsDouble();
        default:
            return 0.0;
        }
    }

    /**
     * Private method for subsystem use only. Outside callers should use isShooterAtSpeed().
     * @return true if the error of the shooter is within the tolerance
     */
    public boolean areWheelsAtSpeed() {
        double leftErr = Math.abs(m_ShooterSetpointL.get() - getShooterVelocity(kShooterSide.kLEFT));
        double rightErr = Math.abs(m_ShooterSetpointR.get() - getShooterVelocity(kShooterSide.kRIGHT));
        return ((leftErr + rightErr) / 2.0) < ShooterConstants.kShooterTolerance;

    }

    public boolean areAtMinSpeed() {
        double leftErr = Math.abs(20 - getShooterVelocity(kShooterSide.kLEFT));
        double rightErr = Math.abs(20 - getShooterVelocity(kShooterSide.kRIGHT));
        return ((leftErr + rightErr) / 2.0) < ShooterConstants.kShooterTolerance;

    }

    /**
     * Quicker method for outside callers (i.e. LEDSubsystem)
     * @return true if the error of the shooter is within the tolerance
     */
    public boolean isShooterAtSpeed () {
        return (m_state == kShooterState.kREADY);
    }

    /*
     * Command Factories
     */
    public Command runShooterCommand(double velocityL, double velocityR) {
        return new RunCommand(()->this.runShooter(velocityL, velocityR), this);
    }

    public Command runTrapShooterCommand() {
        return new RunCommand(()->this.runTrap(), this);
    }

    public Command runShooterCommand() {
        return new RunCommand(()->this.runShooter(), this);
    }

    public Command stopShooterCommand() {
        return new InstantCommand(()->this.stopShooter(), this);
    }

    public Command updateShooterGainsCommand() {
        return new InstantCommand(()->this.updateGains(), this);
    }
}