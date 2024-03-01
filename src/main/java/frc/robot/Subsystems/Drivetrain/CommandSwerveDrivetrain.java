package frc.robot.Subsystems.Drivetrain;

import java.util.function.Supplier;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Vision.PhotonVision;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private Alliance _alliance;
    private Pose2d _speakerPosition;
    public Field2d _field = new Field2d();
    public PhotonVision _vision = new PhotonVision();
    private Rotation2d velocityOffset = new Rotation2d(0);
    private Double correctedDist = 0.0;
    


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configNeutralMode(NeutralModeValue.Brake);
        setSwerveDriveCustomCurrentLimits();        // Setup the current Limits
        configurePathPlanner();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configNeutralMode(NeutralModeValue.Brake);
        setSwerveDriveCustomCurrentLimits();        // Setup the current Limits
        configurePathPlanner();
    }

    private void configurePathPlanner() {

        /*
         * Calculate drivebase radius (in meters). For swerve drive, this is the
         * distance from the center of the robot to the furthest
         * module.
         */
        double driveBaseRadius = .74;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        /*
         * Configure the PathPlanner AutoBuilder
         */
        AutoBuilder.configureHolonomic(

                // Supplier of current robot pose
                () -> this.getState().Pose,

                // Consumer for seeding pose against auto
                this::seedFieldRelative,

                // A supplier for the robot's current robot relative chassis speeds
                this::getCurrentRobotChassisSpeeds,

                // A consumer for setting the robot's robot-relative chassis speeds
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),

                // Method for configuring the path following commands
                new HolonomicPathFollowerConfig(new PIDConstants(.6, 0, .08), new PIDConstants(5, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius, new ReplanningConfig()),

                // Boolean supplier that controls when the path will be mirrored for the red
                // alliance
                // This will flip the path being followed to the red side of the field during
                // auto only.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red & !DriverStation.isTeleop();
                    } else
                        return false;
                },
                this); // Subsystem for requirements
    }

    //Start
    /**
     * Set custom current limits for the swerve drive motors.
     * The constructor already sets the stator current limit, but this method sets the supply current limit.
     * <p>This should be done in the constructor, since the {@link com.ctre.phoenix6.configs.TalonFXConfigurator#refresh()} and {@link com.ctre.phoenix6.configs.TalonFXConfigurator#apply()} are blocking API calls.
     */
    public void setSwerveDriveCustomCurrentLimits() {
        //Create a current configuration to use for the drive motor of each swerve module.
        var customCurrentLimitConfigs = new CurrentLimitsConfigs();

        //Iterate through each module.
        for (var module : Modules) {
            //Get the Configurator for the current drive motor.
            var currentConfigurator = module.getDriveMotor().getConfigurator();

            //Refresh the current configuration, since the stator current limit has already been set.
            currentConfigurator.refresh(customCurrentLimitConfigs);

            //Set all of the parameters related to the supply current.  The values should come from Constants.
            customCurrentLimitConfigs.SupplyCurrentLimit = SwerveConstants.kSwerveDriveSupplyCurrentLimit;
            customCurrentLimitConfigs.SupplyCurrentLimitEnable = SwerveConstants.kSwerveDriveSupplyCurrentLimitEnable;
            customCurrentLimitConfigs.SupplyCurrentThreshold = SwerveConstants.kSwerveDriveSupplyCurrentThreshold;
            customCurrentLimitConfigs.SupplyTimeThreshold = SwerveConstants.kSwerveDriveSupplyTimeThreshold;

            //Apply the new current limit configuration.
            currentConfigurator.apply(customCurrentLimitConfigs);
        }
    }
    //End

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    @Override
    public void periodic(){
        
        _field.setRobotPose(m_odometry.getEstimatedPosition());
        SmartDashboard.putData("Field Test",_field);
        SmartDashboard.putNumber("Distance2Speaker", calcDistToSpeaker());
        SmartDashboard.putNumber("GetVelOffsetDeg", getVelocityOffset().getDegrees());
        
        
        var visionEst = _vision.getEstimatedGlobalPose();
        visionEst.ifPresent(
                est -> {
                    var estPose = est.estimatedPose.toPose2d();
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = _vision.getEstimationStdDevs(estPose);
                    //System.out.println("Adding to vision");

                    this.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });
        
        



    }
    @Override
    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    //Gets current rotation in from Pidgeon 
    public Rotation2d getGyroscopeRotation(){
        return m_odometry.getEstimatedPosition().getRotation();
    }

    // Gets current alliance from driverstation to know which speaker to point at
    public Alliance getAlliance() {
        if (_alliance == null) {
            if (DriverStation.getAlliance().isPresent()) {
                _alliance = DriverStation.getAlliance().get();
            }
        }
        return _alliance;
    }

    //Gets coordinates for appriopriate speaker
    private Pose2d getSpeakerPos() {
        if (_speakerPosition == null) {
            if (getAlliance() != null) {
                _speakerPosition = (getAlliance() == DriverStation.Alliance.Blue) ? Constants.BLUE_SPEAKER
                        : Constants.RED_SPEAKER;
            }
        }

        return _speakerPosition;
    }

    // this setup lets us test the math, but when we actually run the code we don't
    // have to give a pose estimator
    public static double getRadiusToSpeakerInMeters(Pose2d robotPose, Pose2d speakerPos) {

        if (speakerPos == null) return 0;
        
        double xDiff = robotPose.getX() - speakerPos.getX();
        double yDiff = robotPose.getY() - speakerPos.getY();
        double xPow = Math.pow(xDiff, 2);
        double yPow = Math.pow(yDiff, 2);
        // Use pythagorean thm to find hypotenuse, which is our radius
        return Math.sqrt(xPow + yPow);
    }

    
    public double calcAngleToSpeaker() {
        if (getAlliance() == Alliance.Blue) {
            return calcAngleToSpeakerForBlue();
        } else {
            return calcAngleToSpeakerForRed();
        }
    }

    public double calcAngleToSpeaker(Translation2d pose) {
        if (getAlliance() == Alliance.Blue) {
            return calcAngleToSpeakerForBlue(pose);
        } else {
            return calcAngleToSpeakerForRed(pose);
        }
    }

    public Rotation2d RotToSpeaker() {
        return Rotation2d.fromDegrees(calcAngleToSpeaker());
    }

    private double calcAngleToSpeakerForBlue() {
        Pose2d robotPose = m_odometry.getEstimatedPosition();
        Pose2d speakerPos = Constants.BLUE_SPEAKER;
        double xDiff = robotPose.getX() - speakerPos.getX();
        double yDiff = speakerPos.getY() - robotPose.getY();
        //System.out.print(xDiff);
        //System.out.print(yDiff);
        //System.out.println(180 - Math.toDegrees(Math.atan(yDiff / xDiff)));
        return 180 - Math.toDegrees(Math.atan(yDiff / xDiff));
    }



    private double calcAngleToSpeakerForRed() {
        Pose2d robotPose = m_odometry.getEstimatedPosition();
        Pose2d speakerPos = Constants.RED_SPEAKER;
        double xDiff = speakerPos.getX() - robotPose.getX();
        double yDiff = speakerPos.getY() - robotPose.getY();
        //System.out.print(xDiff);
        //System.out.print(yDiff);
        //System.out.println(Math.toDegrees(Math.atan(yDiff / xDiff)));
        return Math.toDegrees(Math.atan(yDiff / xDiff));
    }



    public double calcDistToSpeaker() {
        if(getSpeakerPos()!=null) {
            return getRadiusToSpeakerInMeters(m_odometry.getEstimatedPosition(),getSpeakerPos());
        } else {
            return 999;
        }
        
    }

    public double calcDistToSpeaker(Translation2d pose) {
        if(getSpeakerPos()!=null) {
            return getRadiusToSpeakerInMeters(new Pose2d(pose, m_odometry.getEstimatedPosition().getRotation()),getSpeakerPos());
        } else {
            return 999;
        }
        
    }

    

    private double calcAngleToSpeakerForRed(Translation2d pose) {
        Pose2d speakerPos = Constants.RED_SPEAKER;
        double xDiff = speakerPos.getX() - pose.getX();
        double yDiff = speakerPos.getY() - pose.getY();
        // System.out.print(xDiff);
        // System.out.print(yDiff);
        // System.out.println(Math.toDegrees(Math.atan(yDiff / xDiff)));
        return Math.toDegrees(Math.atan(yDiff / xDiff));
    }

    private double calcAngleToSpeakerForBlue(Translation2d pose) {
        Pose2d speakerPos = Constants.BLUE_SPEAKER;
        double xDiff = pose.getX() - speakerPos.getX();
        double yDiff = speakerPos.getY() - pose.getY();
        // System.out.print(xDiff);
        // System.out.print(yDiff);
        // System.out.println(180 - Math.toDegrees(Math.atan(yDiff / xDiff)));
        return 180 - Math.toDegrees(Math.atan(yDiff / xDiff));
    }
    
    
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return new ChassisSpeeds(
                getCurrentRobotChassisSpeeds().vxMetersPerSecond * this.getState().Pose.getRotation().getCos()
                        - getCurrentRobotChassisSpeeds().vyMetersPerSecond * this.getState().Pose.getRotation().getSin(),
                getCurrentRobotChassisSpeeds().vyMetersPerSecond * this.getState().Pose.getRotation().getCos()
                        + getCurrentRobotChassisSpeeds().vxMetersPerSecond * this.getState().Pose.getRotation().getSin(),
                getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
    }

    public void setVelOffset(Rotation2d angle, double dist) {
        velocityOffset = angle;
        correctedDist = dist;
    }

    public Rotation2d getVelocityOffset() {
        return velocityOffset;
    }

    public double getCorrectedDistance() {
        return correctedDist;
    }



    /*
     * SysID robot drive characterization routines
     */
    //private SwerveVoltageRequest driveVoltageRequest = new SwerveVoltageRequest(true);
    
/* 
    private SysIdRoutine m_driveSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))), null,
                    this));

    private SwerveVoltageRequest steerVoltageRequest = new SwerveVoltageRequest(false);

    private SysIdRoutine m_steerSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> setControl(steerVoltageRequest.withVoltage(volts.in(Volts))), null,
                    this));

    private SysIdRoutine m_slipSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.25).per(Seconds.of(1)), null, null, ModifiedSignalLogger.logState()),
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))), null,
                    this));

    public Command runDriveQuasiTest(Direction direction) {
        return m_driveSysIdRoutine.quasistatic(direction);
    }

    public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.dynamic(direction);
    }

    public Command runSteerQuasiTest(Direction direction) {
        return m_steerSysIdRoutine.quasistatic(direction);
    }

    public Command runSteerDynamTest(SysIdRoutine.Direction direction) {
        return m_steerSysIdRoutine.dynamic(direction);
    }

    public Command runDriveSlipTest() {
        return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }
}
 */

 private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

    /* Use one of these sysidroutines for your particular test */
    private SysIdRoutine SysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(TranslationCharacterization.withVolts(volts)),
                    null,
                    this));

    private final SysIdRoutine SysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(RotationCharacterization.withVolts(volts)),
                    null,
                    this));
    private final SysIdRoutine SysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(7),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(SteerCharacterization.withVolts(volts)),
                    null,
                    this));

    /* Change this to the sysid routine you want to test */
    private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;


    /*
     * Both the sysid commands are specific to one particular sysid routine, change
     * which one you're trying to characterize
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return RoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return RoutineToApply.dynamic(direction);
    }

}