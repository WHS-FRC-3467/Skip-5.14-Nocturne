package frc.robot.Subsystems.Drivetrain;

import java.util.Optional;
import java.util.function.Supplier;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Util.FieldCentricAiming;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();    
    public Field2d _field = new Field2d();
    private Rotation2d velocityOffset = new Rotation2d(0);
    private Double correctedDist = 0.0;
    private FieldCentricAiming m_FieldCentricAiming = new FieldCentricAiming();

    private Optional<Rotation2d> overrideAngle = Optional.empty();
    private boolean atAngle = false;
    private boolean atFutureAngle = false;
    
    

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configNeutralMode(NeutralModeValue.Brake);
        setSwerveDriveCustomCurrentLimits(); // Setup the current Limits
        configurePathPlanner();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configNeutralMode(NeutralModeValue.Brake);
        setSwerveDriveCustomCurrentLimits(); // Setup the current Limits
        configurePathPlanner();
    }

    private void configurePathPlanner() {
        PPHolonomicDriveController.setRotationTargetOverride(() -> overrideAngle);
        
        /*
         * Calculate drivebase radius (in meters). For swerve drive, this is the
         * distance from the center of the robot to the furthest
         * module.
         */
        double driveBaseRadius = 0;
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
                new HolonomicPathFollowerConfig(new PIDConstants(5, 0, .08), new PIDConstants(5, 0, 0),
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

                //PathPlannerLogging.setLogActivePathCallback((poses) -> _field.getObject("path").setPoses(poses)); //Uncomment to see currently followed path
                

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
    public void periodic() {

        _field.setRobotPose(getState().Pose);
        SmartDashboard.putData("Robot Pose Field Map",_field);
        if (RobotConstants.kIsArmTuningMode) {
            SmartDashboard.putNumber("Distance to Speaker", m_FieldCentricAiming.getDistToSpeaker(getState().Pose.getTranslation()));            
        }
        if (RobotConstants.kIsAutoAimTuningMode) {
            SmartDashboard.putBoolean("Is within 3 deg of speaker", atAngle);
            SmartDashboard.putNumber("vel Offset drivertrain", getVelocityOffset().getDegrees());
            SmartDashboard.putBoolean("IsRotatingFast", isRotatingFast());
            SmartDashboard.putNumber("Rot Speed", getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
        }

        atAngle = (Math.abs(m_FieldCentricAiming.getAngleToSpeaker(getState().Pose.getTranslation())
                .minus(getState().Pose.getRotation()).getDegrees()) < Constants.RobotConstants.robotAtAngleTolerance);

        atFutureAngle = (Math.abs(velocityOffset
                .minus(getState().Pose.getRotation()).getDegrees()) < 5);

    }

    @Override
    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    //Gets current rotation from estimated pose
    public Rotation2d getRotation(){
        return getState().Pose.getRotation();
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getCurrentRobotChassisSpeeds(), getRotation());
    }

    public void setVelocityOffset(Rotation2d angle, double dist) {
        velocityOffset = angle;
        correctedDist = dist;
    }

    public Rotation2d getVelocityOffset() {
        return velocityOffset;
    }

    public double getCorrectedDistance() {
        return correctedDist;
    }

    public void setOverrideAngle(Rotation2d angle) {
        overrideAngle = Optional.ofNullable(angle);
    }
    /**
     * Checks if the drivetrain rotation is within tolerance to the speaker
     * @return whether rotation is within angle tolerance
     * @see Constants
     */
    public boolean isAtAngle() {
        return atAngle;
    }

    public boolean isAtFutureAngle() {
        return atFutureAngle;
    }

    public boolean isRotatingFast(){
        if (getCurrentRobotChassisSpeeds().omegaRadiansPerSecond > .2) {
            return true;
        } else {
            return false;
        }
    }

}