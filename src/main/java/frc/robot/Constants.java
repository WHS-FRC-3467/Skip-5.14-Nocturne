// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class CanConstants {
        public static final int k_INTAKE_LEFT_CAN_ID = 15;
        public static final int k_INTAKE_RIGHT_CAN_ID = 16;

    }

    public static class DIOConstants {
        public static final int k_INTAKE_BEAM_BREAK = 0;
        public static final int k_ARM_ENCODER_ID = 1;

    }

    public static class IntakeConstants {
        public static final double k_INTAKE_REV_SPEED = -0.8;
    }

    public static class ShooterConstants {

    }

    public static class ClimberConstants {

    }

    public static class ArmConstants {

        public static final double kSVolts = 0;
        public static final double kGVolts = 0;
        public static final double kVVoltSecondPerRad = 60;
        public static final double kAVoltSecondSquaredPerRad = 100;

        public static final double k_ARM_KP = 5;
        public static final double k_ARM_KI = 0;
        public static final double k_ARM_KD = 1;

        public static final double kMaxVelocityRadPerSecond = 200;
        public static final double kMaxAccelerationRadPerSecSquared = 200;

        public static final double k_ARM_ENCODER_OFFSET_RADIANS = 0;

    }
}
