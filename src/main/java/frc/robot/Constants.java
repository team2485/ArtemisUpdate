// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double NOMINAL_VOLTAGE = 12;
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kControllerDeadband = .08;
  }
  public static final class Feeder {
    public static final int SPARK_PORT = 33;
    public static final int MAX_CURRENT = 80; //keep this
    public static final int SPARK_FEEDER_MAX_STALL_CURRENT = 60; //keep this

    public static final double GEAR_RATIO = 1.0 / 3;

    public static final double RADIUS = 1.4 / 2;

    public static final double DISTANCE_PER_REVOLUTION = GEAR_RATIO * 2 * Math.PI * RADIUS;

    public static final String VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "feederSpark";

    public static final double OUTTAKE_PWM = 0.5;
    public static final double INTAKE_PWM = -1;

    public static final double FEEDER_MAX_VELOCITY = 1000;
    public static final double FEEDER_MIN_VELOCITY = -1000;

    public static final String TAB_NAME = "Shooter";
}
  public static final class Flywheels {
    public static final int SPARK_FLYWHEEL_LEFT_PORT = 30;
    public static final int SPARK_FLYWHEEL_LEFT_MAX_CURRENT = 80;

    public static final int SPARK_FLYWHEEL_RIGHT_PORT = 31;
    public static final int SPARK_FLYWHEEL_RIGHT_MAX_CURRENT = 80;

    public static final double ARC_ADJUST = 5;

    public static final double RPM_CONVERSION_FACTOR = 0.10472;

    public static final double FLYWHEEL_ENERGY_LOSS_FACTOR = 0.9;

    public static final double FYWHEEL_OUTTAKE_PWM = 0.1;

    public static final double RPM_ADJUST = 0;

    public static final double GEAR_RATIO = 2/1;

    public static final double RPM_THRESHOLD = 75;

    public static final String FLYWHEELS_TAB_NAME = "Shooter";
    public static final String INDEXING_TAB_NAME = "Indexing";

    public static final String LEFT_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "flywheelsLeftVelocityController";
    public static final String RIGHT_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "flywheelsRightVelocityController";

    public static final double FLYWHEELS_MAX_VELOCITY = 5000;
    public static final double FLYWHEELS_MIN_VELOCITY = -5000;

    public static final double VELOCITY_ARM = 0.98; //temp
    public static final double VELOCITY_TRIGGER = 0.95; //temp

    //IR
    public static final int ENTRANCE_IR_PORT = 2;
    public static final int TRANSFER_IR_PORT = 0;
    public static final int EXIT_IR_PORT = 1;
    public static final int MAX_DEBOUNCE_TIME = 3;

}
  public static class DriveTrainConstants {
    public static final int kFrontLeftMotorPort = 1;
    public static final int kBackLeftMotorPort = 2;
    public static final int kFrontRightMotorPort = 13;
    public static final int kBackRightMotorPort = 12;
  }

  public static final class Magazine {

    public static final int SPARK_LOW_MAX_CURRENT = 80;
    public static final int SPARK_HIGH_MAX_CURRENT = 80;
    //replace ports with real values
    public static final int SPARK_LOW_PORT = 22;
    public static final int SPARK_HIGH_PORT = 23;

    public static final int ENTRANCE_IR_PORT = 2;
    public static final int TRANSFER_IR_PORT = 3;
    public static final int EXIT_IR_PORT = 4;

    public static final int HIGH_MAGAZINE_BALL_CAPACITY = 3;

    public static final double ROLLER_RADIUS = 1.3 / 2;

   public static final double HIGH_GEAR_RATIO = 1;


    public static final double LOW_GEAR_RATIO = 12.0/30;

    public static final double HIGH_DISTANCE_PER_REVOLUTION = HIGH_GEAR_RATIO * 2 * Math.PI * ROLLER_RADIUS;

    public static final double LOW_INTAKE_BY_ONE_POS = -7;
    public static final double HIGH_INDEX_BY_ONE_POS = -8.25;
    public static final double HIGH_INCREMENT_TELEOP = -5;
    public static final double PUSH_IN_INCREMENT = -6;

    //replace below with actual number
    public static final double LOW_BELT_INTAKE_PWM = -0.4;
    public static final double OUTTAKE_PWM = 0.2;
    public static final double NORMAL_BALL_INCREMENT_TIMEOUT = 1; //seconds

    public static final double HIGH_MAGAZINE_POSITION_CONTROLLER_THRESHOLD = 0.5;

    public static final double COUNTER_MAX_PERIOD = 0.01;
    public static final int SAMPLES_TO_AVERAGE = 40;

    public static final String TAB_NAME = "Magazine";

    public static final String HIGH_MAGAZINE_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "highMagazineVelocityController";
    public static final String HIGH_MAGAZINE_POSITION_CONTROLLER_CONFIGURABLE_LABEL = "highMagazinePositionController";

    public static final String LOW_MAGAZINE_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "lowMagazineVelocityController";

    public static final double MAGAZINE_MAX_VELOCITY = 120;
    public static final double MAGAZINE_MIN_VELOCITY = -120;

    public static final double LOW_INTAKE_VELOCITY = -70;

    

    // public static final int MAX_DEBOUNCE_TIME = 3;
  }

  public static final class TurretConstants {

    public static final boolean TUNING_MODE = false;

    public static final int TALON_PORT = 25;

    public static final double AUTO_TURRET_MANUAL_ADJUST = 10;

    /**
     * counts per revolution of the encoder
     */
    public static final int ENCODER_CPR = 4096;

    public static final double TURRET_SPEED = 360.0 * 0.02; // degrees per ~20 milliseconds

    public static final double MIN_POSITION = -135; // degrees
    public static final double MAX_POSITION = 158; // degrees

    public static final double MAX_VELOCITY = 90; // degrees / second
    public static final double MIN_VELOCITY = -90; // degrees / second

    /**
     * In manual mode the max pwm will linearly clamp starting at the buffer zone size before the min or max positions.
     */
    public static final double BUFFER_ZONE_SIZE = 90; // degrees


    public static final double TURRET_PID_TOLERANCE = 1; // degrees

    public static final String TAB_NAME = "Turret";
    public static final String POSITION_CONTROLLER_CONFIGURABLE_LABEL = "turretPositionController";
    public static final String VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "turretVelocityController";
    public static final String ENCODER_OFFSET_CONFIGURABLE_LABEL = "turretEncoderOffset";
    public static final String ZERO_TURRET_LABEL = "Zero Turret";

    public static final double MANUAL_ANGLE_SCALE = 150;

    public static final int CURRENT_PIPELINE = 6;

    public static final double FINE_VELOCITY = 20;
  }

  public static final class HoodConstants {
    public static final int SPARK_PORT = 32;
    public static final double SPARK_HOOD_MAX_CURRENT = 0;

    public static final int ENCODER_CPR = 1000 * 4; //4x encoding
    public static final double DISTANCE_PER_REVOLUTION = 360;

    public static final double HOOD_LEAD_SCREW_GEAR_RATIO = 1.0/5;

    // these are relative to the vertical axis
    public static final double HOOD_BOTTOM_POSITION_DEG = 10;
    public static final double HOOD_TOP_POSITION_DEG = 42;


    public static final String TAB_NAME = "Shooter";

    //both in radians relative to horizontal
    //check if these should be in degrees or radians
    public static final double MAX_THETA = Math.toRadians(HOOD_TOP_POSITION_DEG);
    public static final double MIN_THETA = Math.toRadians(HOOD_BOTTOM_POSITION_DEG);

    public static final double AUTO_HOOD_MANUAL_ADJUST = 0;

    public static final double MANUAL_ANGLE_SCALE = 20;

    public static final String HOOD_VELOCITY_CONTROLLER_CONFIGURABLE_LABEL = "hoodVelocityController";
    public static final String HOOD_POSITION_CONTROLLER_CONFIGURABLE_LABEL = "hoodPositionController";

    public static final double HOOD_MAX_VELOCITY = 2500;
    public static final double HOOD_MIN_VELOCITY = -2500;

    public static final double BUFFER_ZONE_SIZE = 6;

    public static final double HOOD_DEFAULT_INCREMENT = -10;
  }
}
