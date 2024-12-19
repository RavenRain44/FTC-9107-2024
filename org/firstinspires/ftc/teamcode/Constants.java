package org.firstinspires.ftc.teamcode;

public class Constants {

    // Arm Motor Constants
    public static double MAX_ARM_POSITION = 0.96;
    public static double MIN_ARM_POSITION = 0;
    public static double MAX_EXTEND_POS = 1;
    public static double MIN_EXTEND_POS = 0;
    public static double EXTENSION_ERROR = 0.05;
    public static double MAX_ARM_POWER = 1;
    public static double INTAKE_SLIDE_POSITION = 0.3;
    public static double INTAKE_ARM_POSITION = 0.02;
    public static double MAX_VIPER_POWER = 1;
    public static double DEFAULT_SLIDE_POSITION = 0;
    public static double DEFAULT_ARM_POSITION = 0.255;
    public static double OUTTAKE_SLIDE_POSITION = 0.0;
    public static double OUTTAKE_ARM_POSITION = 1.0;
    public static double ARM_NINETY = 0.96;
    public static double ARM_ZERO = 0.255;

    // Claw Servo Constants
    public static double OPEN_POSITION = 0.9; // TODO Tweak this value
    public static double CLOSE_POSITION = 0.72;  // TODO Tweak this value
    public static double INTAKE_WRIST_ANGLE = 0.68; // between 0.0 and 1.0 // TODO Tweak this value
    public static double OUTTAKE_WRIST_ANGLE = 0.95; // between 0.0 and 1.0 0.95
    public static double WRIST_ZERO = 0.61;
    public static double WRIST_NINETY = 0.95;
    public static double CLAW_BRACE_LENGTH = 4; // inches

    // Intake Constants
    public static double HEIGHT = -9.5; // inches [-height from ground to pivot + height from ground to end of viper slide in intake pos]
    public static double VIPER_MIN_LENGTH = 16.5; // inches
    public static double VIPER_MAX_LENGTH = 34.4094; // inches

    // Camera Constants
    public static double HORIZONTAL_CAMERA_OFFSET = 6.25;
    public static double LATERAL_CAMERA_OFFSET = 8.0;

    public static double BASKETS_HORIZONTAL_ALIGN_OFFSET = -5.5;
    public static double BASKETS_LATERAL_ALIGN_OFFSET = 7.75;
    public static double BASKETS_HEADING_ALIGN_OFFSET = 45;

    public static double MAX_APRIL_TAG_ALIGN_DISTANCE = 10;
    public static double ALIGN_HORIZONTAL_POWER = 1;
    public static double ALIGN_LATERAL_POWER = 1;
    public static double ALIGN_HEADING_POWER = 1;

    public static double MAX_ALIGN_POWER = 0.3;


}
