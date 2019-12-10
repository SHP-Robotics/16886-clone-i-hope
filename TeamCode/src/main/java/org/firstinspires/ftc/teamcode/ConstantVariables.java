package org.firstinspires.ftc.teamcode;

public class ConstantVariables {
    public static final double K_STONE_DETECTION_DISTANCE = 1.5;

    public static final double K_CLAW_SERVO_1_OPEN = 0.25;
    public static final double K_CLAW_SERVO_1_CLOSED = 0.52;
    public static final double K_CLAW_SERVO_1_FULL_OPEN = 0;
    public static final double K_CLAW_SERVO_1_FULL_CLOSED = 1;

    public static final double K_CLAW_SERVO_2_OPEN = 0.75;
    public static final double K_CLAW_SERVO_2_CLOSED = 0.48;
    public static final double K_CLAW_SERVO_2_FULL_OPEN = 1;
    public static final double K_CLAW_SERVO_2_FULL_CLOSED = 0;

    public static final int K_PPR_DRIVE = 1120;
    public static final double K_DRIVE_WHEEL_DIA = 4;
    public static final double K_DRIVE_DIA = 17;

    public static final double K_PPR_BASE_MOVER = 1120;

    public static final double K_DRIVE_WHEEL_CIRCUMFERENCE = K_DRIVE_WHEEL_DIA * Math.PI;
    public static final double K_PPIN_DRIVE = K_PPR_DRIVE / K_DRIVE_WHEEL_CIRCUMFERENCE;

    public static final double K_TURN_CIRCUMFERENCE = K_DRIVE_DIA * Math.PI;
    public static final double K_PPTURN_DRIVE = K_PPIN_DRIVE * K_TURN_CIRCUMFERENCE;
    public static final double K_PPDEG_DRIVE = K_PPTURN_DRIVE / 360;

    public static final double K_SLOW_POWER = 0.5;

    public static final double K_PPR_ARM = 1120;
    public static final double K_ARM_GEAR_DIA = 0.81;

    public static final double K_ARM_GEAR_CIRCUMFERENCE=K_ARM_GEAR_DIA*Math.PI;
    public static final double K_PPIN_ARM=K_PPR_ARM / K_ARM_GEAR_CIRCUMFERENCE;

    public static final double K_PPDEG_BASE_MOVER = K_PPR_BASE_MOVER/360;

    public static final double K_DRIVE_ERROR_P = 250; // higher = less sensitive
}