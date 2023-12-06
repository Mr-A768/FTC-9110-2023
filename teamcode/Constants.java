package org.firstinspires.ftc.teamcode;

import org.opencv.core.Scalar;

public class Constants {

    /* DRIVE CONSTANTS */
    public static final double DRIVE_GRAB_SPEED = .75;
    public static final double DRIVE_DEFAULT_SPEED = 1.0;

    /* ARM CONSTANTS */
    public static final double ARM_PIVOT_HEIGHT = 7.33;
    public static final double ARM_PIVOT_DIST_FROM_FRONT = 10.8;

    public static final double SLIDE_MAX_LENGTH = 33.307;
    public static final double SLIDE_MIN_LENGTH = 14.173;
    public static final double SLIDE_MAX_TICK = 2200;
    public static final double SLIDE_MIN_TICK = 0;
    public static final double SLIDE_MAX_SPEED = 1.0;
    public static final double SLIDE_P = 0;
    public static final double SLIDE_I = 0.0;
    public static final double SLIDE_D = 0.0;
    public static final double SLIDE_F = 0.0;
    public static final double SLIDE_POS = 3.5;
    public static final double ARM_GEAR_RATIO = .80;
    public static final double ARM_MAX_ANGULAR_SPEED = .35;
    public static final double ARM_LOW_ANGULAR_SPEED = .15;
    public static final double ARM_MIN_ANGLE = Math.toRadians(-34.18);
    public static final double ARM_MAX_ANGLE = Math.toRadians(37);
    public static final double ARM_SCOREMODE_ANGLE = Math.toRadians(0);
    public static final double ARM_DRIVEMODE_ANGLE = Math.toRadians(-32);
    public static final double ARM_DRIVEMODE_STACK_ANGLE = Math.toRadians(-26.5);
    public static final double ARM_INIT_TICK = -760;

    /* CLAW CONSTANTS */
    public static final double CLAW_MAX = 0.8;
    public static final double CLAW_MIN = 0.62;
    public static final double WRIST_MAX = 0.8;
    public static final double WRIST_MIN = 0.14;

    /* VISION CONSTANTS */
    /* BLUE */
    public static final Scalar lowerB = new Scalar((206 - 15) / 2, 64, 20); // HSV threshold bounds
    public static final Scalar upperB = new Scalar((206 + 20) / 2, 255, 255);
    /* RED */
    public static final Scalar lowerR = new Scalar((180 - 55) / 2, 64, 20); // HSV threshold bounds
    public static final Scalar upperR = new Scalar((180 + 80) / 2, 255, 255);
}
