package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Configurable
public class Constants {
    // Intake
    public static double INTAKE_MOTOR_POWER = 1;
    public static double REVERSE_INTAKE_MOTOR_POWER = 1;
    public static double OUT_TAKE_TIME = 0.1;
    public static double FLOWER_RIGHT_SERVO_DOWN = 0.37;
    public static double FLOWER_RIGHT_SERVO_UP = 0.74;
    public static double FLOWER_LEFT_SERVO_DOWN = 0.37 ;
    public static double FLOWER_LEFT_SERVO_UP = 0.64;

    // Scoring General
    public static double POLLEN_GATE_OPEN_POSITION = 0.38;
    public static double POLLEN_GATE_CLOSE_POSITION = 0.1;
    public static double ALIGN_IMU_TIME = 0.6;
    public static double ALIGN_APRIL_TAG_TIME = 0.3;
    public static double SHOOTING_TIME = 2.25;

    public static double POLLEN_SERVO_KP = 0.003, POLLEN_SERVO_KI = 0, POLLEN_SERVO_KD = 0.000000004, POLLEN_SERVO_KF = 0.06;
    // Pollen turret AprilTag alignment (copy the kp/ki/kd/kf/reverse you tuned in CameraTest)
    public static boolean POLLEN_TURRET_TAG_REVERSED = false;
    public static double POLLEN_MOTOR_KP = 0.5, POLLEN_MOTOR_KI = 0.0001, POLLEN_MOTOR_KD = 0.005, POLLEN_MOTOR_KF = 0.005;

    public static double[] POLLEN_TURRET_SERVO_CONTROLLER = new double[]{POLLEN_SERVO_KP, POLLEN_SERVO_KI, POLLEN_SERVO_KD, POLLEN_SERVO_KF};
    public static double[] POLLEN_TURRET_MOTOR_CONTROLLER = new double[]{POLLEN_MOTOR_KP, POLLEN_MOTOR_KI, POLLEN_MOTOR_KD, POLLEN_MOTOR_KF};

    // Autonomous
    public static double PARK_TIME = 5.0;
    public static double SHOOTER_SPIN_UP_TIME = 0.8;
    public static double AUTO_INTAKE_TIME = 1.0;
    public static double APRIL_TAG_ALIGN_DETECTION_TIME = 1.0;
}