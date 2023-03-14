package frc.robot;

public class RobotMap {
    // Input USB IDs
    public static int driverJoy = 0;

    // Button IDs
    public static int launchButton = 6;

    // Motor Controller CAN IDs
    public static int driveMotorFrontRight = 2,
                      driveMotorFrontLeft = 4,
                      driveMotorBackLeft = 6,
                      driveMotorBackRight = 8,
                      swivelMotorFrontRight = 3,
                      swivelMotorFrontLeft = 5,
                      swivelMotorBackLeft = 7,
                      swivelMotorBackRight = 9;

    // PWM
    public static int ledStrip = 0;

    // PNEUMATICS
    public static int pcm = 1;
    public static int cannonForwardChannel = 0,
                      cannonReverseChannel = 1;
}
