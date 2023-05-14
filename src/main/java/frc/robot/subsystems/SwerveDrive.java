// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vector2d;

public class SwerveDrive extends SubsystemBase{

    private final double WHEEL_DIAMETER = 4 / 12.0; // (feet)
    private double m_maxDrivePercentSpeed = .37; // (percent) Max speed for driving when using percent speed
    private double MAX_RPS = .225; // (rotations/sec) Max rotation speed when using percent speed to rotate drive train
    private double MAX_DRIVE_ROTATION_EXPONENT = .8; // The power to raise the chassis rotation speed to with lookVector driving at full drive speed
    private double MIN_DRIVE_ROTATION_EXPONENT = .5; // The power to reaise the chassis rotation speed to with lookVector driving while stationary
    private double INPUT_DEADZONE = .05; // (percent) Region at which any drive/rotation input is considered noise, and set to 0
    private double SWIVEL_SPEED = 1; // (percent) Max speed for swiveling module to target angle
    private double ALLOWED_ERROR = 0.6; // (degrees)

    private SwerveDriveKinematics kinematics;

    private SwerveModule[] modules;
    private AHRS navX;

    public SwerveDrive(AHRS navX, SwerveModule... modules) {
        this.modules = modules;
        this.navX = navX;
        Translation2d[] moduleLocations = new Translation2d[modules.length];
        for (int i = 0; i < modules.length; i++)
            moduleLocations[i] = modules[i].getModuleLocation();
        kinematics = new SwerveDriveKinematics(moduleLocations);
    }

    /////////////////////
    // PUBLIC METHODS //
    ///////////////////

    public double getMaxDrivePercentSpeed() {
        return m_maxDrivePercentSpeed;
    }

    public void driveFieldOrientedWithLookVector(Vector2d driveVector, Vector2d lookVector) {
        double targetAngle = optimizeDegrees180(180 - calculateAngleOfVector(lookVector) - 90);
        double omega = 0;
        if (!Double.isNaN(targetAngle)) {
            double angleDiff = optimizeDegrees180(targetAngle - navX.getYaw());
            if (Math.abs(angleDiff) > ALLOWED_ERROR) {
                double drivePercentSpeed = driveVector.magnitude();
                double exponent = (MAX_DRIVE_ROTATION_EXPONENT - MIN_DRIVE_ROTATION_EXPONENT) * drivePercentSpeed + MIN_DRIVE_ROTATION_EXPONENT;
                
                double linearSpeed = (angleDiff / 180);
                omega = Math.pow(Math.abs(linearSpeed), exponent) * Math.signum(linearSpeed);
            }
        }

        driveFieldOriented(driveVector, omega);
    }
    
    /**
     * Drive the chassis with a percent speed vector to drive in the coordinate plane of the field and
     * a percent speed rotation.
     * Invokes method to drive with percent speeds based on the speeds given and the current yaw of the chassis.
     * @param driveVector The speed vector, for x and y (positive y being outfield, postive x being right)
     * @param omega The rotation speed (percent of MAX_RPS)
     */
    public void driveFieldOriented(Vector2d driveVector, double omega) {
        double chassisYaw = navX.getYaw();
        driveVector.rotate(-chassisYaw);
        driveWithPercentSpeeds(driveVector.y, driveVector.x, omega);
    }

    /**
     * Drive the chassis at a percent speed forward/back and left/right,
     * as well as rotate at a given radians per second speed.
     * @param y Percent speed toward front of the chassis to drive at
     * @param x Percent speed toward right side of the chassis to drive at
     * @param omega Percent speed (percent of MAX_RPS rotations per second) at which to rotate the chassis
     */
    public void driveWithPercentSpeeds(double y, double x, double omega) {
        // Prepare speed and angle setpoint arrays
        double moduleDegrees[] = new double[modules.length];
        double moduleSpeeds[] = new double[modules.length];

        // Enforce deadzones
        y = addDeadzoneToInput(y);
        x = addDeadzoneToInput(x);
        omega = addDeadzoneToInput(omega);

        // Don't bother to change anything if there's no input.
        if (y != 0 || x != 0 || omega != 0) {
            // Convert rotation speed from a percent to rotations/sec then to radians/sec (WPI requires radians/sec)
            omega *= MAX_RPS; // To RPS
            omega *= 60; // To RPM
            omega = Units.rotationsPerMinuteToRadiansPerSecond(omega); // To rad/sec

            // Calculate SwerveModuleStates based on input
            ChassisSpeeds speeds;

            SwerveModuleState[] moduleStates;

            // Compensate for 90 degree discrepancy in the library's algorithm for angular setpoints from rotation
            if (Math.abs(omega) > 0) {
                speeds = new ChassisSpeeds(-x * m_maxDrivePercentSpeed, y * m_maxDrivePercentSpeed, omega);

                moduleStates = kinematics.toSwerveModuleStates(speeds);

                for (int i = 0; i < modules.length; i++)
                    moduleStates[i].angle = Rotation2d.fromDegrees(moduleStates[i].angle.getDegrees() + 90);
            } else {
                speeds = new ChassisSpeeds(-y * m_maxDrivePercentSpeed, -x * m_maxDrivePercentSpeed, omega);
                
                moduleStates = kinematics.toSwerveModuleStates(speeds);
            }
            // Optimize module's task of reaching the setpoint
            for (int i = 0; i < modules.length; i++)
                moduleStates[i] = SwerveModuleState.optimize(moduleStates[i], Rotation2d.fromDegrees(modules[i].getEncoderAngleOfSwivelMotor()));

            // Gather drive speeds and angular setpoints
            for (int i = 0; i < modules.length; i++) {
                moduleDegrees[i] = moduleStates[i].angle.getDegrees();
                moduleSpeeds[i] = moduleStates[i].speedMetersPerSecond;
            }
        } else {
            for (int i = 0; i < modules.length; i++) {
                moduleDegrees[i] = modules[i].getEncoderAngleOfSwivelMotor();
                moduleSpeeds[i] = 0;
            }
        }

        // Drive and increment the angular headings toward the setpoints for all modules.
        for (int i = 0; i < modules.length; i++) {
            modules[i].swivelTowardAngle(moduleDegrees[i]);
            modules[i].driveAtPercentSpeed(moduleSpeeds[i]);
        }
    }

    public void stop() {
        for (int i = 0 ; i < modules.length; i++) {
            modules[i].stop();
        }
    }

    /**
     * Reset the displacement (position) and yaw (rotation)
     * of the NavX
     */
    public void resetFieldOrientation() {
        navX.resetDisplacement();
        navX.zeroYaw();
        System.out.println("FIELD ORIENTATION RESET");
    }

    /**
     * Reset all modules (zero their swivel heading)
     */
    public void resetModules() {
        for (SwerveModule m : modules)
            m.resetModule();
        System.out.println("ALL MODULES RESET");
    }

    ////////////////////////
    // UTILITY FUNCTIONS //
    //////////////////////

    private double addDeadzoneToInput(double input) {
        if (Math.abs(input) <= INPUT_DEADZONE) {
            input = 0;
        }
        return input;
    }

    private double calculateAngleOfVector(Vector2d vector) {
        Vector2d baseVector = new Vector2d(1, 0);
        double angleRad = Math.acos(vector.dot(baseVector) / (vector.magnitude() * baseVector.magnitude()));
        if (Math.signum(vector.y) != 0) {
            angleRad *= Math.signum(vector.y);
        }
        return Math.toDegrees(angleRad);
    }

    @SuppressWarnings("unused")
    private void driveMotorAtFPS(TalonSRXMotorController motor, double fps) {
        double wheelCircumference = Math.PI * WHEEL_DIAMETER;
        double rps = fps / wheelCircumference;
        motor.setVelocityRPS(rps);
    }

    @SuppressWarnings("unused")
    private void driveMotorAtPercentSpeed(TalonSRXMotorController motor, double speed) {
        if (speed > m_maxDrivePercentSpeed) {
            System.out.println("WARN: SwerveDrive was told to drive above its set max speed.");
            speed = m_maxDrivePercentSpeed;
        }
        motor.setPercentSpeed(speed);
    }
    
    @SuppressWarnings("unused")
    private void swivelMotorTowardAngle(TalonSRXMotorController motor, double targetAngle) {
        targetAngle = optimizeDegrees(targetAngle);
        double currentAngle = getEncoderAngleOfSwivelMotor(motor);
        double angleDiff = targetAngle - currentAngle;

        if (Math.abs(angleDiff) >= ALLOWED_ERROR) {
            double speed = SWIVEL_SPEED * Math.pow(Math.abs(angleDiff / 180), .5) * Math.signum(angleDiff);
            motor.setPercentSpeed(speed);
            System.out.println(getEncoderAngleOfSwivelMotor(motor) + ", " + targetAngle + ", " + speed);
        } else {
            System.out.println("TARGET REACHED " + getEncoderAngleOfSwivelMotor(motor));
            motor.setPercentSpeed(0);
        }
    }
    
    private double optimizeDegrees(double deg) {
        if (Math.abs(deg) > 360) {
	        deg = deg % 360;
	    }
        if (Math.abs(deg) > 180) {
	        deg -= 360 * Math.signum(deg);
	    }
	    if (Math.abs(deg) > 360) {
	        deg = deg % 360;
	    }
	    
	    return deg;
    }

    public double getEncoderAngleOfSwivelMotor(TalonSRXMotorController motor) {
        double degrees = (motor.getEncoderPosition() / (double) SwerveModule.ENCODER_TICKS_PER_REVOLUTION) * 360;
        return optimizeDegrees(degrees);
    }

    public SwerveModule[] getSwerveModules() {
        return modules;
    }

    @SuppressWarnings("unused")
    private double clamp(double value, double min, double max) {
        if (value < min) {
            return min;
        } else if (value > max) {
            return max;
        } else {
            return value;
        }
    }

    /**
     * Takes an angle in degrees and ensures it is shorter than a magnitude of 180 degrees.
     * Used to ensure shortest path between SwerveModule current heading at setpoint.
     * EXAMPLE: 30 or 90 or 180 degrees remain the same, but 181 becomes -179 and 270 becomes -90.
     * Excess loops are also removed both before and after this calculation.
     * @param deg 
     * @return
     */
    private double optimizeDegrees180(double deg) {
        deg = simplifyDegrees(deg);
        if (Math.abs(deg) > 180) {
	        deg -= 360 * Math.signum(deg);
	    }
	    deg = simplifyDegrees(deg);
	    
	    return deg;
    }

    /**
     * Take any angle in degrees and remove excessive loops above 360
     * @param deg Angle (degrees) to simplify
     * @return Degrees from 0 to 360
     */
    private double simplifyDegrees(double deg) {
        if (Math.abs(deg) > 360) {
	        deg = deg % 360;
	    }
	    
	    return deg;
    }
}
