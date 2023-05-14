// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.Encoder;
// import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class SwerveModule extends SubsystemBase {
    public static class SwivelControl {
        private double m_allowedError;
        private double m_speed;
        private double m_speedExponent;
        private double m_angleForMaxSpeed;
        public SwivelControl(double allowedError, double speed, double speedExponent, double angleForMaxSpeed){
            m_allowedError = allowedError;
            m_speed = speed;
            m_speedExponent = speedExponent;
            m_angleForMaxSpeed = angleForMaxSpeed;
        }
        public double getAllowedError(){
            return m_allowedError;
        }
        public double getSpeed(){
            return m_speed;
        }
        public double getSpeedExponent(){
            return m_speedExponent;
        }
        public double getAngleForMaxSpeed(){
            return m_angleForMaxSpeed;
        }
    }
    public static final int ENCODER_TICKS_PER_REVOLUTION = 1024;
    private final String name;
    private TalonSRXMotorController driveMotor;
    private TalonSRXMotorController swivelMotor;
    private SwivelControl m_swivelControl;
    private boolean reverse;
    private double zeroAngle;
    private double driveMultiplier;
    private Translation2d location;

    public SwerveModule(String name, TalonSRXMotorController driveMotor, TalonSRXMotorController swivelMotor,
                        SwivelControl swivelControl, double zeroAngle, double driveMultiplier, Translation2d location) {
        this.name = name;
        this.driveMotor = driveMotor;
        this.swivelMotor = swivelMotor;
        this.m_swivelControl = swivelControl;
        this.zeroAngle = zeroAngle;
        this.driveMultiplier = driveMultiplier;
        this.location = location;
    }

    //////////////////////
    // MUTATOR METHODS //
    ////////////////////

    public void swivelTowardAngle(double targetAngle) {
        targetAngle = simplifyDegrees(targetAngle);
        double currentAngle = getEncoderAngleOfSwivelMotor();
        double angleDiff = optimizeDegrees180(targetAngle - currentAngle);

        if (Math.abs(angleDiff) >= m_swivelControl.getAllowedError()) {
            double speed = m_swivelControl.getSpeed() * Math.pow(Math.abs(angleDiff / m_swivelControl.getAngleForMaxSpeed()), 1.0 /*m_swivelControl.getSpeedExponent()*/) * Math.signum(angleDiff);
            swivelMotor.setPercentSpeed(speed);
        } else {
            swivelMotor.setPercentSpeed(0);
        }
    }

    public void driveAtPercentSpeed(double speed) {
        if (reverse) {
            speed *= -1;
        }
        driveMotor.setPercentSpeed(speed * driveMultiplier);
    }

    public void stop() {
        driveAtPercentSpeed(0.0);
    }

    public void resetModule() {
        System.out.print(name + ": reset zeroAngle from " + zeroAngle + " to ");
        zeroAngle -= getEncoderAngleOfSwivelMotor();
        System.out.println(zeroAngle);
        reverse = false;
    }

    public void toggleReverseDrive() {
        reverse = !reverse;
    }

    ///////////////////////
    // ACCESSOR METHODS //
    /////////////////////

    public double getEncoderAngleOfSwivelMotor() {
        double degrees = ((swivelMotor.getEncoderPosition()) / (double) ENCODER_TICKS_PER_REVOLUTION) * 360;
        return zeroAngle + degrees;
    }

    public Translation2d getModuleLocation() {
        return location;
    }

    public TalonSRXMotorController getSwivelMotor(){
        return swivelMotor;
    }
    public TalonSRXMotorController getDriveMotor() {
        return driveMotor;
    }
    public String getName(){
        return name;
    }

    //////////////////////
    // UTILITY METHODS //
    ////////////////////

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
