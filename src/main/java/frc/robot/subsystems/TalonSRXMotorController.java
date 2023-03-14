package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonSRXMotorController {
	public enum TalonEncoderType {
		MAG_RELATIVE,
		MAG_ABSOLUTE,
		QUAD,
		ANALOG;
	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private TalonSRX controller;
	private final int standardTimeoutMs = 10;
	private int ticksPerShaftRotation = 1024;
	private final double rpmToClicksPer100ms =  ticksPerShaftRotation/1.0 * 1.0/60.0 * 1.0/10.0;// rev/min = 4096 clicks/rev * 1min/60s * 1s/10 centi seconds
	private final double rpsToClicksPer100ms = rpmToClicksPer100ms*60;
	private final double actualOverRequestedRPM =  0.85589;
	
	/***
	 * 
	 * @param The id of the motor controller
	 */
	public TalonSRXMotorController(int controllerId) {
		controller = new TalonSRX(controllerId);
		//Tell the motors to brake if there is no voltage being applied to them
		setBrakeEnabled(true);	
		configPID(0.01, 0.0, 0.1, 1);
	}
	
	/**
	 * Sets the encoder you want to use for the motor and resets the encoder to zero
	 * @param encoderType The type of encoder to use
	 * @param ticksPerRevolution How many encoder ticks in one shaft rotation of the motor
	 */
	public void configureEncoder(TalonEncoderType encoderType, int ticksPerRevolution) {
		ticksPerShaftRotation = ticksPerRevolution;

		if (encoderType == TalonEncoderType.MAG_RELATIVE) {
			controller.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, standardTimeoutMs);
		}else if (encoderType == TalonEncoderType.MAG_ABSOLUTE) {
			controller.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, standardTimeoutMs);
		}else if (encoderType == TalonEncoderType.QUAD) {
			controller.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, standardTimeoutMs);
		}else if (encoderType == TalonEncoderType.ANALOG) {
			controller.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, standardTimeoutMs);
		}

		//Ensure each encoder starts at zero
		zeroEncoder();
	}

	/***
	 * 
	 * @param brake if true set brake mode on. if false set coast mode on.
	 * control is what happens when power is removed from the motor.
	 */
	public void setBrakeEnabled (Boolean brake) {
		if (brake) {
			controller.setNeutralMode(NeutralMode.Brake);
		}
		else{
			controller.setNeutralMode(NeutralMode.Coast);
		}
	}
	
	/**
	 * Enables the forward limit switch so that it will stop the motor from moving forward when triggered
	 * @param enable Whether or not the enable or disable the limit switch
	 * @param normallyOpen If true then the limit switch will be considered not triggered until it is pressed
	 */
	public void enableForwardLimitSwitch(boolean enable, boolean normallyOpen) {
		LimitSwitchNormal normOpen = LimitSwitchNormal.NormallyOpen;

		if (!normallyOpen) {
			normOpen = LimitSwitchNormal.NormallyClosed;
		}

		//Tell the controller where the limit switch is plugged in
		controller.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, normOpen, standardTimeoutMs);
		controller.overrideLimitSwitchesEnable(!enable);//Enable switch
	}
	
	/**
	 * Enables the reverse limit switch so that it will stop the motor from moving backward when triggered
	 * @param enable Whether or not the enable or disable the limit switch
	 * @param normallyOpen If true then the limit switch will be considered not triggered until it is pressed
	 */
	public void enableReverseLimitSwitch(boolean enable, boolean normallyOpen) {
		LimitSwitchNormal normOpen = LimitSwitchNormal.NormallyOpen;

		if (!normallyOpen) {
			normOpen = LimitSwitchNormal.NormallyClosed;
		}

		//Tell the controller where the limit switch is plugged in
		controller.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, normOpen, standardTimeoutMs);
		controller.overrideLimitSwitchesEnable(!enable);//Enable switch
	}
	
	/**
	 * Set how long it should take for the motors to accelerate to the desired speed
	 * @param secondsToAccelerate How many seconds it should take to accelerate
	 */
	public void setMaxAccelerationTime(double secondsToAccelerate) {
		controller.configClosedloopRamp(secondsToAccelerate,  10);
	}
	
	/**
	 * Gets the Id of the motor controller
	 * @return The id of the motor controller
	 */
	public int getControllerId() {
		return controller.getDeviceID();
	}
	
	/**
	 * Sets which motor controller to follow, this will the controller this is being called on mirror the output of the leader controller
	 * @param leader The motor controller to follow
	 * @param invertOutput If true then when the leader motor spins forward, this motor will spin backward
	 */
	public void follow(TalonSRXMotorController leader, boolean invertOutput) {
		controller.set(ControlMode.Follower, leader.getControllerId());
		mirrorLeaderOutput(!invertOutput);
	}
	
	/***
	 * Set the speed as a percentage
	 * @param percent any percentage from -1.0 to +1.0
	 */
	public void setPercentSpeed(double percent) {
		controller.set(ControlMode.PercentOutput, percent);
	}
	
	/***
	 * Returns the current voltage the the motors are outputting
	 */
	public double getVoltage(){
		return controller.getMotorOutputVoltage();
	}
	
	/***
	 * Configures the Closed Feed-forward PID loop for the motor controllers
	 * @param p The proportional value of the PID loop
	 * @param i The integral value of the PID loop
	 * @param d The derivative value of the PID loop
	 * @param error The allowable error in the PID loop
	 */
	public void configPID(double p, double i, double d, int error){
		controller.config_kF(0, 0.23, standardTimeoutMs);
		controller.config_kP(0, p, standardTimeoutMs);
		controller.config_kI(0, i, standardTimeoutMs);
		controller.config_kD(0, d, standardTimeoutMs);
		controller.configAllowableClosedloopError(0, error, standardTimeoutMs);
	}
	
	/**
	 * @return The current error in the PID loop
	 */
	public int getPIDerror() {
		return (int) controller.getClosedLoopError(0);
	}
	
	/**
	 * @return The position, in ticks of the encoder
	 */
	public int getEncoderPosition() {
		return (int) controller.getSelectedSensorPosition(1);
	}
	
	public boolean getQuadAState() {
		return controller.getSensorCollection().getPinStateQuadB();
	}

	/**
	 * @return The number of Rotations the shaft of the motor has completed
	 */
	public double getRotations() {
		return controller.getSelectedSensorPosition(0)/ticksPerShaftRotation;
	}
	
	/**
	 * @return The velocity of the Encoder in ticks per microsecond
	 */
	public double getRawVelocity() {
			return controller.getSelectedSensorVelocity(0);
	}
	
	/**
	 * @return The velocity in Rotations per minute
	 */
	public double getVelocityRPM() {
		return getRawVelocity()/rpmToClicksPer100ms;	
	}
	
	/**
	 * The velocity in Rotations per second
	 * @return Rotations per second
	 */
	public double getVelocityRPS() {
		return getVelocityRPM() / 60;
	}
	
	/**
	 * Sets the speed of the motor to the speed given
	 * @param rps Rotations per second
	 */
	public void setVelocityRPS(double rps) {
		double clicksPer100ms = rps * rpsToClicksPer100ms;
		
		double finalSpeed = clicksPer100ms / actualOverRequestedRPM;// + 231;
		
		controller.set(ControlMode.Velocity, finalSpeed);
		
		SmartDashboard.putNumber("Requested RPS",  rps);
		SmartDashboard.putNumber("Requested clicks per 100 ms",  clicksPer100ms);
	}
	
	/**
	 * Sets the speed of the motor to the speed given
	 * @param rpm Rotations per minute
	 */
	public void setVelocityRPM(double rpm) {
		double clicksPer100ms = rpm * rpmToClicksPer100ms;
		
		double finalSpeed = clicksPer100ms / actualOverRequestedRPM;// + 231;
		
		controller.set(ControlMode.Velocity, finalSpeed);
		
		SmartDashboard.putNumber("Requested RPM",  rpm);
		SmartDashboard.putNumber("Requested clicks per 100 ms",  clicksPer100ms);
	}
	
	/**
	 * Tells the motor to turn to the ticks specified
	 * @param ticks The position in ticks to set the encoder to
	 */
	public void setPositionInTicks(int ticks) {
			controller.set(ControlMode.Position, ticks);
	}
	
	/**
	 * Tells the motor to rotate the shaft the specified number of times
	 * @param numRotations The number of times to rotate the shaft of the motor
	 */
	public void setRotations(double numRotations) {
			controller.set(ControlMode.Position, rotationsToTicks(numRotations));
	}
	
	/**
	 * Converts rotations to ticks
	 * @param rotations rotations to convert to ticks
	 * @return Number of ticks in specified number of rotations
	 */
	private double rotationsToTicks(double rotations){
		//convert rotations to feet
		return rotations*ticksPerShaftRotation;
	}
	
	/**
	 * Converts ticks to rotations
	 * @param ticks Number of ticks to convert
	 * @return Number of rotations in ticks
	 */
	@SuppressWarnings("unused")
	private double ticksToRotations(double ticks){
		//convert rotations to feet
		return ticks/ticksPerShaftRotation;
	}
	
	/**
	 * Resets the encoder position to 0
	 */
	public void zeroEncoder() {
		controller.setSelectedSensorPosition(0, 0, standardTimeoutMs);
	}
	
	
	public double getAmps() {
		return controller.getStatorCurrent();
	}

	/**
	 * Gets the state of the forward limit switch connected to the Motor Controller
	 * @return A boolean, true if the forward switch is been triggered
	 */
	public boolean getForwardLimitSwitchTriggered() {
		return controller.getSensorCollection().isFwdLimitSwitchClosed();
	}

	/**
	 * Gets the state of the reverse limit switch connected to the Motor Controller
	 * @return A boolean, true if the reverse switch has been triggered
	 */
	public boolean getReverseLimitSwitchTriggered() {
		return controller.getSensorCollection().isRevLimitSwitchClosed();
	}

	/**
	 * Inverts the direction the motor will turn.
	 * For example, if the motor is inverted then when providing the controller with 
	 * a positive voltage the shaft of the motor will spin in the reverse direction
	 */
	public void invertMotor(boolean invert){
		InvertType type = InvertType.None;

		if (invert) {
			type = InvertType.InvertMotorOutput;
		}else {
			type = InvertType.None;
		}

		controller.setInverted(type);
	}

	/**
	 * Indicates if you want this motor to mirror the motor it is following
	 * if true then this motor will spin forward when its leader spins forward
	 * if false then this motor will spin backward when its leader spins forward
	 * @param mirror
	 */
	public void mirrorLeaderOutput(boolean mirror) {
		if (mirror) {
			controller.setInverted(InvertType.FollowMaster);
		}else {
			controller.setInverted(InvertType.OpposeMaster);
		}
	}
}	