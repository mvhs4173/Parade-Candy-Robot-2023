// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NeoMotor extends SubsystemBase {
  private CANSparkMax m_sparkMax;
  private SparkMaxPIDController m_pidController;
  /** Creates a new NeoMotor. */
  public NeoMotor(int motorID) {
    m_sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
    m_pidController = m_sparkMax.getPIDController();
    setVoltage(0);
  }

  public void setVoltage(double speed) {
      m_pidController.setReference(speed, CANSparkMax.ControlType.kVoltage);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
