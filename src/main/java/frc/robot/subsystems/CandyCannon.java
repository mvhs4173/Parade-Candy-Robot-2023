// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/** Add your docs here. */
public class CandyCannon {
    private final Value EXTENDED_VALUE = Value.kForward;

    private DoubleSolenoid doubleSolenoid;

    public CandyCannon(DoubleSolenoid doubleSolenoid) {
        this.doubleSolenoid = doubleSolenoid;
    }

    public boolean getIsExtended() {
        return doubleSolenoid.get() == EXTENDED_VALUE;
    }

    public void setIsExtended(boolean shouldBeExtended) {
        Value value;
        if (shouldBeExtended) {
            value = EXTENDED_VALUE;
        } else {
            if (EXTENDED_VALUE == Value.kForward) {
                value = Value.kReverse;
            } else {
                value = Value.kForward;
            }
        }

        doubleSolenoid.set(value);
    }
}
