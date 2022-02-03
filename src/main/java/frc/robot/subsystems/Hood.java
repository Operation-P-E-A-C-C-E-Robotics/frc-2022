// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Hood extends SubsystemBase {
    WPI_TalonSRX hoodMotor = new WPI_TalonSRX(0); //todo port number
    
    private final double ENCODER_COUNTS_PER_INCH = 0; //todo get actual
    /** Creates a new Hood. */
    public Hood() {
    }

    public void setHoodPercent(double percent){
        hoodMotor.set(percent);
    }

    public void setHoodPosition(double position){
        
    }

    @Override
    public void periodic() {
        if(hoodMotor.isRevLimitSwitchClosed() == 1) hoodMotor.setSelectedSensorPosition(0);
    }
}