/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.debloating;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Add your docs here.
 */
public class ColorSensor {
    I2C.Port i2cPort = I2C.Port.kOnboard;
    public ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    public Color color(){
        return colorSensor.getColor();
    }

    public double ir(){
        return colorSensor.getIR();
    }

    public boolean objPresent(){
        return colorSensor.getProximity() > 1300; //todo get right number
    }

    public boolean isRedNotBlue(){
        return color().red > color().blue;
    }

    public static class RGBValue{
        private int r, g, b;
        public RGBValue(int r, int g, int b){
            this.r = r;
            this.g = g;
            this.b = b;
        }
        public int r(){
            return r;
        }
        public int g(){
            return g;
        }
        public int b(){
            return b;
        }
    }
}
