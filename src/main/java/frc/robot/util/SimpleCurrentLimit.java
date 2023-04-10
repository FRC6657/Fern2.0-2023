package frc.robot.util;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class SimpleCurrentLimit {
    
    public static final SupplyCurrentLimitConfiguration getSimpleCurrentLimit(double limit){
        return new SupplyCurrentLimitConfiguration(true, limit, limit, 0);
    }
    
}
