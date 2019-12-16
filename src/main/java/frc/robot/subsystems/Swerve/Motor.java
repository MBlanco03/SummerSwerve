/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * Add your docs here.
 */
public class Motor {
    private WPI_TalonSRX motorT;
    private WPI_VictorSPX motorV;

    public Motor(WPI_TalonSRX m){
        motorT = m;
    }
    public Motor(WPI_VictorSPX m){
        motorV = m;
    }

    public void stop(){
        motorT.stopMotor();
        motorV.stopMotor();
    }
}
