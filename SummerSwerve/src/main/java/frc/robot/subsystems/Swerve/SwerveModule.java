/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class SwerveModule {

private String name;
private WPI_TalonSRX driveMotor;
private WPI_TalonSRX steerMotor;

    public SwerveModule(String name, WPI_TalonSRX drive, WPI_TalonSRX steer){
        this.name = name;
        driveMotor = drive;
        steerMotor = steer;
    }

    public void stop(){
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    public void setSpeed(double speed){
        driveMotor.set(ControlMode.Position, speed);
    }

    public void setAngle(double angle){
        steerMotor.set(ControlMode.Position, angle);
    }

    public void setEncoder(int postion){
        steerMotor.setSelectedSensorPosition(postion, 0, 10);
    }

    public WPI_TalonSRX getDrive(){
        return driveMotor;
    }

    public WPI_TalonSRX getSteer(){
        return steerMotor;
    }

    public void move(double speed, double angle){
        setSpeed(speed);
        if (speed != 0.0) { setAngle(angle); }
    }

}
