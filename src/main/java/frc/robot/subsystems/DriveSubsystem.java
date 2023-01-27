// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;

public class DriveSubsystem extends SubsystemBase {
  // drive motors
  private final CANSparkMax frontLeftMotor = new CANSparkMax (Constants.DriveConstants.kfrontLeftMotorDeviceID, MotorType.kBrushless);
  private final CANSparkMax rearLeftMotor = new CANSparkMax   (Constants.DriveConstants.krearLeftMotorDeviceID, MotorType.kBrushless);
  private final CANSparkMax frontRightMotor = new CANSparkMax (Constants.DriveConstants.kfrontRightMotorDeviceID, MotorType.kBrushless);
  private final CANSparkMax rearRightMotor = new CANSparkMax  (Constants.DriveConstants.krearRightMotorDeviceID, MotorType.kBrushless);

  // the robot's drive
  private final MecanumDrive m_drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);


  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    m_drive.setDeadband(OperatorConstants.kDeadband);

    frontRightMotor.setInverted(true);
    rearRightMotor.setInverted(true);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //m_drive.driveCartesian(0, 0, 0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void drive (double xSpeed, double ySpeed, double rot){
    m_drive.driveCartesian(xSpeed, ySpeed, rot);
  }

    public void setMotors(double frontLeftSpeed, double rearLeftSpeed, double frontRightSpeed, double rearRightSpeed){
      frontLeftMotor.set(applyDeadband(frontLeftSpeed,OperatorConstants.kDeadband));
      rearLeftMotor.set(applyDeadband(rearLeftSpeed,OperatorConstants.kDeadband));
      frontRightMotor.set(applyDeadband(frontRightSpeed,OperatorConstants.kDeadband));
      rearRightMotor.set(applyDeadband(rearRightSpeed,OperatorConstants.kDeadband));
    }
  

  public double applyDeadband(double value, double deadBandCutOff){
    //Ref: https://www.chiefdelphi.com/t/how-do-i-program-a-joystick-deadband/122625/7
    //
    // -1                  -deadBandCutOff                         deadBandCutOff                   1
    // ├─────────────────────────────────┼────────────────────────┼─────────────────────────────────┤
    // │ value in this range             │ value in this range    │ value in this range             │
    // │ will be scaled to [-1,0]        │ will be set to 0       │ will be scaled to [0, 1]        │
    // │ to avoid joystick drifting      │ to avoid burning motor │ to avoid joystick drifting      │
    // 
    //
    if(Math.abs(value) < deadBandCutOff) return 0.0;  // if value in the range [-deadBandCutOff, deadBandCutOff], set it to 0 to avoid burning the motor
    else{                                             // if value in the range [-1, -deadBandCutOff] or [deadBandCutOff, 1] 
                                                      //    scale it to [-1,0] or [0,1] to avoid joystick drifting
      return (value -                                 // initially in one of two ranges: [-1,-deadBandCutOff] or [deadBandCutOff,1]
            (Math.abs(value)/ value                   // -1 if value < 0, 1 if value > 0, so effectively it's the sign of value
            * deadBandCutOff)                         // multiply by the sign so that for >0, it comes out to - (deadBandCutOff), and for <0 it comes to - (-deadBandCutOff)
            )                                         // now in either [-1+deadBandCutOff,0] or [0,1-deadBandCutOff]
            / (1-deadBandCutOff);                     // scale to [-1,0] or [0,1]
    }
  }
}
