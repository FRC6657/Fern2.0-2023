package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants.State;
import frc.robot.Constants.RobotConstants.CAN;

public class Pivot extends SubsystemBase {
    
    private final WPI_TalonFX mPivot;
    private final DutyCycleEncoder mEncoder;
    private final PIDController mPID;

    private State mCurrentState = State.STARTING;
    public double falconOffset;
    public double trimVal = 0;

    public boolean firstRun = true; 

    public Pivot() {

        mPivot = new WPI_TalonFX(CAN.kPivot);
        mEncoder = new DutyCycleEncoder(9);
        mPID = new PIDController(2.5 / 20d, 0, 0);
       
        Timer.delay(2);
        
        mPivot.setInverted(InvertType.InvertMotorOutput);
        mPivot.setSelectedSensorPosition(0);
        mEncoder.setPositionOffset(PivotConstants.kThroughboreOffset);
        falconOffset = degreeToFalcon(getThroughBoreAngle());

        configureMotor();
        
    }

    public void configureMotor() {

        mPivot.configFactoryDefault();
        mPivot.setNeutralMode(NeutralMode.Brake);

        mPivot.configVoltageCompSaturation(10);
        mPivot.enableVoltageCompensation(true);
        mPivot.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 25, 0));
    }

    public void runPivot() {
        mPivot.set(mPID.calculate(getAngle(), mCurrentState.angle)/12);
        SmartDashboard.putNumber("output",-mPID.calculate(getAngle(), mCurrentState.angle)/12);
    }

    public boolean atTarget() {

        double tolerance = 0.5; 

        return (Math.abs(getAngle() - mCurrentState.angle + trimVal) < tolerance);

    }
  

    public double falconToDegrees(double val){
        return val * 1/2048d * PivotConstants.kGearing * 360;
    }

    public void zeroEncoder() {
        mPivot.setSelectedSensorPosition(0);
        falconOffset = degreeToFalcon(getThroughBoreAngle());
    }

    public double getAngle() {
        return falconToDegrees(mPivot.getSelectedSensorPosition() + falconOffset);
    }

    public double getThroughBoreAngle () {
        return ((mEncoder.getAbsolutePosition()) - mEncoder.getPositionOffset()) * 360;
    }

    public void resetFalcon() {
        mPivot.setSelectedSensorPosition(0);
    }

    public double degreeToFalcon(double deg) {
        return (deg * 2048d * 1/PivotConstants.kGearing * 1/360);
    }

    public Command changeState(State state){
      return new InstantCommand(() -> mCurrentState = state);
    }

    public void set(double percent){
      mPivot.set(percent);
    }

    @Override
    public void periodic() {

        runPivot();

        SmartDashboard.putNumber("TBE Raw", mEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("TBE Degrees", getThroughBoreAngle());
        SmartDashboard.putNumber("Falcon Degrees", getAngle());
        SmartDashboard.putNumber("Falcon Offset", falconOffset);
        SmartDashboard.putBoolean("At Setpoint", atTarget());
        SmartDashboard.putNumber("Motor Voltage", mPivot.get() * 12);
        SmartDashboard.putNumber("Set Point", mCurrentState.angle);

    }
}