package frc.robot.Mechanisms;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController; //used to make VictorSPX motors follow eachother
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot.BananaConstants;
import frc.robot.Robot.*;
import edu.wpi.first.wpilibj.AnalogInput;

public class BananaBrake {



    private static WPI_VictorSPX leftBrake;
    private static WPI_VictorSPX rightBrake;

    private int leftBrake_ID = 26;
    private int rightBrake_ID = 27;

    public static boolean brakeOn = false;

    public BananaBrake()
    {

        leftBrake = new WPI_VictorSPX(leftBrake_ID);
        rightBrake = new WPI_VictorSPX(rightBrake_ID);

    }

    public static void changeBrakeState()
    {
        brakeOn = true;
    }
    public static void Brake(double power) //can only turn brake on, not off
    {
        leftBrake.set(power);
        rightBrake.set(-power);
    }


    public boolean isBrakeOn()
    {
        return brakeOn;
    }

    public double getleftBrakePower()
    {
        return leftBrake.get();
    }

    public double getrightBrakePower()
    {
        return rightBrake.get();
    }
}
