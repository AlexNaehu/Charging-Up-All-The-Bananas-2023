package frc.robot.Mechanisms;


//import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.XboxController;

//import com.ctre.phoenix.motorcontrol.can.BaseMotorController; //used to make VictorSPX motors follow eachother
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.IMotorController;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Robot.BananaConstants;



public class BananaIntake {
    
    private static WPI_VictorSPX leftRoller;
    private static WPI_VictorSPX rightRoller;
    private static WPI_VictorSPX claw;


    private int leftRoller_ID = 25;
    private int rightRoller_ID = 29;
    private int claw_ID = 25;


    public static boolean intakeOpen = true;

    public BananaIntake()
    {

        leftRoller = new WPI_VictorSPX(leftRoller_ID);
        rightRoller = new WPI_VictorSPX(rightRoller_ID);
        //claw = new WPI_VictorSPX(claw_ID);

    }

    public static void changeClawState()
    {
        intakeOpen = !intakeOpen;
    }
    public static void intake(double power) //takes 2 seconds to close around a cone from full open 
    {
        leftRoller.set(-power*0.8);//0.55
        rightRoller.set(power*0.8);//0.55
        //claw.set(0.5);
    }

    public static void output(double power) // takes 2 seconds to open from a cone closed position back to full open
    {
        leftRoller.set(power*0.6);
        rightRoller.set(-power*0.6);
        //claw.set(-0.5);
    }

    public boolean isIntakeOpen()
    {
        return intakeOpen;
    }

    public double getLeftRollerPower()
    {
        return leftRoller.get();
        //return claw.get();
    }

    public double getRightRollerPower()
    {
        return rightRoller.get();
    }
    


}

