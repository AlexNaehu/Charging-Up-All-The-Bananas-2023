package frc.robot.Autonomous;

import frc.robot.Mechanisms.BananaClaw;
import frc.robot.Robot.BananaPreSets;
import frc.robot.Robot.Robot;

public class BananaAutonPaths {





    





    private final static double leftHalfTurn = 90.0; //"left turn" means the front right corner whips around CCW
    private final static double leftFullTurn = 180;
    private final static double rightHalfTurn = -90.0; //"right turn" means the front left corner whips around CW
    private final static double rightFullTurn = -180;

    

    private final static int straightForwardSide = 0;
    private final static int straightForwardMid = 1;
    private final static int tipToeForward = 2; //Used for inching onto the charging station and for approaching Scoring

    private final static int straightBackwardSide = 3;
    private final static int straightBackwardMid = 4;
    private final static int tipToeBackward = 5; //Using for inching into the charging station and for approaching Scoring



    /*IMPORTANT: FIGURE OUT HOW TO PRELOAD OBJECTS INTO THE CLAW*/
    /*Also Important: Consider using a timer to seperate each command by 0.2 seconds so that error doesn't stack */





   






    public static void rightScoreMob() //turns left, away from the wall on the right side of the robot
    {   
        BananaPreSets.cargoPickUp();
        BananaDriveStraight.driveStraight(tipToeBackward);
        BananaClaw.closeClaw(0.2);
        BananaPreSets.lvl3RocketBall();
        BananaDriveStraight.driveStraight(tipToeForward);
        //Robot.driveTrain.aimPIDState = true;
        BananaClaw.openClaw(0.2);
        //Robot.driveTrain.aimPIDState = false;
        BananaDriveStraight.driveStraight(straightBackwardSide);
        BananaPreSets.cargoPickUp();
        BananaTurn.turnPID(leftFullTurn, 0.0);
        BananaDriveStraight.driveStraight(straightForwardSide);
        BananaClaw.closeClaw(0.2);
        BananaPreSets.lvl1RocketBall();
        BananaTurn.turnPID(leftFullTurn, 0.0);
        BananaDriveStraight.driveStraight(straightForwardSide);

    }

    public static void leftScoreMob() //turns right, away from the wall on the left side of the robot
    {
        //BananaClaw.closeClaw(0.2);
        BananaPreSets.lvl3RocketBall();
        BananaDriveStraight.driveStraight(tipToeForward);
        //Robot.driveTrain.aimPIDState = true;
        BananaClaw.openClaw(0.2);
        //Robot.driveTrain.aimPIDState = false;
        BananaDriveStraight.driveStraight(straightBackwardSide);
        BananaPreSets.cargoPickUp();
        BananaTurn.turnPID(rightFullTurn, 0.0);
        BananaDriveStraight.driveStraight(straightForwardSide);
        BananaClaw.closeClaw(0.2);
        BananaPreSets.lvl1RocketBall();
        BananaTurn.turnPID(rightFullTurn, 0.0);
        BananaDriveStraight.driveStraight(straightForwardSide);

    }

    //turns left, doesn't actually matter which way; also inches onto the charging station
    public static void midScorePark() 
    {
        //BananaClaw.closeClaw(0.2);
        BananaPreSets.lvl3RocketBall();
        BananaDriveStraight.driveStraight(tipToeForward);
        //Robot.driveTrain.aimPIDState = true;
        BananaClaw.openClaw(0.2);
        //Robot.driveTrain.aimPIDState = false;
        BananaDriveStraight.driveStraight(straightBackwardMid);
        BananaClaw.closeClaw(0.2);
        BananaPreSets.neutralPivotAngle();
        BananaTurn.turnPID(leftFullTurn, 0.0);
        BananaDriveStraight.driveStraight(straightForwardMid);
        BananaDriveStraight.driveStraight(tipToeForward);

    } 
}
