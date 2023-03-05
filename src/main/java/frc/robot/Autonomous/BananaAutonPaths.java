package frc.robot.Autonomous;

import frc.robot.Mechanisms.BananaClaw;
import frc.robot.Robot.BananaPreSets;

public class BananaAutonPaths {
    
    private final double leftHalfTurn = 90.0;
    private final double leftFullTurn = 180;
    private final double rightHalfTurn = -90.0;
    private final double rightFullTurn = -180;

    

    private final int straightForwardSide = 0;
    private final int straightForwardMid = 1;
    private final int tipToeForward = 2; //Used for inching onto the charging station and for approaching Scoring

    private final int striaghtBackwardSide = 3;
    private final int straightBackWardMid = 4;
    private final int tipToeBackward = 5; //Using for inching into the charging station and for approaching Scoring

    public void rightScoreMob()
    {
        BananaPreSets.lvl3RocketBall();
        BananaDriveStraight.driveStraight(2);
        BananaClaw.openClaw(0.2);
        BananaDriveStraight.driveStraight(3);
        BananaTurn.turnPID(leftFullTurn, 0.0);

    }
}
