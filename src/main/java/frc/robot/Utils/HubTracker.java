package frc.robot.Utils;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HubTracker {
    public static HubTracker instance;

    private HubTracker(){}

    public void initalizeMatchTimer(){
        matchTimer.reset();
        matchTimer.start();
    }
    public void stopMatchTimer(){
        matchTimer.stop();
    }

    public static HubTracker getInstance(){
        if(instance == null) instance = new HubTracker();
        return instance;
    }

    public double getMatchTime(){
        return matchTimer.get();
    }

    private Timer matchTimer = new Timer();

    public Optional<Alliance> getAutoWinner(){
        char gameData = DriverStation.getGameSpecificMessage().length() >= 1 ? DriverStation.getGameSpecificMessage().charAt(0) : ' ';
        switch(gameData){
            case 'B':
                return Optional.of(Alliance.Blue);
            case 'R':
                return Optional.of(Alliance.Red);
            default:
                return Optional.empty();
            
        }
    }

    public boolean isAutoWinner(){
        return getAutoWinner().equals(DriverStation.getAlliance());
    }

    public Shift getCurrentShift(){
        if(DriverStation.isAutonomous() && matchTimer.get() <= Shift.AUTO.getEndTime()){
            return Shift.AUTO;
        }
        if(matchTimer.get() <= Shift.TRANSITION.getEndTime()) {
            return Shift.TRANSITION;
        } else if(matchTimer.get() <= Shift.SHIFT_1.getEndTime()) {
            return Shift.SHIFT_1;
        } else if(matchTimer.get() <= Shift.SHIFT_2.getEndTime()) {
            return Shift.SHIFT_2;
        } else if(matchTimer.get() <= Shift.SHIFT_3.getEndTime()) {
            return Shift.SHIFT_3;
        } else if(matchTimer.get() <= Shift.SHIFT_4.getEndTime()) {
            return Shift.SHIFT_4;
        } else if (matchTimer.get() <= Shift.END_GAME.getEndTime()) {
            return Shift.END_GAME;
        } else {
            return Shift.UNKNOWN;
        }
    }

    public boolean isHubActive(){
        if(getAutoWinner().isEmpty()){
            return false;
        }
        if (isAutoWinner()) {
            return getCurrentShift().getActiveType().equals(ActiveType.AUTO_WINNER) || getCurrentShift().getActiveType().equals(ActiveType.BOTH);
        } else {
            return getCurrentShift().getActiveType().equals(ActiveType.AUTO_LOSER) || getCurrentShift().getActiveType().equals(ActiveType.BOTH);
        }
    }

    public enum ActiveType{
        BOTH,
        AUTO_WINNER,
        AUTO_LOSER
    }

    public enum Shift{
        AUTO(0, 20, ActiveType.BOTH),
        TRANSITION(0, 10, ActiveType.BOTH),
        SHIFT_1(10, 35, ActiveType.AUTO_LOSER),
        SHIFT_2(35, 60, ActiveType.AUTO_WINNER),
        SHIFT_3(60, 85, ActiveType.AUTO_LOSER),
        SHIFT_4(85, 110, ActiveType.AUTO_WINNER),
        END_GAME(110, 140, ActiveType.BOTH),
        UNKNOWN(0, 140, ActiveType.BOTH);

        final double startTime;
        final double endTime;
        final ActiveType activeType;

        private Shift(double startTime, double endTime, ActiveType activeType){
            this.startTime = startTime;
            this.endTime = endTime;
            this.activeType = activeType;
        }

        public double getEndTime(){
            return this.endTime;
        }

        public ActiveType getActiveType(){
            return this.activeType;
        }

    }
}
