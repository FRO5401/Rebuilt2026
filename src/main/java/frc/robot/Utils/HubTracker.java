package frc.robot.Utils;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HubTracker {
    public static HubTracker instance;
    private Timer matchTimer = new Timer();

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

    protected double getDriverStationTime(){
        return DriverStation.getMatchTime();
    }

    protected double getTimerTime(){
        return 140 - matchTimer.get();
    }

    public double getMatchTime(){
        //checks if the timers are within a second and return the more accurate time
        return (getDriverStationTime() % 3 == 0 && !MathHelp.epsilonEquals(getDriverStationTime(), getTimerTime(), 1.01)) ? getDriverStationTime() : getTimerTime();
    }

    public double getShiftTimeCountdown(){
        return getMatchTime() - getCurrentShift().endTime;
    }

    //TODO: test this on the real robot is this may crash if this doesnt return a value.
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
        if(DriverStation.isAutonomous() && getMatchTime() >= Shift.AUTO.getEndTime()){
            return Shift.AUTO;
        }
        if(getMatchTime() >= Shift.TRANSITION.getEndTime()) {
            return Shift.TRANSITION;
        } else if(getMatchTime() >= Shift.SHIFT_1.getEndTime()) {
            return Shift.SHIFT_1;
        } else if(getMatchTime() >= Shift.SHIFT_2.getEndTime()) {
            return Shift.SHIFT_2;
        } else if(getMatchTime() >= Shift.SHIFT_3.getEndTime()) {
            return Shift.SHIFT_3;
        } else if(getMatchTime() >= Shift.SHIFT_4.getEndTime()) {
            return Shift.SHIFT_4;
        } else if (getMatchTime() >= Shift.END_GAME.getEndTime()) {
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
        AUTO(20, 0, ActiveType.BOTH),
        TRANSITION(140, 130, ActiveType.BOTH),
        SHIFT_1(130, 105, ActiveType.AUTO_LOSER),
        SHIFT_2(105, 80, ActiveType.AUTO_WINNER),
        SHIFT_3(80, 55, ActiveType.AUTO_LOSER),
        SHIFT_4(55, 30, ActiveType.AUTO_WINNER),
        END_GAME(30, 0, ActiveType.BOTH),
        UNKNOWN(140, 0, ActiveType.BOTH);

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
