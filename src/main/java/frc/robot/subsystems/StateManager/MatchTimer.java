package frc.robot.subsystems.StateManager;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class MatchTimer {
  // mechanical advantage is goated my brain is too small for ts
  private Timer timer;
  private Alliance wAlliance; // auto winning alliance
  private Alliance mAlliance; // our alliance
  private Shift[] shiftVals = Shift.values();
  private Shift currShift = shiftVals[0];
  private Shift nextShift = shiftVals[0];
  private int index = 0;
  private boolean wonAuto = false;

  public MatchTimer() {
    // mAlliance = DriverStation.getAlliance().orElse(null);
    mAlliance = Alliance.Red; // because sim is funky with alliances
    timer = new Timer();
  }

  public MatchTimer getInstance() {
    return this;
  }

  public void startTeleop() {
    timer.start();
    updateAutoWinningAlliance();
    currShift = shiftVals[0];
    index = 0;
    if (!wonAuto) nextShift = shiftVals[1]; // shift 1
    else nextShift = shiftVals[2]; // shift 2
  }

  public void updateAll() {
    updateAutoWinningAlliance();
    updateShift();
  }

  public void updateShift() {
    double time = timer.get();
    if (index != 5) {
      if (time >= shiftVals[index + 1].time) {
        currShift = shiftVals[++index];
      }
      if (time >= nextShift.time) {
        int nextInd = index + 2;
        if (nextInd > 5) nextInd = 5;
        nextShift = shiftVals[nextInd];
      }
    }
  }

  public void updateAutoWinningAlliance() {
    String gameData = DriverStation.getGameSpecificMessage();

    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'R':
          wAlliance = Alliance.Red;
          break;
        case 'B':
          wAlliance = Alliance.Blue;
          break;
        default:
          break;
      }

      if (wAlliance == mAlliance) wonAuto = true;
    }
  }

  public Alliance getAutoWinningAlliance() {
    return wAlliance;
  }

  public Alliance getFirstShiftAlliance() {
    return wAlliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
  }

  public double getTimeToShift() {
    double tElapsed = timer.get();
    return nextShift.time - tElapsed;
  }

  public Shift getCurrentShift() {
    return currShift;
  }

  public Shift getNextShift() {
    return nextShift;
  }

  public boolean didWeWin() {
    return wonAuto;
  }

  /* match timing
   * 2:20 (0 s elapsed) teleop start
   *
   * 2:10 (10 s) transition shift end, shifts start
   * 1:45 (35 s)
   * 1:20 (60 s)
   * 0:55 (85 s)
   *
   * 0:30 (110 s) endgame
   * 0:00 (140 s) match end
   *
   *
   * set 1: if you won auto
   * 35s
   * 85s
   * 110s endgame
   *
   * set 2: if you lost auto
   * 10s
   * 60s
   * 110s endgame
   */
}
