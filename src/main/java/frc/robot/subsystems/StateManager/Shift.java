package frc.robot.subsystems.StateManager;

public enum Shift {
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
   * s2: 35s
   * s4: 85s
   * endgame: 110s
   *
   * set 2: if you lost auto
   * s1: 10s
   * s3: 60s
   * endgame: 110s
   */

  TRANSITION(0.0),
  S1(10.0),
  S2(35.0),
  S3(60.0),
  S4(85.0),
  ENDGAME(110.0),
  END(140.0);

  public double time;

  private Shift(double t) {
    time = t;
  }
}
