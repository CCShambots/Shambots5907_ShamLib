package frc.robot.ShamLib;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WhileDisabledInstantCommand extends InstantCommand {

  /**
   * This command exists because the default InstandCommand doesn't run while disabled, and the SMF
   * generally relies on running at least some commands while the bot is disabled (especially for
   * always-enabled subsystems like vision and lights)
   *
   * @param toRun Normal Instant Command style runnable
   */
  public WhileDisabledInstantCommand(Runnable toRun) {
    super(toRun);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
