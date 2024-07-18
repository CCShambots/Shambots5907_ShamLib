package frc.robot.ShamLib.test.smf;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.ShamLibConstants;
import org.junit.jupiter.api.*;

public class StateMachineTesting {
  private ExampleStateMachine stateMachine;

  @BeforeEach
  public void setUp() {
    stateMachine = new ExampleStateMachine("x", ExampleState.A, ExampleState.class);
    stateMachine.enable();
    CommandScheduler.getInstance().enable();
  }

  @AfterEach
  public void tearDown() {
    stateMachine.disable();
    CommandScheduler.getInstance().disable();
  }

  @Test
  public void determine() {
    stateMachine.determineState();
    assert stateMachine.isDetermined();
    assert stateMachine.getState() == ExampleState.B;
  }

  @Test
  public void instantTransition() {
    stateMachine.determineState();

    stateMachine.requestTransition(ExampleState.D);
    assert stateMachine.getState() == ExampleState.D;

    stateMachine.requestTransition(ExampleState.C);
    assert stateMachine.getState() == ExampleState.C;
    assert !stateMachine.isTransitioning();
  }

  @Test
  public void timedTransition() {
    Timer t = new Timer();

    stateMachine.determineState();

    stateMachine.requestTransition(ExampleState.E);

    assert stateMachine.isTransitioning();
    assert stateMachine.getState() != ExampleState.E;

    t.start();
    while (!t.hasElapsed(3.5)) {
      stepCommandScheduler(1);
    }
    t.stop();

    assert stateMachine.getState() == ExampleState.E;
    assert !stateMachine.isTransitioning();
  }

  @Test
  public void flagStates() {
    stateMachine.determineState();
    stateMachine.requestTransition(ExampleState.D);
    stepCommandScheduler(2);

    assert stateMachine.isFlag(ExampleState.FLAG);

    stateMachine.requestTransition(ExampleState.C);

    assert !stateMachine.isFlag(ExampleState.FLAG);
  }

  @Test
  public void timedTransitionInterrupt() {
    Timer t = new Timer();

    stateMachine.determineState();
    stateMachine.requestTransition(ExampleState.E);

    stateMachine.requestTransition(ExampleState.D);
    assert stateMachine.getCurrentTransition().getEndState() != ExampleState.D;

    t.start();
    while (!t.hasElapsed(ShamLibConstants.SMF.TRANSITIONTIMEOUT + 0.1)) stepCommandScheduler(1);
    t.stop();

    assert !stateMachine.isTransitioning();
    assert stateMachine.getState() == ExampleState.D;
  }

  private void stepCommandScheduler(int i) {
    for (int j = 0; j < i; j++) {
      CommandScheduler.getInstance().run();
    }
  }
}

class ExampleStateMachine extends StateMachine<ExampleState> {
  public ExampleStateMachine(
      String name, ExampleState undeterminedState, Class<ExampleState> enumType) {
    super(name, undeterminedState, enumType);
    declareTransitions();
    declareStateCommands();
  }

  @Override
  protected void onEnable() {}

  @Override
  protected void onDisable() {}

  private void declareStateCommands() {
    registerStateCommand(
        ExampleState.D, new DisabledInstantCommand(() -> setFlag(ExampleState.FLAG)));
  }

  private void declareTransitions() {
    addTransition(ExampleState.B, ExampleState.D, new InstantCommand());
    addTransition(ExampleState.D, ExampleState.C, new InstantCommand());
    addTransition(ExampleState.B, ExampleState.E, new WaitCommand(3));
  }

  @Override
  protected void update() {}

  @Override
  protected void determineSelf() {
    setState(ExampleState.B);
  }
}

class DisabledInstantCommand extends InstantCommand {

  public DisabledInstantCommand(Runnable toRun, Subsystem... requirements) {
    super(toRun, requirements);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

enum ExampleState {
  A,
  B,
  C,
  D,
  E,
  FLAG
}
