package com.stormbots.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * A lambda-based Command to enable short, simplistic CommandGroups and OI buttons
 * without creating excessive numbers of simple Commands.
 * <br/>
 * To use, create a new LambdaCommand, and provide a Lambda function that returns a boolean, eg
 * <pre>
 * new LambdaCommand(()->{print("do stuff") ; return true;});
 * </pre>
 * If needed, you can <code>require</code> subsystems by simply ading them as additional  LambdaCommand constructor arguments:
 * <pre>
 * new LambdaCommand(()->{print("do stuff") ; return true;}, Robot.chassis, Robot.lifter2);
 * </pre>
 * Any number of subsystems can be <code>require</code>'d this way.
 * <br>
 * In a <code>CommandGroup</code>, you can always exit based on a timer by using the optional run duration argument for
 * AddSequential and AddParallel, eg:
 * <pre>
 * //Does nothing, but does it for 3 seconds
 * addSequential(new LambdaCommand(()->{return false}),3);
 * //Drive forward for 3 seconds
 * addSequential(new LambdaCommand(()->{Robot.chassis.arcadeDrive(0.1, 0); return false},Robot.chassis),3);
 */
public class LambdaCommand extends Command {
  public BooleanSupplier lambda;
  boolean finished = true;
  
  /**
   * Quick interface to Commands, suitable for more simplistic ones that don't need initializtion. See type 
   * documentation for examples and more details. 
   * 
   * @param lambda A lambda function that returns a boolean indicating if the command should stop. eg <pre>()->{return true;}</pre>
   * @param requires Optional: Any number of subsystems this command should <code>require</code> as part of it's operation.
   */
  public LambdaCommand(BooleanSupplier lambda, Subsystem ...requires) {
      this.lambda = lambda;
      for(Subsystem system : requires){
        requires(system);
      }
  }

  @Override
  protected void execute() {
    this.finished = lambda.getAsBoolean();
  }

  @Override
  protected boolean isFinished() {
    return finished;
  }


}
