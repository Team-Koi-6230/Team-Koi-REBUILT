# Team Koi#6230 REBUILT

## Library changes for 2026

### Installations

[WPILib + tools for 2026](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)

[Driver station 2026](https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html#581857)

[Rev Client 2 - 2026v](https://docs.revrobotics.com/rev-hardware-client-2)

### WPILib
- Deprecate `Command.schedule()`, use `CommandScheduler.getInstance().schedule(/*Command*/)` instead

(some other stuff changed but they don't really matter I'll be honest)
 
- 2026 Game data
```
import edu.wpi.first.wpilibj.DriverStation;
String gameData;
gameData = DriverStation.getGameSpecificMessage();
if(gameData.length() > 0)
{
  switch (gameData.charAt(0))
  {
    case 'B' :
      //Blue case code
      break;
    case 'R' :
      //Red case code
      break;
    default :
      //This is corrupt data
      break;
  }
} else {
  //Code for no data received yet
}
```
honestly no Idea what to do with this, but it's new for this season.

- Not new but Shmickler wants to use [Servos](https://en.wikipedia.org/wiki/Servomotor) on the robot so here's how to use them:
```java
Servo exampleServo = new Servo(/*PWM port*/);
exampleServo.set(0.5); // voltage from 0-1 (0 starting angle, 1 final possible angle)
exampleServo.setAngle(75); // im not explaining it you should get it by the name
```

### REVLib
lots of changes here, I hate it and love it at the same time

- `closedLoopController.setRefrence(params)` was deprecated, we will now use the following:
```java
m_controller.setSetpoint(/*params are the same I have no idea why they changed it*/);
```

- This one is actually a banger, they added FeedForward into closedLoop controller:
```java
config.closedLoop.feedForward
    .kS(s)
    .kV(v)
    .kA(a)
    .kG(g) // normal gravity, for elevators
    .kCos(g) // gravity related to angle, for arms
    .kCosRatio(cosRatio); // kCosRatio relates the encoder position to absolute position
```

