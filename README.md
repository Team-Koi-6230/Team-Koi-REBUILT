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

