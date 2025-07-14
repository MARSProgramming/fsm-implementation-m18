package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;

public class RobotMap {
    public class DIO {
      public static final int LIMIT = 0;
    }

    public class PWM {
      public static final int SERVO = 4;
    }

    public class CAN {
      public static final CANBus SECONDARY_BUS = new CANBus("CAN-2", "./logs/example.hoot");

      public static final int CORAL = 12;
      public static final int ELEVATOR_MASTER = 18;
      public static final int ELEVATOR_FOLLOWER = 19;

      public static final int PIGEON_LEFT = 20;
      public static final int CANDLE = 40;

      public static final int FRONT_LEFT_DRIVE = 2;
      public static final int FRONT_LEFT_STEER = 3;
      public static final int BACK_LEFT_DRIVE = 4;
      public static final int BACK_LEFT_STEER = 5;
      public static final int FRONT_RIGHT_DRIVE = 8;
      public static final int FRONT_RIGHT_STEER = 9;
      public static final int BACK_RIGHT_DRIVE = 6;
      public static final int BACK_RIGHT_STEER = 7;
    }

    public class SPARK {
      public static final int ALGAE = 11;
    }

    public class VISION {
      public static final String reefCameraName = "reef_cam";
      public static final String feederCameraName = "feeder_cam";

      public static final Transform3d feederRobotToCam =
          new Transform3d(
              Units.Inches.of(-6),
              Units.Inches.of(9.06),
              Units.Inches.of(11.55),
              new Rotation3d(Units.Degrees.of(0), Units.Degrees.of(0), Units.Degrees.of(125)));

      public static final Transform3d reefRobotToCam =
          new Transform3d(
              Units.Inches.of(-6.3),
              Units.Inches.of(9.49),
              Units.Inches.of(13.44),
              new Rotation3d(Units.Degrees.of(-10), Units.Degrees.of(0), Units.Degrees.of(90)));
    }

    public class RIO {
      public static final int IR = 1;
    }
  
}
