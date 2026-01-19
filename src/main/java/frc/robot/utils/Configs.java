package frc.robot.utils;


import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

//import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Configs {

    public static final class HopperConfigs {
        public static final SparkFlexConfig hopperConfig = new SparkFlexConfig();
        public static final MAXMotionConfig hopperMotionConfig = new MAXMotionConfig();

        static {
            hopperConfig
                    .idleMode(IdleMode.kBrake);

            hopperConfig.closedLoop
                    .p(0)
                    .i(0)
                    .d(0);

        }

    }

    public static final class IntakeConfigs {
        public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
        static {
            intakeConfig
                    .idleMode(IdleMode.kBrake);
        }
    }

}