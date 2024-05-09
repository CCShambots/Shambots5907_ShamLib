package frc.robot.ShamLib.swerve;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.ShamLibConstants.BuildMode;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.swerve.module.ModuleInfo;

public class SwerveDriveConfig {
    public BuildMode buildMode = BuildMode.REPLAY;
    
    //Gyro config
    public int pigeon2ID = 0;
    public String gyroCanbus = "";
    
    //Module configs
    public PIDSVGains moduleDriveGains = new PIDSVGains();
    public PIDSVGains moduleTurnGains = new PIDSVGains();
    public double maxModuleTurnVelo = 0;
    public double maxModuleTurnAccel = 0;
    public String moduleCanbus = "";
    public ModuleInfo[] moduleInfos = new ModuleInfo[]{};

    public SwerveSpeedLimits standardSpeedLimits = new SwerveSpeedLimits(0, 0, 0, 0);
    public PIDGains autoThetaGains = new PIDGains();
    public PIDGains translationGains = new PIDGains();
    
    public CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();

    //Just construct this to something random to avoid any potential null issues
    public Subsystem subsystem = new Subsystem() {};

    public BooleanSupplier flipTrajectory = () -> false;

    //It's fine that this value defaults to null (won't cause errors)
    //Standard deviations (basically a trust matrix) for the drive modules and gryo odometry
    public Matrix<N3, N1> standardDeviations = null;
    public double loopPeriod = 0.05;
    

    public SwerveDriveConfig() {}

    public void setModuleInfos(ModuleInfo... moduleInfos) {
        this.moduleInfos = moduleInfos;
    }

}
