package API_ROS2_Sunrise;


// RoboticsAPI
import com.kuka.jogging.provider.api.common.ICartesianJoggingSupport;
import com.kuka.roboticsAPI.deviceModel.kmp.SunriseOmniMoveMobilePlatform;

// Java Util
import java.util.TimerTask;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class KMPjogger
{
  private long JOG_UPDATE_PERIOD = 50L;

  public ScheduledExecutorService _executor;

  private ICartesianJoggingSupport _joggableDevice;
  
  private double[] _velocities;

  
  public KMPjogger( ICartesianJoggingSupport joggableDevice, long updateperiod) {
	this._velocities = new double[3];
    this._joggableDevice = joggableDevice;
    this.JOG_UPDATE_PERIOD = updateperiod;
  }

  public KMPjogger( ICartesianJoggingSupport joggableDevice) {
	this._velocities = new double[3];
    this._joggableDevice = joggableDevice;
  }
  
  public class JogTimerTask extends TimerTask {
    private int executionCount = 0;
    
    public JogTimerTask() {
        LogUtil.logInfo("KMPjogger: JogTimerTask created");
    }
    
    public void run() {
        executionCount++;
        boolean isReady = true;
        
        // Check if device is still valid before attempting to use it
        if (KMPjogger.this._joggableDevice instanceof SunriseOmniMoveMobilePlatform) {
            try {
                SunriseOmniMoveMobilePlatform device = (SunriseOmniMoveMobilePlatform)KMPjogger.this._joggableDevice;
                
                // Check controller validity without using isRegisteredInContext
                try {
                    // Try accessing controller methods - this will throw exception if disposed
                    device.getController().getName();
                    // If we get here, controller is still valid
                } catch (Exception e) {
                    LogUtil.logInfo("KMPjogger: Controller appears to be disposed or invalid: " + e.getMessage());
                    KMPjogger.this._executor.shutdownNow();
                    return;
                }
                
                // Check if the device is ready to move
                isReady = device.isReadyToMove();
                
                if (!isReady) {
                    LogUtil.logInfo("KMPjogger: Device no longer ready to move, shutting down executor");
                    KMPjogger.this._executor.shutdown();
                    return;
                }
            } catch (Exception e) {
                LogUtil.logInfo("KMPjogger: Exception checking device state: " + e.getMessage());
                KMPjogger.this._executor.shutdownNow();
                return;
            }
        }
        
        if (executionCount % 20 == 0) { // Only log every 20th execution to avoid log flooding
            LogUtil.logInfo("KMPjogger: Sending jog command #" + executionCount + 
                            " - x: " + KMPjogger.this._velocities[0] + 
                            ", y: " + KMPjogger.this._velocities[1] + 
                            ", theta: " + KMPjogger.this._velocities[2] + 
                            ", isReady: " + isReady);
        }
        
        try {
            KMPjogger.this._joggableDevice.jog(KMPjogger.this._velocities);
        } catch (Exception e) {
            LogUtil.logInfo("KMPjogger: ERROR during jog: " + e.getMessage());
            e.printStackTrace();
            
            // If we get a controller disposed exception, shut down the executor immediately
            if (e.getMessage() != null && e.getMessage().contains("disposed")) {
                LogUtil.logInfo("KMPjogger: Controller disposed error detected, shutting down executor");
                KMPjogger.this._executor.shutdownNow();
            }
        }
    }
  }


  public void stopDevice() {
    LogUtil.logInfo("KMPjogger: Stopping device by setting zero velocities");
    for (int i = 0; i < this._velocities.length; i++) {
        this._velocities[i] = 0.0D;
    }

    boolean isReady = false;
    if (_joggableDevice instanceof SunriseOmniMoveMobilePlatform) {
        isReady = ((SunriseOmniMoveMobilePlatform)_joggableDevice).isReadyToMove();
        LogUtil.logInfo("KMPjogger: Device isReadyToMove=" + isReady);
    }

    if(isReady) {
        LogUtil.logInfo("KMPjogger: Sending zero jog command");
        this._joggableDevice.jog(this._velocities);
    } else {
        LogUtil.logInfo("KMPjogger: Not ready to move, can't send zero jog command");
    }
  }
  
  
  public void startJoggingExecution() {
    LogUtil.logInfo("KMPjogger: Attempting to start jogging execution");
    if (this._executor == null || this._executor.isShutdown()) {
        LogUtil.logInfo("KMPjogger: Creating new executor");
        this._executor = Executors.newScheduledThreadPool(2);
    } else {
        LogUtil.logInfo("KMPjogger: Reusing existing executor");
    }
    
    LogUtil.logInfo("KMPjogger: Starting jog task with period " + this.JOG_UPDATE_PERIOD + "ms");
    this._executor.scheduleAtFixedRate(new JogTimerTask(), 
        0L, 
        this.JOG_UPDATE_PERIOD, 
        TimeUnit.MILLISECONDS);
    LogUtil.logInfo("KMPjogger: Jogging task scheduled successfully");
  }


  public void killJoggingExecution(boolean ismoving) {
    LogUtil.logInfo("KMPjogger: Stopping jogging execution, isMoving=" + ismoving);
    
    if (this._executor != null) {
        try {
            LogUtil.logInfo("KMPjogger: Shutting down executor");
            this._executor.shutdown();
            
            // Wait for the executor to terminate
            if (!this._executor.awaitTermination(2, TimeUnit.SECONDS)) {
                LogUtil.logInfo("KMPjogger: Forcing executor shutdown since tasks did not terminate");
                this._executor.shutdownNow();
                if (!this._executor.awaitTermination(2, TimeUnit.SECONDS)) {
                    LogUtil.logInfo("KMPjogger: Executor still did not terminate");
                }
            }
            LogUtil.logInfo("KMPjogger: Executor shutdown complete");
        } catch(Exception e) {
            LogUtil.logInfo("KMPjogger: ERROR - Could not stop executor: " + e.getMessage());
            e.printStackTrace();
            // Try forced shutdown in case of exception
            if (this._executor != null) {
                this._executor.shutdownNow();
            }
        }
        // Set to null after shutdown to avoid reuse of terminated executor
        this._executor = null;
    } else {
        LogUtil.logInfo("KMPjogger: Executor was already null");
    }
    
    if(ismoving) {
        LogUtil.logInfo("KMPjogger: Stopping device since it was moving");
        stopDevice();
    } else {
        LogUtil.logInfo("KMPjogger: Device was not moving, no need to stop");
    }
    
    // Important: this method should NOT set any shutdown flags in the commander
    LogUtil.logInfo("KMPjogger: Jogger stopped without triggering commander shutdown");
  }
  
  
  public void updateVelocities(double[] vel) {
    LogUtil.logInfo("KMPjogger: Updating velocities - x: " + vel[0] + ", y: " + vel[1] + ", theta: " + vel[2]);
    this._velocities = vel;
  } 
}