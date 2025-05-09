package API_ROS2_Sunrise;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.Executors;

/**
 * Base class for command-based nodes handling socket I/O and lifecycle.
 */
public abstract class AbstractCommander<T extends ISocket> extends Node {
    protected final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
    protected volatile boolean running = true;
    protected T socketClient;

    protected AbstractCommander(int port, String connectionType, String nodeName) {
        super(port, connectionType, nodeName);
        // initialize the socket client reference
        @SuppressWarnings("unchecked")
        T client = (T) getSocket();
        this.socketClient = client;
    }

    /**
     * Establish or re-establish the socket connection.
     */
    protected abstract void connect();

    /**
     * Cleanly close or disconnect the socket.
     */
    protected abstract void disconnect();

    /**
     * Single iteration of processing loop (e.g. receive_message + handle logic).
     * Should throw exception on unrecoverable errors.
     */
    protected abstract void runLoop() throws Exception;

    @Override
    public void run() {
        try {
            connect();
            while (running) {
                runLoop();
            }
        } catch (Exception e) {
            if (logger != null) logger.error(nodename + ": Error in commander loop", e);
        } finally {
            disconnect();
            scheduler.shutdownNow();
        }
    }

    /**
     * Stops the processing loop and shuts down scheduler.
     */
    public void shutdown() {
        running = false;
        scheduler.shutdownNow();
    }
}