package API_ROS2_Sunrise;

import java.io.IOException;
import java.net.Socket;
import java.util.ArrayList;
import com.kuka.task.ITaskLogger;
import javax.inject.Inject;

public class CheckOpenPorts {

    @Inject
    private ITaskLogger logger;

	public ArrayList<String> check_open_port() {
        ArrayList<String> portlist = new ArrayList<String>();
        Socket socket = null;

        try {
            logger.info("Following ports are open: \n");
            String s = "LBR command port (30005): ";
            socket = new Socket("127.0.0.1", 30005);
            s += "Open";
            logger.info(s);
            socket.close();
            portlist.add("LBRPort");

            s = "LBR status port (30001): ";
            socket = new Socket("127.0.0.1", 30001);
            s += "Open";
            logger.info(s);
            socket.close();
            portlist.add("LBRStatus");

            s = "LBR sensor port (30000): ";
            socket = new Socket("127.0.0.1", 30000);
            s += "Open";
            logger.info(s);
            socket.close();
            portlist.add("LBRSensor");

            s = "KMP command port (30002): ";
            socket = new Socket("127.0.0.1", 30002);
            s += "Open";
            logger.info(s);
            socket.close();
            portlist.add("KmpCommand");

            s = "KMP status port (30003): ";
            socket = new Socket("127.0.0.1", 30003);
            s += "Open";
            logger.info(s);
            socket.close();
            portlist.add("KmpStatus");

            s = "KMP laser port (30004): ";
            socket = new Socket("127.0.0.1", 30004);
            s += "Open";
            logger.info(s);
            socket.close();
            portlist.add("KmpLaser");

            s = "KMP odometry port (30006): ";
            socket = new Socket("127.0.0.1", 30006);
            s += "Open";
            logger.info(s);
            socket.close();
            portlist.add("KmpOdom");

        } catch (IOException e1) {
            String s = "Could not connect to all ports";
            logger.error(s);
        }

        return portlist;
    }
}


